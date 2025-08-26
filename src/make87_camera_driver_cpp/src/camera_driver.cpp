#include <chrono>
#include <memory>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <thread>
#include <vector>
#include <cstring>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <regex>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#ifdef ENABLE_JPEG_COMPRESSION
#include "sensor_msgs/msg/compressed_image.hpp"
#endif
#include <nlohmann/json.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

using namespace std::chrono_literals;

// URL encode function for username/password in RTSP URLs
std::string url_encode(const std::string& value) {
    std::ostringstream escaped;
    escaped.fill('0');
    escaped << std::hex;

    for (char c : value) {
        // Keep alphanumeric and these safe characters
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            escaped << c;
        } else {
            // Percent-encode everything else
            escaped << std::uppercase;
            escaped << '%' << std::setw(2) << int(static_cast<unsigned char>(c));
            escaped << std::nouppercase;
        }
    }

    return escaped.str();
}

// Secret resolution function - resolves {{ secret.NAME }} patterns
std::string resolve_secret_string(const std::string& input) {
    static const std::regex secret_pattern(R"(^\s*\{\{\s*secret\.([A-Za-z0-9_]+)\s*}}\s*$)");
    std::smatch matches;
    
    if (std::regex_match(input, matches, secret_pattern)) {
        std::string secret_name = matches[1].str();
        std::string secret_path = "/run/secrets/" + secret_name + ".secret";
        
        std::ifstream secret_file(secret_path);
        if (!secret_file.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to read secret file: %s", secret_path.c_str());
            return input; // Return original string if secret can't be read
        }
        
        std::string secret_value;
        std::getline(secret_file, secret_value);
        
        // Trim whitespace
        secret_value.erase(0, secret_value.find_first_not_of(" \t\n\r"));
        secret_value.erase(secret_value.find_last_not_of(" \t\n\r") + 1);
        
        RCLCPP_DEBUG(rclcpp::get_logger("make87_camera_driver"),
                    "Resolved secret %s from %s", secret_name.c_str(), secret_path.c_str());
        
        return secret_value;
    }
    
    return input; // Return original string if no secret pattern found
}

// Recursively resolve secrets in nlohmann::json
nlohmann::json resolve_secrets(const nlohmann::json& value) {
    if (value.is_object()) {
        nlohmann::json resolved_obj = nlohmann::json::object();
        for (auto& [key, val] : value.items()) {
            resolved_obj[key] = resolve_secrets(val);
        }
        return resolved_obj;
    } else if (value.is_array()) {
        nlohmann::json resolved_arr = nlohmann::json::array();
        for (const auto& item : value) {
            resolved_arr.push_back(resolve_secrets(item));
        }
        return resolved_arr;
    } else if (value.is_string()) {
        return resolve_secret_string(value.get<std::string>());
    } else {
        return value; // Return other types (numbers, booleans, null) unchanged
    }
}

struct CameraConfig {
    std::string camera_name;
    std::string camera_ip;
    int camera_port;
    std::string camera_path;
    std::string camera_username;
    std::string camera_password;
};

CameraConfig parse_camera_config() {
    const char* config_env = std::getenv("MAKE87_CONFIG");
    CameraConfig camera_config;

    if (!config_env) {
        RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                   "MAKE87_CONFIG not found, no camera will be configured");
        return camera_config;
    }

    try {
        auto parsed = nlohmann::json::parse(config_env);
        
        // Resolve secrets in the entire JSON structure
        parsed = resolve_secrets(parsed);
        
        // The camera config is nested under the "config" key
        if (!parsed.contains("config")) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "MAKE87_CONFIG does not contain 'config' section");
            return camera_config;
        }
        
        auto config = parsed["config"];

        camera_config.camera_ip = config.value("camera_ip", "");
        camera_config.camera_name = config.value("camera_name", camera_config.camera_ip);
        camera_config.camera_port = config.value("camera_port", 554);
        camera_config.camera_path = config.value("camera_path", "/");
        camera_config.camera_username = config.value("camera_username", "");
        camera_config.camera_password = config.value("camera_password", "");

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                    "Error parsing MAKE87_CONFIG JSON: %s", e.what());
    }

    return camera_config;
}

std::string get_topic_name() {
    const char* config_env = std::getenv("MAKE87_CONFIG");
    const std::string default_topic = "camera_frames";

    if (!config_env) {
        RCLCPP_INFO(rclcpp::get_logger("make87_camera_driver"),
                   "MAKE87_CONFIG not found, using default topic: %s", default_topic.c_str());
        return default_topic;
    }

    try {
        auto config = nlohmann::json::parse(config_env);

        // Navigate to interfaces.ros.publishers.camera_frames.topic_key
        if (config.contains("interfaces") &&
            config["interfaces"].contains("ros") &&
            config["interfaces"]["ros"].contains("publishers") &&
            config["interfaces"]["ros"]["publishers"].contains("camera_frames") &&
            config["interfaces"]["ros"]["publishers"]["camera_frames"].contains("topic_key")) {

            std::string topic_key = config["interfaces"]["ros"]["publishers"]["camera_frames"]["topic_key"];

            // Replace all dashes with underscores
            std::string sanitized_topic = topic_key;
            std::replace(sanitized_topic.begin(), sanitized_topic.end(), '-', '_');

            std::string prefixed_topic = "make87_" + sanitized_topic;

            RCLCPP_INFO(rclcpp::get_logger("make87_camera_driver"),
                       "Extracted topic_key from MAKE87_CONFIG: %s", prefixed_topic.c_str());
            return prefixed_topic;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("make87_camera_driver"),
                       "Could not find camera_frames publisher topic_key in MAKE87_CONFIG, using default: %s",
                       default_topic.c_str());
            return default_topic;
        }
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                    "Error parsing MAKE87_CONFIG JSON: %s, using default topic: %s",
                    e.what(), default_topic.c_str());
        return default_topic;
    }
}

class FFmpegCameraDriver {
public:
    FFmpegCameraDriver(const CameraConfig& config) : config_(config) {
        // Initialize FFmpeg
        avformat_network_init();
    }

    ~FFmpegCameraDriver() {
        cleanup();
        avformat_network_deinit();
    }

    bool initialize() {
        std::string rtsp_url = build_rtsp_url(config_);
        RCLCPP_INFO(rclcpp::get_logger("make87_camera_driver"),
                   "Connecting to camera: %s", rtsp_url.c_str());

        // Allocate format context
        format_ctx_ = avformat_alloc_context();
        if (!format_ctx_) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to allocate AVFormatContext");
            return false;
        }

        // Set options for RTSP
        AVDictionary* options = nullptr;
        av_dict_set(&options, "rtsp_transport", "tcp", 0);
        av_dict_set(&options, "timeout", "5000000", 0); // 5 seconds in microseconds
        av_dict_set(&options, "allowed_media_types", "video", 0);
        av_dict_set(&options, "flags", "low_delay", 0);
        av_dict_set(&options, "fflags", "+discardcorrupt", 0);

        // Open input stream
        if (avformat_open_input(&format_ctx_, rtsp_url.c_str(), nullptr, &options) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to open input stream: %s", rtsp_url.c_str());
            av_dict_free(&options);
            return false;
        }
        av_dict_free(&options);

        // Find stream info
        if (avformat_find_stream_info(format_ctx_, nullptr) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to find stream info");
            return false;
        }

        // Find video stream
        video_stream_index_ = av_find_best_stream(format_ctx_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
        if (video_stream_index_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to find video stream");
            return false;
        }

        // Get codec parameters
        AVCodecParameters* codec_params = format_ctx_->streams[video_stream_index_]->codecpar;

        // Find decoder
        const AVCodec* codec = avcodec_find_decoder(codec_params->codec_id);
        if (!codec) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to find codec");
            return false;
        }

        // Allocate codec context
        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to allocate codec context");
            return false;
        }

        // Copy codec parameters to context
        if (avcodec_parameters_to_context(codec_ctx_, codec_params) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to copy codec parameters");
            return false;
        }

        // Open codec
        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to open codec");
            return false;
        }

        // Allocate frame and packet
        frame_ = av_frame_alloc();
        target_frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();

        if (!frame_ || !target_frame_ || !packet_) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to allocate frame/packet");
            return false;
        }

#ifdef ENABLE_JPEG_COMPRESSION
        // For JPEG compression, keep original format or convert to YUV420P for better compression
        target_pix_fmt_ = AV_PIX_FMT_YUVJ420P;  // JPEG-compatible YUV format
#else
        // For RGB8 output, convert to RGB24
        target_pix_fmt_ = AV_PIX_FMT_RGB24;
#endif

        // Initialize scaling context
        sws_ctx_ = sws_getContext(
            codec_ctx_->width, codec_ctx_->height, codec_ctx_->pix_fmt,
            codec_ctx_->width, codec_ctx_->height, target_pix_fmt_,
            SWS_BILINEAR, nullptr, nullptr, nullptr
        );

        if (!sws_ctx_) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to initialize scaling context");
            return false;
        }

        // Allocate buffer for target format frame
        int buffer_size = av_image_get_buffer_size(target_pix_fmt_, codec_ctx_->width, codec_ctx_->height, 1);
        frame_buffer_ = (uint8_t*)av_malloc(buffer_size);
        if (!frame_buffer_) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "Failed to allocate frame buffer");
            return false;
        }

        // Setup target format frame
        av_image_fill_arrays(target_frame_->data, target_frame_->linesize,
                           frame_buffer_, target_pix_fmt_,
                           codec_ctx_->width, codec_ctx_->height, 1);

        width_ = codec_ctx_->width;
        height_ = codec_ctx_->height;

        RCLCPP_INFO(rclcpp::get_logger("make87_camera_driver"),
                   "Successfully initialized FFmpeg camera driver (%dx%d)", width_, height_);

        return true;
    }

    bool read_frame(std::vector<uint8_t>& output_data) {
        while (true) {
            int ret = av_read_frame(format_ctx_, packet_);
            if (ret < 0) {
                if (ret == AVERROR_EOF) {
                    RCLCPP_WARN(rclcpp::get_logger("make87_camera_driver"),
                               "End of stream reached");
                } else {
                    char error_buf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, error_buf, sizeof(error_buf));
                    RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                                "Error reading frame: %s", error_buf);
                }
                return false;
            }

            if (packet_->stream_index != video_stream_index_) {
                av_packet_unref(packet_);
                continue;
            }

            ret = avcodec_send_packet(codec_ctx_, packet_);
            av_packet_unref(packet_);

            if (ret < 0) {
                char error_buf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, error_buf, sizeof(error_buf));
                RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                            "Error sending packet to decoder: %s", error_buf);
                continue;
            }

            ret = avcodec_receive_frame(codec_ctx_, frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                continue;
            } else if (ret < 0) {
                char error_buf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, error_buf, sizeof(error_buf));
                RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                            "Error receiving frame from decoder: %s", error_buf);
                continue;
            }

            // Convert frame to target format
            sws_scale(sws_ctx_,
                     (const uint8_t* const*)frame_->data, frame_->linesize,
                     0, codec_ctx_->height,
                     target_frame_->data, target_frame_->linesize);

#ifdef ENABLE_JPEG_COMPRESSION
            // Encode as JPEG
            if (!encode_jpeg(output_data)) {
                RCLCPP_WARN(rclcpp::get_logger("make87_camera_driver"),
                           "Failed to encode JPEG, retrying...");
                continue;
            }
#else
            // Copy RGB24 data to output vector
            int rgb_size = width_ * height_ * 3; // 3 bytes per pixel (RGB)
            output_data.resize(rgb_size);

            // Copy RGB data (single plane)
            memcpy(output_data.data(), target_frame_->data[0], rgb_size);
#endif

            return true;
        }
    }

    int get_width() const { return width_; }
    int get_height() const { return height_; }

#ifdef ENABLE_JPEG_COMPRESSION
    bool encode_jpeg(std::vector<uint8_t>& jpeg_data) {
        // Initialize JPEG encoder if not already done
        if (!jpeg_encoder_ctx_) {
            if (!init_jpeg_encoder()) {
                return false;
            }
        }

        // Create frame for JPEG encoder
        AVFrame* jpeg_frame = av_frame_alloc();
        if (!jpeg_frame) {
            return false;
        }

        jpeg_frame->format = target_pix_fmt_;
        jpeg_frame->width = width_;
        jpeg_frame->height = height_;
        
        // Copy data from target_frame to jpeg_frame
        av_frame_copy(jpeg_frame, target_frame_);

        // Send frame to encoder
        int ret = avcodec_send_frame(jpeg_encoder_ctx_, jpeg_frame);
        av_frame_free(&jpeg_frame);
        
        if (ret < 0) {
            return false;
        }

        // Receive encoded packet
        AVPacket* jpeg_packet = av_packet_alloc();
        ret = avcodec_receive_packet(jpeg_encoder_ctx_, jpeg_packet);
        
        if (ret == 0) {
            // Copy JPEG data
            jpeg_data.resize(jpeg_packet->size);
            memcpy(jpeg_data.data(), jpeg_packet->data, jpeg_packet->size);
            av_packet_free(&jpeg_packet);
            return true;
        }
        
        av_packet_free(&jpeg_packet);
        return false;
    }

    bool init_jpeg_encoder() {
        const AVCodec* jpeg_codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
        if (!jpeg_codec) {
            RCLCPP_ERROR(rclcpp::get_logger("make87_camera_driver"),
                        "MJPEG encoder not found");
            return false;
        }

        jpeg_encoder_ctx_ = avcodec_alloc_context3(jpeg_codec);
        if (!jpeg_encoder_ctx_) {
            return false;
        }

        jpeg_encoder_ctx_->width = width_;
        jpeg_encoder_ctx_->height = height_;
        jpeg_encoder_ctx_->pix_fmt = target_pix_fmt_;
        jpeg_encoder_ctx_->time_base = {1, 30}; // 30 FPS timebase
        jpeg_encoder_ctx_->qmin = 10;
        jpeg_encoder_ctx_->qmax = 63;

        if (avcodec_open2(jpeg_encoder_ctx_, jpeg_codec, nullptr) < 0) {
            avcodec_free_context(&jpeg_encoder_ctx_);
            return false;
        }

        return true;
    }
#endif

private:
    std::string build_rtsp_url(const CameraConfig& config) {
        std::string url = "rtsp://";

        if (!config.camera_username.empty() && !config.camera_password.empty()) {
            // URL encode username and password to handle special characters
            std::string encoded_username = url_encode(config.camera_username);
            std::string encoded_password = url_encode(config.camera_password);
            url += encoded_username + ":" + encoded_password + "@";
        }

        url += config.camera_ip + ":" + std::to_string(config.camera_port);

        if (!config.camera_path.empty()) {
            if (config.camera_path[0] != '/') {
                url += "/";
            }
            url += config.camera_path;
        }

        return url;
    }

    void cleanup() {
        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }

        if (frame_buffer_) {
            av_free(frame_buffer_);
            frame_buffer_ = nullptr;
        }

        if (frame_) {
            av_frame_free(&frame_);
        }

        if (target_frame_) {
            av_frame_free(&target_frame_);
        }

#ifdef ENABLE_JPEG_COMPRESSION
        if (jpeg_encoder_ctx_) {
            avcodec_free_context(&jpeg_encoder_ctx_);
        }
#endif

        if (packet_) {
            av_packet_free(&packet_);
        }

        if (codec_ctx_) {
            avcodec_free_context(&codec_ctx_);
        }

        if (format_ctx_) {
            avformat_close_input(&format_ctx_);
        }
    }

    CameraConfig config_;
    AVFormatContext* format_ctx_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVFrame* target_frame_ = nullptr;
    AVPacket* packet_ = nullptr;
    uint8_t* frame_buffer_ = nullptr;
    AVPixelFormat target_pix_fmt_;
#ifdef ENABLE_JPEG_COMPRESSION
    AVCodecContext* jpeg_encoder_ctx_ = nullptr;
#endif
    int video_stream_index_ = -1;
    int width_ = 0;
    int height_ = 0;
};

class CameraDriver : public rclcpp::Node {
public:
  CameraDriver() : rclcpp::Node("make87_camera_driver") {
    std::string topic_name = get_topic_name();
    CameraConfig camera_config = parse_camera_config();

    if (camera_config.camera_ip.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No camera IP configured. Please check MAKE87_CONFIG.");
        return;
    }

    // Initialize FFmpeg camera driver
    ffmpeg_driver_ = std::make_unique<FFmpegCameraDriver>(camera_config);

    if (!ffmpeg_driver_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera driver");
        return;
    }

#ifdef ENABLE_JPEG_COMPRESSION
    compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name, 10);
    RCLCPP_INFO(this->get_logger(), "make87_camera_driver started; publishing JPEG compressed images to '%s'", topic_name.c_str());
#else
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    RCLCPP_INFO(this->get_logger(), "make87_camera_driver started; publishing RGB8 images to '%s'", topic_name.c_str());
#endif

    // Start capture thread
    capture_thread_ = std::thread(&CameraDriver::capture_loop, this);
  }

  ~CameraDriver() {
    should_stop_ = true;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
  }

private:
  void capture_loop() {
    std::vector<uint8_t> frame_data;

    while (!should_stop_ && rclcpp::ok()) {
        if (!ffmpeg_driver_->read_frame(frame_data)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame, retrying...");
            std::this_thread::sleep_for(10ms);
            continue;
        }

#ifdef ENABLE_JPEG_COMPRESSION
        // Create ROS CompressedImage message
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        msg->format = "jpeg";
        msg->data = frame_data;

        compressed_pub_->publish(std::move(msg));
        RCLCPP_DEBUG(this->get_logger(), "Published JPEG frame (%zu bytes)", frame_data.size());
#else
        // Create ROS Image message
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        msg->height = ffmpeg_driver_->get_height();
        msg->width = ffmpeg_driver_->get_width();
        msg->encoding = sensor_msgs::image_encodings::RGB8;
        msg->is_bigendian = false;
        msg->step = ffmpeg_driver_->get_width() * 3; // RGB8: 3 bytes per pixel
        msg->data = frame_data;

        // Get values before move
        auto width = msg->width;
        auto height = msg->height;
        
        image_pub_->publish(std::move(msg));
        RCLCPP_DEBUG(this->get_logger(), "Published RGB8 frame %dx%d (%zu bytes)",
                    width, height, frame_data.size());
#endif
    }
  }

#ifdef ENABLE_JPEG_COMPRESSION
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
#else
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
#endif
  std::unique_ptr<FFmpegCameraDriver> ffmpeg_driver_;
  std::thread capture_thread_;
  std::atomic<bool> should_stop_{false};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriver>());
  rclcpp::shutdown();
  return 0;
}
