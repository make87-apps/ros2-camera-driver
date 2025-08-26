#!/bin/bash

#==============================================================================
# ROS2 Entrypoint Script
#==============================================================================
# Description: Entrypoint script for ROS2 Jazzy talker application
# Author: Development Team
# Version: 1.0
# Usage: ./entrypoint.sh
#==============================================================================

set -euo pipefail  # Exit on error, undefined vars, pipe failures
IFS=$'\n\t'        # Secure Internal Field Separator

#==============================================================================
# CONFIGURATION
#==============================================================================

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_DIR="${PROJECT_DIR:-/home/state/code}"
readonly ROS_ENTRYPOINT="${ROS_ENTRYPOINT:-/ros_entrypoint.sh}"
readonly SETUP_SCRIPT="${SETUP_SCRIPT:-install/setup.bash}"
readonly PACKAGE_NAME="${PACKAGE_NAME:-make87_camera_driver_cpp}"
readonly NODE_NAME="${NODE_NAME:-camera_driver}"

# Zenoh configuration
readonly ZENOH_LISTEN_PORT=7447
readonly ZENOH_LISTEN_HOST="0.0.0.0"

# Development environment detection
readonly IS_DEV_ENV="${IS_DEV_ENV:-false}"

#==============================================================================
# LOGGING FUNCTIONS
#==============================================================================

log_info() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [INFO] $*" >&2
}

log_warn() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [WARN] $*" >&2
}

log_error() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [ERROR] $*" >&2
}

log_fatal() {
    log_error "$*"
    exit 1
}

#==============================================================================
# VALIDATION FUNCTIONS
#==============================================================================

validate_environment() {
    log_info "Validating environment..."

    # Check if project directory exists
    if [[ ! -d "${PROJECT_DIR}" ]]; then
        log_fatal "Project directory does not exist: ${PROJECT_DIR}"
    fi

    # Check if ROS entrypoint exists
    if [[ ! -f "${ROS_ENTRYPOINT}" ]]; then
        log_fatal "ROS entrypoint script not found: ${ROS_ENTRYPOINT}"
    fi

    # Check if ROS entrypoint is executable
    if [[ ! -x "${ROS_ENTRYPOINT}" ]]; then
        log_fatal "ROS entrypoint script is not executable: ${ROS_ENTRYPOINT}"
    fi

    log_info "Environment validation passed"
}

validate_ros_workspace() {
    log_info "Validating ROS2 workspace..."

    # Check if setup script exists
    if [[ ! -f "${PROJECT_DIR}/${SETUP_SCRIPT}" ]]; then
        log_fatal "ROS2 setup script not found: ${PROJECT_DIR}/${SETUP_SCRIPT}"
    fi

    log_info "ROS2 workspace validation passed"
}

validate_git_environment() {
    log_info "Validating git environment for development..."

    # Only validate git in development environment
    if [[ "${IS_DEV_ENV}" != "true" ]]; then
        log_info "Not in development environment, skipping git validation"
        return 0
    fi

    # Check if git is available
    if ! command -v git &> /dev/null; then
        log_fatal "git is required for development environment but is not installed"
    fi

    # Check if GIT_URL is provided
    if [[ -z "${GIT_URL:-}" ]]; then
        log_fatal "GIT_URL environment variable is required for development environment"
    fi

    log_info "Git environment validation passed"
}

validate_make87_config() {
    log_info "Validating MAKE87_CONFIG environment variable..."

    if [[ -z "${MAKE87_CONFIG:-}" ]]; then
        log_fatal "MAKE87_CONFIG environment variable is not set"
    fi

    # Validate that jq is available for JSON parsing
    if ! command -v jq &> /dev/null; then
        log_fatal "jq is required for parsing MAKE87_CONFIG but is not installed"
    fi

    # Validate that MAKE87_CONFIG contains valid JSON
    if ! echo "${MAKE87_CONFIG}" | jq . &> /dev/null; then
        log_fatal "MAKE87_CONFIG does not contain valid JSON"
    fi

    log_info "MAKE87_CONFIG validation passed"
}

check_port_in_use() {
    local port=$1

    # Check if port is in use using netstat or ss
    if command -v ss &> /dev/null; then
        ss -tuln | grep -q ":${port} "
    elif command -v netstat &> /dev/null; then
        netstat -tuln | grep -q ":${port} "
    else
        # Fallback: assume port is not in use if we can't check
        log_warn "Neither 'ss' nor 'netstat' available, assuming port ${port} is free"
        return 1
    fi
}

generate_zenoh_endpoints() {
    log_info "Generating Zenoh endpoints from MAKE87_CONFIG..."

    local listen_endpoints=""
    local connect_endpoints=""

    # Generate listen endpoint if port is not in use
    if ! check_port_in_use "${ZENOH_LISTEN_PORT}"; then
        listen_endpoints="[\"tcp/${ZENOH_LISTEN_HOST}:${ZENOH_LISTEN_PORT}\"]"
        log_info "Port ${ZENOH_LISTEN_PORT} is available, adding listen endpoint"
    else
        listen_endpoints="[]"
        log_warn "Port ${ZENOH_LISTEN_PORT} is in use, no listen endpoint will be configured"
    fi

    # Extract VPN IPs and ports from subscribers and requesters across all interfaces
    local vpn_endpoints
    vpn_endpoints=$(echo "${MAKE87_CONFIG}" | jq -r '
        [
            (.interfaces // {} | to_entries[] | .value.subscribers // {} | to_entries[] | .value | select(.vpn_ip and .vpn_port) | "tcp/\(.vpn_ip):\(.vpn_port)"),
            (.interfaces // {} | to_entries[] | .value.requesters // {} | to_entries[] | .value | select(.vpn_ip and .vpn_port) | "tcp/\(.vpn_ip):\(.vpn_port)")
        ] | unique | @json
    ')

    if [[ "${vpn_endpoints}" != "null" && "${vpn_endpoints}" != "[]" ]]; then
        connect_endpoints="${vpn_endpoints}"
        log_info "Found VPN endpoints for connection: ${connect_endpoints}"
    else
        connect_endpoints="[]"
        log_info "No VPN endpoints found in MAKE87_CONFIG"
    fi

    # Export the configuration
    export ZENOH_LISTEN_ENDPOINTS="${listen_endpoints}"
    export ZENOH_CONNECT_ENDPOINTS="${connect_endpoints}"

    log_info "Generated Zenoh endpoints:"
    log_info "  Listen endpoints: ${ZENOH_LISTEN_ENDPOINTS}"
    log_info "  Connect endpoints: ${ZENOH_CONNECT_ENDPOINTS}"
}

#==============================================================================
# GIT SETUP FUNCTIONS (Development Only)
#==============================================================================

setup_ssh_keys() {
    log_info "Setting up SSH keys..."

    # Ensure .ssh directory exists
    mkdir -p /root/.ssh
    chmod 700 /root/.ssh

    # Generate SSH key if it doesn't exist
    if [[ ! -f "/root/.ssh/id_ed25519" ]]; then
        ssh-keygen -t ed25519 -C "app@make87.com" -f "/root/.ssh/id_ed25519" -N ""
        log_info "Generated new SSH key"
    else
        log_info "SSH key already exists"
    fi

    # Copy authorized_keys if provided
    if [[ -f "/root/.ssh/authorized_keys_src" ]]; then
        cp /root/.ssh/authorized_keys_src /root/.ssh/authorized_keys
        chown root:root /root/.ssh/authorized_keys
        chmod 600 /root/.ssh/authorized_keys
        log_info "Copied authorized_keys from mounted source"
    fi
}

setup_git_config() {
    log_info "Setting up git configuration..."

    git config --global user.email "make87"
    git config --global user.name "user"

    log_info "Git configuration completed"
}

clone_or_update_repository() {
    log_info "Setting up git repository..."

    local target_dir="${PROJECT_DIR}"
    local git_url="${GIT_URL}"
    local git_branch="${GIT_BRANCH:-}"

    if [[ ! -d "${target_dir}" ]]; then
        log_info "Cloning repository from ${git_url} to ${target_dir}"

        if ! git clone "${git_url}" "${target_dir}"; then
            log_fatal "Failed to clone repository from ${git_url}"
        fi

        cd "${target_dir}" || log_fatal "Failed to change to ${target_dir}"

        # Checkout specified branch if provided
        if [[ -n "${git_branch}" ]]; then
            log_info "Checking out branch: ${git_branch}"
            if ! git checkout "${git_branch}"; then
                log_fatal "Failed to checkout branch: ${git_branch}"
            fi
        fi
    else
        log_info "Repository directory exists, updating..."

        cd "${target_dir}" || log_fatal "Failed to change to ${target_dir}"

        # Check if this is actually a git repository
        if ! git rev-parse --git-dir > /dev/null 2>&1; then
            log_warn "Directory exists but is not a git repository. Initializing git repository..."
            git init
            git remote add origin "${git_url}"
        else
            # Set remote URL
            git remote set-url origin "${git_url}"
        fi

        # Handle branch switching if specified
        if [[ -n "${git_branch}" ]]; then
            local current_branch
            # Only get current branch if we have commits (not a fresh repo)
            if git rev-parse --verify HEAD > /dev/null 2>&1; then
                current_branch=$(git rev-parse --abbrev-ref HEAD)
            else
                current_branch="(no commits)"
            fi

            if [[ "${current_branch}" != "${git_branch}" ]]; then
                log_info "Current branch: ${current_branch}, target branch: ${git_branch}"

                # Stage and commit any changes (only if we have commits already)
                if [[ "${current_branch}" != "(no commits)" ]]; then
                    if ! git diff --quiet || ! git diff --cached --quiet; then
                        log_info "Staging and committing local changes..."
                        git add .
                        git commit -m "Saving changes before switching branch" || log_info "No changes to commit"

                        # Try to push changes
                        git push origin "${current_branch}" || log_warn "Failed to push changes to ${current_branch}"
                    fi
                fi

                # Switch to target branch
                log_info "Switching to branch: ${git_branch}"
                if ! git checkout "${git_branch}"; then
                    log_fatal "Failed to checkout branch: ${git_branch}"
                fi
            fi
        fi

        # Pull latest changes (only if we have a proper repository with commits)
        if git rev-parse --verify HEAD > /dev/null 2>&1; then
            local pull_branch="${git_branch:-$(git rev-parse --abbrev-ref HEAD)}"
            log_info "Pulling latest changes from branch: ${pull_branch}"
            git pull origin "${pull_branch}" || log_warn "Failed to pull latest changes"
        else
            log_info "Fresh repository detected, performing initial fetch..."
            git fetch origin || log_warn "Failed to fetch from origin"
            if [[ -n "${git_branch}" ]]; then
                git checkout -b "${git_branch}" "origin/${git_branch}" 2>/dev/null || \
                git checkout "${git_branch}" 2>/dev/null || \
                log_warn "Could not checkout branch ${git_branch}, staying on current branch"
            fi
        fi
    fi

    log_info "Repository setup completed"
}

setup_development_environment() {
    log_info "Setting up development environment..."

    # Only run development setup if in dev environment
    if [[ "${IS_DEV_ENV}" != "true" ]]; then
        log_info "Not in development environment, skipping dev setup"
        return 0
    fi

    setup_ssh_keys
    setup_git_config
    clone_or_update_repository

    log_info "Development environment setup completed"
}

build_ros_workspace() {
    log_info "Building ROS2 workspace..."

    # Only build in development environment or if source files exist
    if [[ "${IS_DEV_ENV}" == "true" ]] || [[ -d "${PROJECT_DIR}/src" ]]; then
        cd "${PROJECT_DIR}" || log_fatal "Failed to change to ${PROJECT_DIR}"

        # Clean install directory if it exists to avoid layout conflicts
        if [[ -d "install" ]]; then
            log_info "Removing existing install directory to avoid layout conflicts"
            rm -rf install
        fi

        # Source ROS environment
        # Temporarily disable undefined variable check for ROS sourcing
        set +u
        # shellcheck disable=SC1090
        source /opt/ros/${ROS_DISTRO}/setup.bash
        set -u  # Re-enable undefined variable check

        # Build the workspace
        log_info "Running colcon build..."
        
        # Determine build args based on ENABLE_JPEG_COMPRESSION environment variable
        CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release"
        if [[ "${ENABLE_JPEG_COMPRESSION:-OFF}" == "ON" ]]; then
            CMAKE_ARGS="${CMAKE_ARGS} -DENABLE_JPEG_COMPRESSION=ON"
            log_info "Building with JPEG compression enabled"
        else
            log_info "Building with RGB8 output (JPEG compression disabled)"
        fi
        
        if ! colcon build --merge-install --cmake-args ${CMAKE_ARGS}; then
            log_fatal "Failed to build ROS2 workspace"
        fi

        log_info "ROS2 workspace build completed"
    else
        log_info "No source directory found, skipping build"
    fi
}

#==============================================================================
# SETUP FUNCTIONS
#==============================================================================

change_to_project_directory() {
    log_info "Changing to project directory: ${PROJECT_DIR}"

    if ! cd "${PROJECT_DIR}"; then
        log_fatal "Failed to change to project directory: ${PROJECT_DIR}"
    fi

    log_info "Successfully changed to: $(pwd)"
}

source_ros_entrypoint() {
    log_info "Sourcing ROS entrypoint: ${ROS_ENTRYPOINT}"

    # Temporarily disable undefined variable check for ROS sourcing
    set +u
    # shellcheck disable=SC1090
    if ! source "${ROS_ENTRYPOINT}"; then
        set -u  # Re-enable undefined variable check
        log_fatal "Failed to source ROS entrypoint: ${ROS_ENTRYPOINT}"
    fi
    set -u  # Re-enable undefined variable check

    log_info "Successfully sourced ROS entrypoint"
}

source_workspace_setup() {
    log_info "Sourcing workspace setup: ${SETUP_SCRIPT}"

    # Temporarily disable undefined variable check for ROS sourcing
    set +u
    # shellcheck disable=SC1091
    if ! source "${SETUP_SCRIPT}"; then
        set -u  # Re-enable undefined variable check
        log_fatal "Failed to source workspace setup: ${SETUP_SCRIPT}"
    fi
    set -u  # Re-enable undefined variable check

    log_info "Successfully sourced workspace setup"
}

configure_zenoh() {
    log_info "Configuring Zenoh networking..."

    # Generate endpoints dynamically
    generate_zenoh_endpoints

    # Build the Zenoh configuration override
    export ZENOH_CONFIG_OVERRIDE="listen/endpoints=${ZENOH_LISTEN_ENDPOINTS};connect/endpoints=${ZENOH_CONNECT_ENDPOINTS}"

    log_info "Zenoh configuration override set: ${ZENOH_CONFIG_OVERRIDE}"
}

#==============================================================================
# CLEANUP FUNCTIONS
#==============================================================================

cleanup() {
    local exit_code=$?

    if [[ ${exit_code} -ne 0 ]]; then
        log_error "Script exited with error code: ${exit_code}"
    else
        log_info "Script completed successfully"
    fi

    # Perform any necessary cleanup here
    # (e.g., kill background processes, remove temp files)

    exit ${exit_code}
}

#==============================================================================
# SIGNAL HANDLERS
#==============================================================================

handle_interrupt() {
    log_warn "Received interrupt signal (SIGINT/SIGTERM)"
    log_info "Shutting down gracefully..."
    exit 130
}

#==============================================================================
# MAIN EXECUTION
#==============================================================================

main() {
    log_info "Starting ROS2 entrypoint script"
    log_info "Script: ${BASH_SOURCE[0]}"
    log_info "PID: $$"
    log_info "Environment: ${IS_DEV_ENV}"

    # Set up signal handlers
    trap cleanup EXIT
    trap handle_interrupt SIGINT SIGTERM

    # Validation phase
    validate_environment
    validate_git_environment
    validate_make87_config

    # Development environment setup
    setup_development_environment

    # Setup phase
    change_to_project_directory

    # Build workspace if in development
    build_ros_workspace

    validate_ros_workspace
    source_ros_entrypoint
    source_workspace_setup
    configure_zenoh

    # Launch ROS2 node
    log_info "Launching ROS2 node: ${PACKAGE_NAME}/${NODE_NAME}"
    log_info "Press Ctrl+C to stop the node"

    # Use exec to replace the shell process with the ROS2 node
    # This ensures proper signal handling and resource management
    exec ros2 run "${PACKAGE_NAME}" "${NODE_NAME}"
}

#==============================================================================
# SCRIPT ENTRY POINT
#==============================================================================

# Only run main if script is executed directly (not sourced)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
