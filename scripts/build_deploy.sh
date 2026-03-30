#!/bin/bash
# =============================================================================
# build_deploy.sh
#
# Builds a deploy Docker image for isaac_ros_visual_slam_orb.
# Uses the existing docker_deploy.sh infrastructure from isaac_ros_common.
#
# Prerequisites:
#   1. ORB-SLAM3 must be built inside the dev container (install_orb_slam3.sh)
#   2. The ROS workspace must be built: colcon build --packages-up-to isaac_ros_visual_slam_orb
#
# Usage:
#   bash build_deploy.sh                    # default settings
#   bash build_deploy.sh --build-libs       # also build ORB-SLAM3 libs (extract step)
#   bash build_deploy.sh --force            # force rebuild everything
#   bash build_deploy.sh --name my_image    # custom image name
#
# =============================================================================
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-/mnt/nova_ssd/workspaces/isaac_ros-dev}"
COMMON_SCRIPTS="${ISAAC_ROS_WS}/src/isaac_ros_common/scripts"
source "${COMMON_SCRIPTS}/utils/print_color.sh"

# ─── Defaults ────────────────────────────────────────────────────────────────
DEPLOY_IMAGE_NAME="isaac_ros_visual_slam_orb_deploy"
BASE_IMAGE_KEY="aarch64.ros2_humble.realsense"
LAUNCH_PACKAGE="isaac_ros_visual_slam_orb"
LAUNCH_FILE="isaac_ros_visual_slam_orb_realsense_d456.launch.py"
TARBALL_PATH="${ISAAC_ROS_WS}/orb_slam3_deploy_libs.tar.gz"
BUILD_LIBS=0
FORCE=0

# System debians needed at runtime for ORB-SLAM3 + Pangolin + OpenCV GUI
INSTALL_DEBIANS="libglew2.2,libgtk-3-0,libgl1,libegl1,libx11-6,libxext6"
INSTALL_DEBIANS="${INSTALL_DEBIANS},libboost-filesystem1.74.0,libboost-serialization1.74.0"
INSTALL_DEBIANS="${INSTALL_DEBIANS},libssl3"

# ─── Parse arguments ────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)         DEPLOY_IMAGE_NAME="$2"; shift 2 ;;
        --base-key)     BASE_IMAGE_KEY="$2"; shift 2 ;;
        --build-libs)   BUILD_LIBS=1; shift ;;
        --force)        FORCE=1; shift ;;
        --help|-h)
            echo "Usage: $0 [--name IMAGE_NAME] [--base-key KEY] [--build-libs] [--force]"
            echo "  --name         Deploy image name (default: isaac_ros_visual_slam_orb_deploy)"
            echo "  --base-key     Base image key (default: aarch64.ros2_humble.realsense)"
            echo "  --build-libs   Run extract_orb_slam3_libs.sh with --build before building"
            echo "  --force        Force rebuild tarball and image"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

print_info "============================================================"
print_info "  Building deploy image: ${DEPLOY_IMAGE_NAME}"
print_info "============================================================"

# ─── Check prerequisites ────────────────────────────────────────────────────
# 1. Check that the ROS workspace has been built
REQUIRED_PKGS=("isaac_ros_visual_slam_orb" "isaac_ros_visual_slam_interfaces" "isaac_common")
MISSING_PKGS=()
for pkg in "${REQUIRED_PKGS[@]}"; do
    if [[ ! -d "${ISAAC_ROS_WS}/install/${pkg}" ]]; then
        MISSING_PKGS+=("${pkg}")
    fi
done

if [[ ${#MISSING_PKGS[@]} -gt 0 ]]; then
    print_error "Missing built packages in ${ISAAC_ROS_WS}/install/:"
    for pkg in "${MISSING_PKGS[@]}"; do
        print_error "  - ${pkg}"
    done
    print_error ""
    print_error "Build them first inside the dev container:"
    print_error "  ./src/isaac_ros_common/scripts/run_dev.sh"
    print_error "  # (inside container)"
    print_error "  bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh"
    print_error "  colcon build --packages-up-to isaac_ros_visual_slam_orb"
    exit 1
fi
print_info "All required ROS packages found in install/."

# 2. Check/build ORB-SLAM3 libs tarball
if [[ ! -f "${TARBALL_PATH}" || ${FORCE} -eq 1 ]]; then
    if [[ ${BUILD_LIBS} -eq 1 ]]; then
        print_info "Extracting ORB-SLAM3 libraries (--build-libs)..."
        EXTRACT_ARGS="--build"
        if [[ ${FORCE} -eq 1 ]]; then
            EXTRACT_ARGS="${EXTRACT_ARGS} --force"
        fi
        bash "${SCRIPT_DIR}/extract_orb_slam3_libs.sh" ${EXTRACT_ARGS}
    elif [[ ! -f "${TARBALL_PATH}" ]]; then
        print_error "ORB-SLAM3 libs tarball not found: ${TARBALL_PATH}"
        print_error ""
        print_error "Either:"
        print_error "  1) Run: bash ${SCRIPT_DIR}/extract_orb_slam3_libs.sh --build"
        print_error "  2) Or pass --build-libs to this script"
        exit 1
    fi
else
    print_info "Using existing tarball: ${TARBALL_PATH}"
fi

# ─── Prepare custom entrypoint ──────────────────────────────────────────────
# Stage the entrypoint so docker_deploy.sh can include it via -d
ENTRYPOINT_SRC="${SCRIPT_DIR}/deploy_entrypoint.sh"
if [[ ! -f "${ENTRYPOINT_SRC}" ]]; then
    print_error "deploy_entrypoint.sh not found at ${ENTRYPOINT_SRC}"
    exit 1
fi
chmod +x "${ENTRYPOINT_SRC}"

# ─── Build deploy image ─────────────────────────────────────────────────────
print_info "Calling docker_deploy.sh..."
print_info "  Base image key : ${BASE_IMAGE_KEY}"
print_info "  Image name     : ${DEPLOY_IMAGE_NAME}"
print_info "  Launch package : ${LAUNCH_PACKAGE}"
print_info "  Launch file    : ${LAUNCH_FILE}"
print_info "  Debians        : ${INSTALL_DEBIANS}"
print_info "  Tarball        : ${TARBALL_PATH}"
print_info "  Entrypoint     : ${ENTRYPOINT_SRC}"

"${COMMON_SCRIPTS}/docker_deploy.sh" \
    -b "${BASE_IMAGE_KEY}" \
    -n "${DEPLOY_IMAGE_NAME}" \
    -w "${ISAAC_ROS_WS}" \
    -t "${TARBALL_PATH}" \
    -d "${ENTRYPOINT_SRC}:/usr/local/bin/scripts/deploy-entrypoint.sh" \
    -i "${INSTALL_DEBIANS}" \
    -p "${LAUNCH_PACKAGE}" \
    -f "${LAUNCH_FILE}"

print_info "============================================================"
print_info "  Deploy image built: ${DEPLOY_IMAGE_NAME}"
print_info "============================================================"
print_info ""
print_info "Run with:"
print_info "  bash ${SCRIPT_DIR}/run_deploy_orb.sh"
print_info "  bash ${SCRIPT_DIR}/run_deploy_orb.sh --mode stereo-imu"
print_info "  bash ${SCRIPT_DIR}/run_deploy_orb.sh -s   # interactive shell"
