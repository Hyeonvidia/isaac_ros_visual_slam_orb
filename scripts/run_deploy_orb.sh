#!/bin/bash
# =============================================================================
# run_deploy_orb.sh
#
# Runs the isaac_ros_visual_slam_orb deploy container with X11/OpenGL display
# forwarding for Pangolin viewer and OpenCV windows, RealSense D456 USB access,
# and Jetson GPU support.
#
# Usage:
#   ./run_deploy_orb.sh                          # default: stereo mode
#   ./run_deploy_orb.sh --mode stereo-imu        # stereo + IMU
#   ./run_deploy_orb.sh --mode rgbd              # RGB-D mode
#   ./run_deploy_orb.sh -s                       # interactive shell
#   ./run_deploy_orb.sh --mode stereo key:=value # extra launch args
#
# =============================================================================
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-/mnt/nova_ssd/workspaces/isaac_ros-dev}"
COMMON_SCRIPTS="${ISAAC_ROS_WS}/src/isaac_ros_common/scripts"
source "${COMMON_SCRIPTS}/utils/print_color.sh"

# ─── Defaults ────────────────────────────────────────────────────────────────
DEPLOY_IMAGE_NAME="isaac_ros_visual_slam_orb_deploy"
LAUNCH_PACKAGE="isaac_ros_visual_slam_orb"
LAUNCH_FILE="isaac_ros_visual_slam_orb_realsense_d456.launch.py"
MODE="stereo"
INTERACTIVE_SHELL=0

# ─── Parse arguments ────────────────────────────────────────────────────────
LAUNCH_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--shell)
            INTERACTIVE_SHELL=1; shift ;;
        -m|--mode)
            MODE="$2"; shift 2 ;;
        --image)
            DEPLOY_IMAGE_NAME="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [-s|--shell] [-m|--mode MODE] [--image IMAGE] [launch_args...]"
            echo ""
            echo "Options:"
            echo "  -s, --shell     Open interactive bash (launch file will NOT auto-start)"
            echo "  -m, --mode      Sensor mode: mono, mono-inertial, stereo, stereo-inertial, rgb-d"
            echo "                  Aliases: mono-imu, stereo-imu, rgbd, rgbd-imu (default: stereo)"
            echo "  --image         Deploy image name (default: isaac_ros_visual_slam_orb_deploy)"
            echo ""
            echo "Extra arguments (key:=value) are forwarded to the launch file."
            exit 0 ;;
        --)
            shift; LAUNCH_ARGS+=("$@"); break ;;
        *)
            LAUNCH_ARGS+=("$1"); shift ;;
    esac
done

# ─── Check docker group ─────────────────────────────────────────────────────
RE="\<docker\>"
if [[ ! $(groups ${USER}) =~ $RE ]]; then
    print_error "User |${USER}| is not in the 'docker' group."
    print_error "Run: sudo usermod -aG docker \${USER} && newgrp docker"
    exit 1
fi

# ─── Check image exists ─────────────────────────────────────────────────────
if [[ -z "$(docker image ls --quiet ${DEPLOY_IMAGE_NAME})" ]]; then
    print_error "Deploy image '${DEPLOY_IMAGE_NAME}' not found."
    print_error "Run: bash ${SCRIPT_DIR}/build_deploy.sh"
    exit 1
fi

PLATFORM="$(uname -m)"
CONTAINER_NAME="${DEPLOY_IMAGE_NAME}-container"

# Remove previously exited container with the same name
if [ "$(docker ps -a --quiet --filter status=exited --filter name=${CONTAINER_NAME})" ]; then
    docker rm "${CONTAINER_NAME}" > /dev/null
fi

# ─── Allow X11 connections from Docker ───────────────────────────────────────
xhost +local:root > /dev/null 2>&1 || true

# ─── Docker arguments ───────────────────────────────────────────────────────
DOCKER_ARGS=()

# Display / X11 (required for Pangolin viewer and OpenCV highgui)
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v ${HOME}/.Xauthority:/root/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY=${DISPLAY:-:0}")

# NVIDIA GPU
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

# ORB-SLAM3 / Pangolin library paths
DOCKER_ARGS+=("-e LD_LIBRARY_PATH=/opt/orb_slam3/lib:/usr/local/lib:\${LD_LIBRARY_PATH}")

# ROS workspace
DOCKER_ARGS+=("-e ISAAC_ROS_WS=/workspaces/isaac_ros-dev")
DOCKER_ARGS+=("-e ROS_WS=/workspaces/isaac_ros-dev")

# ROS domain
if [[ ! -z "${ROS_DOMAIN_ID}" ]]; then
    DOCKER_ARGS+=("-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID}")
fi

# Timezone
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro")

# RealSense USB device access
DOCKER_ARGS+=("-v /dev:/dev")

# Jetson / aarch64-specific
if [[ "${PLATFORM}" == "aarch64" ]]; then
    DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=nvidia.com/gpu=all,nvidia.com/pva=all")
    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/:/tmp/")
    DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("-v /usr/share/vpi3:/usr/share/vpi3")
    DOCKER_ARGS+=("-v /dev/input:/dev/input")
    DOCKER_ARGS+=("--pid=host")
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
    fi
else
    DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
fi

# ─── Jetson power optimization ──────────────────────────────────────────────
if [[ "${PLATFORM}" == "aarch64" ]] && command -v nvpmodel &>/dev/null; then
    CURRENT_MODE=$(nvpmodel -q 2>/dev/null | grep -oP 'NV Power Mode: \K\w+' || true)
    if [[ "${CURRENT_MODE}" != "MAXN" ]]; then
        print_warning "Setting Jetson to MAXN power mode for best performance"
        sudo nvpmodel -m 0 2>/dev/null || true
    fi
    sudo jetson_clocks 2>/dev/null || true
fi

# ─── Ctrl+C handler: stop and remove container ────────────────────────────
function cleanup_container {
    echo ""
    print_warning "Stopping container: ${CONTAINER_NAME}"
    docker stop "${CONTAINER_NAME}" 2>/dev/null || true
    docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
    print_info "Container stopped."
    exit 0
}
trap cleanup_container SIGINT SIGTERM

# ─── Run container ──────────────────────────────────────────────────────────
print_info "============================================================"
print_info "  ORB-SLAM3 Deploy Container"
print_info "============================================================"
print_info "  Image     : ${DEPLOY_IMAGE_NAME}"
print_info "  Container : ${CONTAINER_NAME}"
print_info "  Mode      : ${MODE}"
print_info "  Ctrl+C to stop and remove container"

if [[ ${INTERACTIVE_SHELL} -eq 1 ]]; then
    print_info "  Shell     : interactive bash"
    print_info "============================================================"
    docker run -it --rm \
        --privileged \
        --network host \
        --ipc=host \
        --runtime nvidia \
        ${DOCKER_ARGS[@]} \
        --name "${CONTAINER_NAME}" \
        "${DEPLOY_IMAGE_NAME}" \
        /bin/bash
else
    FULL_LAUNCH_CMD="ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} mode:=${MODE} ${LAUNCH_ARGS[*]}"
    print_info "  Launch    : ${FULL_LAUNCH_CMD}"
    print_info "============================================================"
    # Run without -t so the trap/wait pattern works for Ctrl+C
    docker run -i --rm \
        --privileged \
        --network host \
        --ipc=host \
        --runtime nvidia \
        ${DOCKER_ARGS[@]} \
        --name "${CONTAINER_NAME}" \
        "${DEPLOY_IMAGE_NAME}" \
        bash -c "${FULL_LAUNCH_CMD}" &
    wait $!
fi
