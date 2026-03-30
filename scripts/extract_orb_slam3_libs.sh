#!/bin/bash
# =============================================================================
# extract_orb_slam3_libs.sh
#
# Extracts pre-built ORB-SLAM3 and Pangolin libraries from the dev container
# into a tarball on the host filesystem.
#
# The tarball can then be passed to docker_deploy.sh via the -t flag to include
# these libraries in the deploy image.
#
# Prerequisites:
#   - Dev image 'isaac_ros_dev-aarch64:latest' must exist
#   - install_orb_slam3.sh must have been run inside the dev container at least
#     once, OR use --build to run it automatically
#
# Usage:
#   # Extract from a running dev container (libs already built)
#   bash extract_orb_slam3_libs.sh
#
#   # Build ORB-SLAM3 inside a fresh container, then extract
#   bash extract_orb_slam3_libs.sh --build
#
#   # Force re-extraction even if tarball already exists
#   bash extract_orb_slam3_libs.sh --force
#
# =============================================================================
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-/mnt/nova_ssd/workspaces/isaac_ros-dev}"
COMMON_SCRIPTS="${ISAAC_ROS_WS}/src/isaac_ros_common/scripts"
source "${COMMON_SCRIPTS}/utils/print_color.sh"

# ─── Defaults ────────────────────────────────────────────────────────────────
DEV_IMAGE="isaac_ros_dev-aarch64:latest"
CONTAINER_NAME="isaac_ros_orb_extract"
TARBALL_NAME="orb_slam3_deploy_libs.tar.gz"
TARBALL_PATH="${ISAAC_ROS_WS}/${TARBALL_NAME}"
DO_BUILD=0
FORCE=0

# ─── Parse arguments ────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)      DO_BUILD=1; shift ;;
        --force)      FORCE=1; shift ;;
        --image)      DEV_IMAGE="$2"; shift 2 ;;
        --output)     TARBALL_PATH="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [--build] [--force] [--image IMAGE] [--output PATH]"
            echo "  --build   Run install_orb_slam3.sh inside the container before extracting"
            echo "  --force   Overwrite existing tarball"
            echo "  --image   Dev image to use (default: isaac_ros_dev-aarch64:latest)"
            echo "  --output  Output tarball path (default: \${ISAAC_ROS_WS}/orb_slam3_deploy_libs.tar.gz)"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ─── Pre-checks ─────────────────────────────────────────────────────────────
if [[ ${FORCE} -eq 0 && -f "${TARBALL_PATH}" ]]; then
    print_info "Tarball already exists: ${TARBALL_PATH}"
    print_info "Use --force to overwrite."
    exit 0
fi

if [[ -z "$(docker image ls --quiet ${DEV_IMAGE})" ]]; then
    print_error "Dev image '${DEV_IMAGE}' not found. Run run_dev.sh first to build it."
    exit 1
fi

# ─── Cleanup function ───────────────────────────────────────────────────────
function cleanup {
    print_info "Cleaning up container: ${CONTAINER_NAME}"
    docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
}
trap cleanup EXIT

# ─── Remove any previous container with same name ───────────────────────────
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

# ─── Start the dev container ────────────────────────────────────────────────
print_info "Starting dev container: ${CONTAINER_NAME}"
print_info "  Image: ${DEV_IMAGE}"
print_info "  Workspace mount: ${ISAAC_ROS_WS} -> /workspaces/isaac_ros-dev"

PLATFORM="$(uname -m)"
DOCKER_ARGS=()
DOCKER_ARGS+=("-v ${ISAAC_ROS_WS}:/workspaces/isaac_ros-dev")
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro")
DOCKER_ARGS+=("--network host")
DOCKER_ARGS+=("--ipc=host")
DOCKER_ARGS+=("--runtime nvidia")

if [[ "${PLATFORM}" == "aarch64" ]]; then
    DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=nvidia.com/gpu=all,nvidia.com/pva=all")
else
    DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
fi
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

docker run -d --privileged \
    ${DOCKER_ARGS[@]} \
    --name "${CONTAINER_NAME}" \
    -w /workspaces/isaac_ros-dev \
    "${DEV_IMAGE}" \
    sleep infinity

print_info "Container started."

# ─── Optionally build ORB-SLAM3 inside the container ────────────────────────
if [[ ${DO_BUILD} -eq 1 ]]; then
    print_info "Building ORB-SLAM3 inside the container..."
    docker exec "${CONTAINER_NAME}" \
        bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh
    print_info "ORB-SLAM3 build complete."
else
    # Verify that ORB-SLAM3 is already installed
    if ! docker exec "${CONTAINER_NAME}" test -f /opt/orb_slam3/lib/libORB_SLAM3.so; then
        print_error "ORB-SLAM3 not found in the container at /opt/orb_slam3/lib/libORB_SLAM3.so"
        print_error "Either:"
        print_error "  1) Run install_orb_slam3.sh inside the dev container first, OR"
        print_error "  2) Use --build flag to build automatically"
        exit 1
    fi
    print_info "ORB-SLAM3 libraries found in container."
fi

# ─── Create tarball inside the container ─────────────────────────────────────
print_info "Creating tarball of ORB-SLAM3 + Pangolin libraries..."

docker exec "${CONTAINER_NAME}" bash -c '
set -e

TARBALL="/tmp/orb_slam3_deploy_libs.tar.gz"

# Collect all paths to include
PATHS_TO_TAR=""

# ORB-SLAM3 libraries, vocabulary, and headers
if [ -d /opt/orb_slam3 ]; then
    PATHS_TO_TAR="${PATHS_TO_TAR} opt/orb_slam3"
fi

# Pangolin shared libraries
PANGO_LIBS=$(find /usr/local/lib -maxdepth 1 -name "libpango*" -type f -o -name "libpango*" -type l 2>/dev/null | sed "s|^/||" || true)
PATHS_TO_TAR="${PATHS_TO_TAR} ${PANGO_LIBS}"

# Pangolin cmake config
if [ -d /usr/local/lib/cmake/Pangolin ]; then
    PATHS_TO_TAR="${PATHS_TO_TAR} usr/local/lib/cmake/Pangolin"
fi

# Create the tarball from root
cd /
tar czf "${TARBALL}" ${PATHS_TO_TAR}

echo "Tarball created: ${TARBALL}"
ls -lh "${TARBALL}"
'

# ─── Copy tarball to host ───────────────────────────────────────────────────
print_info "Copying tarball to host: ${TARBALL_PATH}"
docker cp "${CONTAINER_NAME}:/tmp/orb_slam3_deploy_libs.tar.gz" "${TARBALL_PATH}"

# Show tarball info
TARBALL_SIZE=$(ls -lh "${TARBALL_PATH}" | awk '{print $5}')
print_info "Tarball extracted successfully: ${TARBALL_PATH} (${TARBALL_SIZE})"

# List contents summary
print_info "Tarball contents:"
tar tzf "${TARBALL_PATH}" | head -30
echo "  ... (use 'tar tzf ${TARBALL_PATH}' to see full listing)"

print_info "DONE. Tarball ready for deploy image build."
