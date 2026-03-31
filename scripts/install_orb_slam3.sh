#!/usr/bin/env bash
# =============================================================================
# install_orb_slam3.sh
#
# Builds and installs ORB-SLAM3 (with Pangolin viewer + examples) inside the
# isaac_ros Docker container (ros2_humble.realsense, JetPack 6 / CUDA 12.x).
#
# Key design decisions:
#   - Reuses the container's system OpenCV (4.5.4) if compatible (>= 4.2).
#     Only builds OpenCV with CUDA when --with-cuda-opencv is passed.
#   - Detects the container's GCC version and uses C++17 throughout.
#   - Builds Pangolin with X11/OpenGL display support for GUI visualization.
#   - Builds ORB-SLAM3 example executables (mono, stereo, rgbd) for testing.
#   - Installs to a workspace-mounted path so results persist on the host.
#
# Usage (run INSIDE the container):
#   # Default: use system OpenCV, build to workspace-mounted path
#   bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh
#
#   # With CUDA-enabled OpenCV (longer build, ~40min on Orin)
#   bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh --with-cuda-opencv
#
#   # Custom install prefix
#   bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh --prefix /opt/orb_slam3
#
# =============================================================================
set -euo pipefail

# ─── Defaults ────────────────────────────────────────────────────────────────
INSTALL_PREFIX="/opt/orb_slam3"
# Build dir on mounted workspace — survives container restarts
BUILD_DIR="/workspaces/isaac_ros-dev/.orb_slam3_build"
BUILD_CUDA_OPENCV=0
FORCE_REBUILD=0
NPROC=$(nproc)

# ─── Parse arguments ────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-cuda-opencv) BUILD_CUDA_OPENCV=1; shift ;;
    --prefix)           INSTALL_PREFIX="$2"; shift 2 ;;
    --build-dir)        BUILD_DIR="$2"; shift 2 ;;
    --force)            FORCE_REBUILD=1; shift ;;
    --help|-h)
      echo "Usage: $0 [--with-cuda-opencv] [--prefix /path] [--build-dir /path] [--force]"
      echo "  --force  Force rebuild even if libraries already exist"
      exit 0 ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

# ─── Colors / logging ───────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
step()  { echo -e "\n${CYAN}══════════════════════════════════════════════${NC}"; \
          echo -e "${CYAN}  $*${NC}"; \
          echo -e "${CYAN}══════════════════════════════════════════════${NC}"; }

# ─── Detect environment ─────────────────────────────────────────────────────
GCC_VERSION=$(gcc -dumpversion 2>/dev/null || echo "unknown")
CXX_STANDARD="17"  # Container's GCC 11 fully supports C++17
ARCH=$(uname -m)

step "Environment Detection"
info "Install prefix   : ${INSTALL_PREFIX}"
info "Build directory  : ${BUILD_DIR}"
info "GCC version      : ${GCC_VERSION}"
info "C++ standard     : C++${CXX_STANDARD}"
info "Architecture     : ${ARCH}"
info "Parallel jobs    : ${NPROC}"
info "CUDA OpenCV build: $([ ${BUILD_CUDA_OPENCV} -eq 1 ] && echo 'YES' || echo 'NO (using system OpenCV)')"
info "Force rebuild    : $([ ${FORCE_REBUILD} -eq 1 ] && echo 'YES' || echo 'NO (skip if already built)')"

# Use sudo for system directories if not running as root
if [[ $(id -u) -ne 0 ]]; then
  SUDO="sudo"
else
  SUDO=""
fi

${SUDO} mkdir -p "${BUILD_DIR}" "${INSTALL_PREFIX}"/{lib,include,bin,Vocabulary,examples}
# Ensure current user can write to build/install dirs
${SUDO} chown -R "$(id -u):$(id -g)" "${BUILD_DIR}" "${INSTALL_PREFIX}"

# =============================================================================
# 0. System packages (apt) — only install what's missing
# =============================================================================
step "Step 0: System Dependencies"
info "Installing apt dependencies..."
${SUDO} apt-get update -qq 2>/dev/null || warn "apt-get update had warnings (non-fatal)"
${SUDO} apt-get install -y --no-install-recommends \
  git cmake ninja-build pkg-config \
  libglew-dev libgtk-3-dev \
  libeigen3-dev \
  libboost-filesystem-dev libboost-thread-dev libboost-serialization-dev \
  libssl-dev \
  ca-certificates \
  libx11-dev libxext-dev \
  2>/dev/null

# Verify critical dependencies
info "Checking critical dependencies..."
command -v cmake >/dev/null  || error "cmake not found"
command -v ninja >/dev/null  || error "ninja not found"
command -v g++   >/dev/null  || error "g++ not found"
pkg-config --exists eigen3   || error "eigen3 not found"
info "All critical dependencies OK."

# Set up ccache if available (speeds up recompilation significantly)
if command -v ccache &>/dev/null; then
  export CC="ccache gcc"
  export CXX="ccache g++"
  info "ccache enabled (CC=${CC}, CXX=${CXX})"
else
  # Try to install ccache — small package, big speedup on rebuilds
  ${SUDO} apt-get install -y --no-install-recommends ccache 2>/dev/null && {
    export CC="ccache gcc"
    export CXX="ccache g++"
    info "ccache installed and enabled."
  } || info "ccache not available (builds will work, just slower on rebuild)."
fi

# =============================================================================
# 1. OpenCV — reuse system or build with CUDA
# =============================================================================
step "Step 1: OpenCV"

# Detect system OpenCV
SYSTEM_OPENCV_VER=$(pkg-config --modversion opencv4 2>/dev/null || echo "none")
info "System OpenCV version: ${SYSTEM_OPENCV_VER}"

if [[ "${BUILD_CUDA_OPENCV}" -eq 1 ]]; then
  # ── User explicitly requested CUDA OpenCV ──────────────────────────────
  OPENCV_CUDA_CHECK=$(python3 -c \
    "import cv2; print(cv2.cuda.getCudaEnabledDeviceCount())" 2>/dev/null || echo "0")

  if [[ "${OPENCV_CUDA_CHECK}" != "0" ]]; then
    info "OpenCV CUDA already available (${OPENCV_CUDA_CHECK} device(s)). Skipping rebuild."
  else
    info "Building OpenCV 4.10 with CUDA (this takes ~40 min on Orin)..."

    # Install cuDNN dev for CUDA OpenCV build
    ${SUDO} apt-get install -y --no-install-recommends libcudnn9-dev-cuda-12 2>/dev/null || true

    OPENCV_VERSION="4.10.0"
    cd "${BUILD_DIR}"

    [[ ! -d "opencv" ]] && \
      git clone --depth 1 --branch "${OPENCV_VERSION}" https://github.com/opencv/opencv.git
    [[ ! -d "opencv_contrib" ]] && \
      git clone --depth 1 --branch "${OPENCV_VERSION}" https://github.com/opencv/opencv_contrib.git

    # Symlink cuDNN headers if needed
    if [[ -f "/usr/include/${ARCH}-linux-gnu/cudnn.h" ]] && [[ ! -f "/usr/include/cudnn.h" ]]; then
      ${SUDO} ln -sf /usr/include/${ARCH}-linux-gnu/cudnn*.h /usr/include/ 2>/dev/null || true
      info "Symlinked cuDNN headers to /usr/include/"
    fi

    # Hide broken cuDSS cmake config
    CUDSS_MOVED=0
    if [[ -d "/usr/local/libcudss-linux-0.3.0.9_cuda12-archive" ]]; then
      ${SUDO} mv /usr/local/libcudss-linux-0.3.0.9_cuda12-archive \
         /usr/local/_libcudss_backup 2>/dev/null || true
      CUDSS_MOVED=1
    fi

    # Detect CUDA compute capability
    CUDA_ARCH="8.7"  # Jetson AGX Orin default
    if command -v nvidia-smi &>/dev/null; then
      DETECTED_ARCH=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 || echo "")
      [[ -n "${DETECTED_ARCH}" ]] && CUDA_ARCH="${DETECTED_ARCH}"
    fi
    info "CUDA compute capability: ${CUDA_ARCH}"

    mkdir -p opencv/build && cd opencv/build
    cmake .. \
      -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_CXX_STANDARD=${CXX_STANDARD} \
      -DOPENCV_EXTRA_MODULES_PATH="${BUILD_DIR}/opencv_contrib/modules" \
      -DWITH_CUDA=ON \
      -DWITH_CUDNN=OFF \
      -DOPENCV_DNN_CUDA=OFF \
      -DBUILD_opencv_dnn=OFF \
      -DCUDA_ARCH_BIN="${CUDA_ARCH}" \
      -DENABLE_FAST_MATH=ON \
      -DCUDA_FAST_MATH=ON \
      -DWITH_CUBLAS=ON \
      -DBUILD_opencv_python3=ON \
      -DBUILD_TESTS=OFF \
      -DBUILD_PERF_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_DOCS=OFF \
      2>&1 | tail -15

    # Cap parallel jobs to avoid OOM on Jetson
    BUILD_JOBS=$(( NPROC > 6 ? 6 : NPROC ))
    info "Building with ${BUILD_JOBS} parallel jobs..."
    ninja -j"${BUILD_JOBS}"
    ${SUDO} ninja install
    ${SUDO} ldconfig

    # Restore cuDSS
    if [[ "${CUDSS_MOVED}" == "1" ]] && [[ -d "/usr/local/_libcudss_backup" ]]; then
      ${SUDO} mv /usr/local/_libcudss_backup \
         /usr/local/libcudss-linux-0.3.0.9_cuda12-archive 2>/dev/null || true
    fi

    info "OpenCV ${OPENCV_VERSION} with CUDA installed to /usr/local"
    OPENCV_CMAKE_DIR="/usr/local/lib/cmake/opencv4"
  fi
else
  # ── Reuse system OpenCV (no CUDA modules, but sufficient for ORB-SLAM3) ─
  if [[ "${SYSTEM_OPENCV_VER}" == "none" ]]; then
    error "No system OpenCV found. Install libopencv-dev or use --with-cuda-opencv"
  fi

  # Check minimum version (ORB-SLAM3 requires >= 4.2)
  OPENCV_MAJOR=$(echo "${SYSTEM_OPENCV_VER}" | cut -d. -f1)
  OPENCV_MINOR=$(echo "${SYSTEM_OPENCV_VER}" | cut -d. -f2)
  if [[ "${OPENCV_MAJOR}" -lt 4 ]] || { [[ "${OPENCV_MAJOR}" -eq 4 ]] && [[ "${OPENCV_MINOR}" -lt 2 ]]; }; then
    error "System OpenCV ${SYSTEM_OPENCV_VER} too old (need >= 4.2). Use --with-cuda-opencv"
  fi

  info "Reusing system OpenCV ${SYSTEM_OPENCV_VER} (sufficient for ORB-SLAM3)"
  info "  -> CUDA modules not available; ORB feature extraction will run on CPU"
  info "  -> To enable GPU acceleration, rerun with --with-cuda-opencv"

  # Find OpenCV cmake directory
  OPENCV_CMAKE_DIR=$(pkg-config --variable=libdir opencv4 2>/dev/null)/cmake/opencv4
  if [[ ! -d "${OPENCV_CMAKE_DIR}" ]]; then
    # Fallback search
    OPENCV_CMAKE_DIR=$(find /usr -name "OpenCVConfig.cmake" -exec dirname {} \; 2>/dev/null | head -1 || echo "")
  fi
  info "OpenCV cmake dir: ${OPENCV_CMAKE_DIR}"
fi

# =============================================================================
# 2. Pangolin (viewer with X11/OpenGL display support)
# =============================================================================
step "Step 2: Pangolin (with display support)"

REBUILD_PANGOLIN=0
if pkg-config --exists pangolin 2>/dev/null; then
  info "Pangolin already installed. Skipping."
else
  REBUILD_PANGOLIN=1
fi

if [[ "${REBUILD_PANGOLIN}" -eq 1 ]]; then
  info "Building Pangolin with X11/OpenGL display support..."
  cd "${BUILD_DIR}"
  if [[ ! -d "Pangolin" ]]; then
    git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git
  fi

  # Patch out -Werror flags (OpenEXR headers trigger warnings on GCC 11)
  sed -i 's/-Wall -Wextra -Werror/-Wall -Wextra/' Pangolin/CMakeLists.txt
  sed -i 's/-Werror=maybe-uninitialized/-Wmaybe-uninitialized/' Pangolin/CMakeLists.txt
  sed -i 's/-Werror=vla/-Wvla/' Pangolin/CMakeLists.txt

  # Patch CUDA 12 compatibility: memoryType → type (deprecated in CUDA 11, removed in 12)
  # Try both possible source paths (varies by Pangolin version)
  for MEMCPY_H in Pangolin/include/pangolin/image/memcpy.h \
                   Pangolin/components/pango_image/include/pangolin/image/memcpy.h; do
    if [[ -f "${MEMCPY_H}" ]]; then
      sed -i 's/attributes\.memoryType/attributes.type/g' "${MEMCPY_H}"
      info "Patched ${MEMCPY_H}"
    fi
  done

  mkdir -p Pangolin/build && cd Pangolin/build
  cmake .. \
    -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_CXX_STANDARD=${CXX_STANDARD} \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=ON \
    2>&1 | tail -5
  ninja -j"${NPROC}"
  ${SUDO} ninja install
  ${SUDO} ldconfig

  # Patch installed header as well (for colcon build later)
  if [[ -f "/usr/local/include/pangolin/image/memcpy.h" ]]; then
    ${SUDO} sed -i 's/attributes\.memoryType/attributes.type/g' \
      /usr/local/include/pangolin/image/memcpy.h
    info "Patched installed Pangolin header for CUDA 12."
  fi
  info "Pangolin installed with display support."

  # Quick verification: check that Pangolin found OpenGL
  if [[ -f "/usr/local/lib/libpangolin.so" ]] || [[ -f "/usr/local/lib/libpango_display.so" ]]; then
    info "Pangolin libraries OK."
  else
    warn "Pangolin library files not found in expected location, checking alternatives..."
    find /usr/local/lib -name "libpango*" -o -name "libpangolin*" 2>/dev/null | head -5
  fi
fi

# =============================================================================
# 3. ORB-SLAM3 — library + example executables
# =============================================================================
step "Step 3: ORB-SLAM3 (library + examples)"

info "Cloning ORB-SLAM3..."
cd "${BUILD_DIR}"

ORB_SLAM3_REPO="https://github.com/UZ-SLAMLab/ORB_SLAM3.git"
ORB_SLAM3_BRANCH="master"

if [[ ! -d "ORB_SLAM3" ]]; then
  git clone --depth 1 --branch "${ORB_SLAM3_BRANCH}" "${ORB_SLAM3_REPO}"
fi
cd ORB_SLAM3

# ── Patch for C++17 / GCC ${GCC_VERSION} / aarch64 ──────────────────────────
info "Patching ORB-SLAM3 for C++${CXX_STANDARD} / GCC ${GCC_VERSION} / ${ARCH}..."

# Use container's C++ standard (C++17) instead of hardcoded C++11
sed -i "s/-std=c++11/-std=c++${CXX_STANDARD}/g" CMakeLists.txt
sed -i "s/-std=c++0x/-std=c++${CXX_STANDARD}/g" CMakeLists.txt

# Also patch Thirdparty CMakeLists for consistent C++ standard
for TPCML in Thirdparty/DBoW2/CMakeLists.txt Thirdparty/g2o/CMakeLists.txt; do
  if [[ -f "${TPCML}" ]]; then
    sed -i "s/-std=c++11/-std=c++${CXX_STANDARD}/g" "${TPCML}"
    sed -i "s/-std=c++0x/-std=c++${CXX_STANDARD}/g" "${TPCML}"
    sed -i "s/-std=c++14/-std=c++${CXX_STANDARD}/g" "${TPCML}"
  fi
done

# Suppress deprecated Boost.Bind placeholders warning
if ! grep -q "BOOST_BIND_GLOBAL_PLACEHOLDERS" CMakeLists.txt; then
  sed -i '/find_package(Boost/a add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)' CMakeLists.txt
fi

# Remove hardcoded -march=native (breaks cross-compilation / aarch64 detection)
sed -i 's/-march=native//' CMakeLists.txt

# Fix: mnFullBAIdx declared as bool but used with ++ (forbidden in C++17)
sed -i 's/bool mnFullBAIdx;/int mnFullBAIdx;/' include/LoopClosing.h

# Fix: Rectified stereo leaves calibration2_ null → SIGSEGV when printing settings.
# Patch readCamera2 to create calibration2_ as copy of calibration1_ for Rectified type.
# Guard uses a unique string from the patch to avoid false-positive from PinHole block.
if ! grep -q "calibration2_ = new Pinhole(vCal1)" src/Settings.cc 2>/dev/null; then
  sed -i '/if(cameraType_ == Rectified){/{
    N
    s|if(cameraType_ == Rectified){\n            b_ = |if(cameraType_ == Rectified){\n            if(!calibration2_ \&\& calibration1_){\n                vector<float> vCal1 = {calibration1_->getParameter(0),calibration1_->getParameter(1),calibration1_->getParameter(2),calibration1_->getParameter(3)};\n                calibration2_ = new Pinhole(vCal1);\n                originalCalib2_ = new Pinhole(vCal1);\n            }\n            b_ = |
  }' src/Settings.cc
  info "Patched Settings.cc: Rectified stereo calibration2_ fix."
else
  info "Settings.cc Rectified fix already applied."
fi

# ── Enable Viewer in CMakeLists ──────────────────────────────────────────────
# Ensure viewer is built (for Pangolin visualization testing)
# ORB-SLAM3's CMakeLists.txt should find Pangolin automatically

# ── Build Thirdparty/DBoW2 ──────────────────────────────────────────────────
if [[ -f "${INSTALL_PREFIX}/lib/libDBoW2.so" ]] && [[ "${FORCE_REBUILD}" -eq 0 ]]; then
  info "DBoW2 already built. Skipping. (use --force to rebuild)"
else
  info "Building DBoW2..."
  cd Thirdparty/DBoW2
  rm -rf build && mkdir -p build && cd build
  cmake .. -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=${CXX_STANDARD} \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    2>&1 | tail -3
  ninja -j"${NPROC}"
  cp ../lib/libDBoW2.so "${INSTALL_PREFIX}/lib/"
fi
cd "${BUILD_DIR}/ORB_SLAM3"

# ── Build Thirdparty/g2o ─────────────────────────────────────────────────────
if [[ -f "${INSTALL_PREFIX}/lib/libg2o.so" ]] && [[ "${FORCE_REBUILD}" -eq 0 ]]; then
  info "g2o already built. Skipping. (use --force to rebuild)"
else
  info "Building g2o..."
  cd Thirdparty/g2o
  rm -rf build && mkdir -p build && cd build
  cmake .. -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=${CXX_STANDARD} \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    2>&1 | tail -3
  ninja -j"${NPROC}"
  cp ../lib/libg2o.so "${INSTALL_PREFIX}/lib/"
fi
cd "${BUILD_DIR}/ORB_SLAM3"

# ── Build ORB-SLAM3 main library ─────────────────────────────────────────────
if [[ -f "${INSTALL_PREFIX}/lib/libORB_SLAM3.so" ]] && [[ "${FORCE_REBUILD}" -eq 0 ]]; then
  info "ORB-SLAM3 already built. Skipping. (use --force to rebuild)"
else
  info "Building ORB-SLAM3 main library..."
  rm -rf build && mkdir -p build && cd build

  CMAKE_OPENCV_ARGS=""
  if [[ -n "${OPENCV_CMAKE_DIR:-}" ]] && [[ -d "${OPENCV_CMAKE_DIR}" ]]; then
    CMAKE_OPENCV_ARGS="-DOpenCV_DIR=${OPENCV_CMAKE_DIR}"
  fi

  cmake .. \
    -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=${CXX_STANDARD} \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    ${CMAKE_OPENCV_ARGS} \
    -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    2>&1 | tail -10

  # Cap parallel jobs on Jetson to avoid OOM
  BUILD_JOBS=$(( NPROC > 6 ? 6 : NPROC ))
  info "Building with ${BUILD_JOBS} parallel jobs..."
  ninja -j"${BUILD_JOBS}"
fi

# ── Install library + headers ─────────────────────────────────────────────────
cd "${BUILD_DIR}/ORB_SLAM3"

info "Installing ORB-SLAM3 to ${INSTALL_PREFIX}..."
cp lib/libORB_SLAM3.so "${INSTALL_PREFIX}/lib/"

# Headers (preserving directory structure)
cp -r include/* "${INSTALL_PREFIX}/include/"

# Thirdparty headers
mkdir -p "${INSTALL_PREFIX}/include/Thirdparty"
cp -r Thirdparty/DBoW2 "${INSTALL_PREFIX}/include/Thirdparty/"
cp -r Thirdparty/g2o   "${INSTALL_PREFIX}/include/Thirdparty/"
cp -r Thirdparty/Sophus "${INSTALL_PREFIX}/include/Thirdparty/"
# Also top-level Sophus for direct includes
cp -r Thirdparty/Sophus "${INSTALL_PREFIX}/include/"

# ── Copy vocabulary file ──────────────────────────────────────────────────────
if [[ ! -f "${INSTALL_PREFIX}/Vocabulary/ORBvoc.txt" ]]; then
  if [[ -f "Vocabulary/ORBvoc.txt.tar.gz" ]]; then
    cd Vocabulary && tar -xzf ORBvoc.txt.tar.gz && cd ..
    cp Vocabulary/ORBvoc.txt "${INSTALL_PREFIX}/Vocabulary/"
    info "Vocabulary extracted."
  elif [[ -f "Vocabulary/ORBvoc.txt" ]]; then
    cp Vocabulary/ORBvoc.txt "${INSTALL_PREFIX}/Vocabulary/"
    info "Vocabulary copied."
  else
    warn "Vocabulary file not found. Download manually to ${INSTALL_PREFIX}/Vocabulary/ORBvoc.txt"
  fi
else
  info "Vocabulary already exists. Skipping."
fi

# ── Copy example executables ──────────────────────────────────────────────────
step "Step 4: Install Example Executables"
info "Copying ORB-SLAM3 example binaries..."

EXAMPLES_DIR="${INSTALL_PREFIX}/examples"
mkdir -p "${EXAMPLES_DIR}"

# Copy all built example binaries
EXAMPLE_BINS=(
  "Examples/Monocular/mono_euroc"
  "Examples/Monocular/mono_kitti"
  "Examples/Monocular/mono_tum"
  "Examples/Monocular/mono_tum_vi"
  "Examples/Monocular/mono_realsense_t265"
  "Examples/Monocular/mono_realsense_D435i"
  "Examples/Stereo/stereo_euroc"
  "Examples/Stereo/stereo_kitti"
  "Examples/Stereo/stereo_tum_vi"
  "Examples/Stereo/stereo_realsense_t265"
  "Examples/Stereo/stereo_realsense_D435i"
  "Examples/Monocular-Inertial/mono_inertial_euroc"
  "Examples/Monocular-Inertial/mono_inertial_tum_vi"
  "Examples/Monocular-Inertial/mono_inertial_realsense_t265"
  "Examples/Monocular-Inertial/mono_inertial_realsense_D435i"
  "Examples/Stereo-Inertial/stereo_inertial_euroc"
  "Examples/Stereo-Inertial/stereo_inertial_tum_vi"
  "Examples/Stereo-Inertial/stereo_inertial_realsense_t265"
  "Examples/Stereo-Inertial/stereo_inertial_realsense_D435i"
  "Examples/RGB-D/rgbd_tum"
  "Examples/RGB-D/rgbd_realsense_D435i"
  "Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i"
)

COPIED_COUNT=0
for BIN in "${EXAMPLE_BINS[@]}"; do
  if [[ -f "${BUILD_DIR}/ORB_SLAM3/${BIN}" ]]; then
    cp "${BUILD_DIR}/ORB_SLAM3/${BIN}" "${EXAMPLES_DIR}/"
    COPIED_COUNT=$((COPIED_COUNT + 1))
  fi
done
info "Copied ${COPIED_COUNT} example binaries to ${EXAMPLES_DIR}/"

# Copy example config/yaml files for RealSense cameras
mkdir -p "${EXAMPLES_DIR}/configs"
find "${BUILD_DIR}/ORB_SLAM3/Examples" -name "*.yaml" -exec cp {} "${EXAMPLES_DIR}/configs/" \; 2>/dev/null || true
YAML_COUNT=$(find "${BUILD_DIR}/ORB_SLAM3/Examples" -name "*.yaml" 2>/dev/null | wc -l)
info "Copied ${YAML_COUNT} example config files"

# ── Copy example EuRoC/TUM download helper ────────────────────────────────────
cat > "${EXAMPLES_DIR}/download_test_data.sh" << 'DLEOF'
#!/usr/bin/env bash
# Download a small EuRoC MAV dataset for testing ORB-SLAM3 examples
set -euo pipefail
DATA_DIR="${1:-/tmp/orb_slam3_test_data}"
mkdir -p "${DATA_DIR}"
echo "Downloading EuRoC MH_01_easy sequence..."
cd "${DATA_DIR}"
if [[ ! -d "MH_01_easy" ]]; then
  wget -q --show-progress http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
  unzip -q MH_01_easy.zip -d MH_01_easy
  rm -f MH_01_easy.zip
  echo "Downloaded to ${DATA_DIR}/MH_01_easy"
else
  echo "MH_01_easy already exists at ${DATA_DIR}/MH_01_easy"
fi
echo ""
echo "Run mono example:"
echo "  cd ${DATA_DIR}"
echo "  /opt/orb_slam3/examples/mono_euroc /opt/orb_slam3/Vocabulary/ORBvoc.txt /opt/orb_slam3/examples/configs/EuRoC.yaml ${DATA_DIR}/MH_01_easy/mav0/cam0/data /opt/orb_slam3/examples/configs/EuRoC_TimeStamps/MH01.txt"
echo ""
echo "Run stereo example:"
echo "  /opt/orb_slam3/examples/stereo_euroc /opt/orb_slam3/Vocabulary/ORBvoc.txt /opt/orb_slam3/examples/configs/EuRoC.yaml ${DATA_DIR}/MH_01_easy/mav0/cam0/data ${DATA_DIR}/MH_01_easy/mav0/cam1/data /opt/orb_slam3/examples/configs/EuRoC_TimeStamps/MH01.txt"
DLEOF
chmod +x "${EXAMPLES_DIR}/download_test_data.sh"

# ── Copy EuRoC timestamp files if they exist ──────────────────────────────────
if [[ -d "${BUILD_DIR}/ORB_SLAM3/Examples/Monocular/EuRoC_TimeStamps" ]]; then
  cp -r "${BUILD_DIR}/ORB_SLAM3/Examples/Monocular/EuRoC_TimeStamps" "${EXAMPLES_DIR}/configs/"
  info "EuRoC timestamp files copied."
fi

# =============================================================================
# 5. Create RealSense D456 config + convenience test scripts
# =============================================================================
step "Step 5: RealSense D456 Config & Test Scripts"

# ── Generate D456-specific ORB-SLAM3 settings YAML ──────────────────────────
# D456 specs:
#   - Stereo baseline: 95mm
#   - IMU sensor: BMI055
#   - Global shutter IR cameras
#   - Camera intrinsics are overridden at runtime from CameraInfo,
#     but IMU noise and baseline must be set correctly here.

cat > "${INSTALL_PREFIX}/examples/configs/RealSense_D456.yaml" << 'D456EOF'
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters (overridden by RealSense SDK at runtime for exact calibration)
#--------------------------------------------------------------------------------------------
Camera.type: "PinHole"

# Default D456 IR camera @ 640x480 (approximate — SDK provides exact values)
Camera.fx: 382.0
Camera.fy: 382.0
Camera.cx: 320.0
Camera.cy: 240.0

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

Camera.width: 640
Camera.height: 480
Camera.fps: 30

# Stereo baseline [m] x fx — D456 baseline = 95mm
# bf = 0.095 * 382.0 = 36.29
Camera.bf: 36.29

# Color order (IR images are grayscale)
Camera.RGB: 0

# Stereo: close/far threshold (baseline x ThDepth = max depth)
ThDepth: 40.0

# Depth map factor (RealSense depth in mm -> meters: 1/1000)
DepthMapFactor: 1000.0

#--------------------------------------------------------------------------------------------
# IMU Parameters — BMI055 (RealSense D456)
#--------------------------------------------------------------------------------------------
IMU.NoiseGyro:   0.000244    # rad/(s*sqrt(Hz))
IMU.NoiseAcc:    0.001862    # m/(s^2*sqrt(Hz))
IMU.GyroWalk:    0.000019393 # rad/(s^2*sqrt(Hz))
IMU.AccWalk:     0.003       # m/(s^3*sqrt(Hz))
IMU.Frequency:   200         # Hz

# IMU-to-camera extrinsic (body=IMU frame, sensor=left IR camera)
# (exact transform from RealSense factory calibration at runtime)
Tbc: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [ 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0]

# Stereo rectified
Camera.newHeight: 0
Camera.newWidth: 0

#--------------------------------------------------------------------------------------------
# ORB Feature Extractor
#--------------------------------------------------------------------------------------------
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
D456EOF
info "Created D456 config: ${INSTALL_PREFIX}/examples/configs/RealSense_D456.yaml"

# ── Test scripts (using D435i binaries — compatible with D456) ───────────────
# Note: ORB-SLAM3 *_realsense_D435i examples use librealsense SDK directly,
# which auto-detects any connected RealSense camera (D435i/D455/D456).
# The only difference is the settings YAML (IMU noise, baseline).

cat > "${INSTALL_PREFIX}/bin/test_realsense_mono.sh" << 'RSEOF'
#!/usr/bin/env bash
# Test ORB-SLAM3 Monocular mode with RealSense D456 (live camera)
# Uses D435i binary (librealsense auto-detects D456)
set -euo pipefail

VOCAB="${1:-/opt/orb_slam3/Vocabulary/ORBvoc.txt}"
SETTINGS="${2:-/opt/orb_slam3/examples/configs/RealSense_D456.yaml}"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "WARNING: DISPLAY not set. Pangolin/OpenCV windows may not appear."
  echo "  Try: export DISPLAY=:0  (or :1 if using VNC)"
fi

# ORB-SLAM3 spawns threads that ignore SIGINT — trap and kill the process group
cleanup() {
  echo ""
  echo "Shutting down ORB-SLAM3..."
  kill -SIGTERM -- -$$ 2>/dev/null || true
  sleep 1
  kill -SIGKILL -- -$$ 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

echo "Starting ORB-SLAM3 Monocular with RealSense D456..."
echo "  Vocabulary: ${VOCAB}"
echo "  Settings  : ${SETTINGS}"
echo "  Press Ctrl+C to stop."
echo ""

/opt/orb_slam3/examples/mono_realsense_D435i "${VOCAB}" "${SETTINGS}" &
wait $!
RSEOF
chmod +x "${INSTALL_PREFIX}/bin/test_realsense_mono.sh"

cat > "${INSTALL_PREFIX}/bin/test_realsense_stereo_inertial.sh" << 'RSEOF'
#!/usr/bin/env bash
# Test ORB-SLAM3 Stereo-Inertial mode with RealSense D456 (live camera)
set -euo pipefail

VOCAB="${1:-/opt/orb_slam3/Vocabulary/ORBvoc.txt}"
SETTINGS="${2:-/opt/orb_slam3/examples/configs/RealSense_D456.yaml}"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "WARNING: DISPLAY not set. Pangolin/OpenCV windows may not appear."
  echo "  Try: export DISPLAY=:0"
fi

cleanup() {
  echo ""
  echo "Shutting down ORB-SLAM3..."
  kill -SIGTERM -- -$$ 2>/dev/null || true
  sleep 1
  kill -SIGKILL -- -$$ 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

echo "Starting ORB-SLAM3 Stereo-Inertial with RealSense D456..."
echo "  Vocabulary: ${VOCAB}"
echo "  Settings  : ${SETTINGS}"
echo ""

/opt/orb_slam3/examples/stereo_inertial_realsense_D435i "${VOCAB}" "${SETTINGS}" &
wait $!
RSEOF
chmod +x "${INSTALL_PREFIX}/bin/test_realsense_stereo_inertial.sh"

cat > "${INSTALL_PREFIX}/bin/test_realsense_rgbd.sh" << 'RSEOF'
#!/usr/bin/env bash
# Test ORB-SLAM3 RGB-D mode with RealSense D456 (live camera)
set -euo pipefail

VOCAB="${1:-/opt/orb_slam3/Vocabulary/ORBvoc.txt}"
SETTINGS="${2:-/opt/orb_slam3/examples/configs/RealSense_D456.yaml}"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "WARNING: DISPLAY not set. Pangolin/OpenCV windows may not appear."
  echo "  Try: export DISPLAY=:0"
fi

cleanup() {
  echo ""
  echo "Shutting down ORB-SLAM3..."
  kill -SIGTERM -- -$$ 2>/dev/null || true
  sleep 1
  kill -SIGKILL -- -$$ 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

echo "Starting ORB-SLAM3 RGB-D with RealSense D456..."
echo "  Vocabulary: ${VOCAB}"
echo "  Settings  : ${SETTINGS}"
echo ""

/opt/orb_slam3/examples/rgbd_realsense_D435i "${VOCAB}" "${SETTINGS}" &
wait $!
RSEOF
chmod +x "${INSTALL_PREFIX}/bin/test_realsense_rgbd.sh"

info "RealSense D456 test scripts created in ${INSTALL_PREFIX}/bin/"

# =============================================================================
# 6. Set up LD_LIBRARY_PATH and ldconfig
# =============================================================================
step "Step 6: Library Path Configuration"

# Add to ldconfig so .so files are found at runtime
${SUDO} sh -c "echo '${INSTALL_PREFIX}/lib' > /etc/ld.so.conf.d/orb_slam3.conf"
${SUDO} ldconfig

# Create env setup script
cat > "${INSTALL_PREFIX}/setup.sh" << ENVEOF
#!/usr/bin/env bash
# Source this file to set up ORB-SLAM3 environment:
#   source ${INSTALL_PREFIX}/setup.sh
export ORB_SLAM3_ROOT="${INSTALL_PREFIX}"
export LD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
export PATH="${INSTALL_PREFIX}/bin:${INSTALL_PREFIX}/examples:\${PATH}"

# X11 display for Pangolin/OpenCV windows
if [[ -z "\${DISPLAY:-}" ]]; then
  export DISPLAY=:0
fi

echo "ORB-SLAM3 environment loaded."
echo "  ORB_SLAM3_ROOT=\${ORB_SLAM3_ROOT}"
echo "  DISPLAY=\${DISPLAY}"
ENVEOF
chmod +x "${INSTALL_PREFIX}/setup.sh"

info "Environment setup script: source ${INSTALL_PREFIX}/setup.sh"

# =============================================================================
# 7. Verification
# =============================================================================
step "Step 7: Verification"

info "Checking installed libraries..."
echo "  Libraries:"
ls -la "${INSTALL_PREFIX}/lib/"*.so 2>/dev/null || warn "No .so files found!"
echo ""
echo "  Vocabulary:"
ls -la "${INSTALL_PREFIX}/Vocabulary/" 2>/dev/null || warn "No vocabulary found!"
echo ""
echo "  Example binaries:"
ls "${INSTALL_PREFIX}/examples/" 2>/dev/null | grep -v "\.sh$" | grep -v "configs" | head -20 || warn "No example binaries found!"
echo ""

# Check OpenCV
info "OpenCV verification:"
python3 -c "
import cv2
print(f'  Version: {cv2.__version__}')
try:
    n = cv2.cuda.getCudaEnabledDeviceCount()
    print(f'  CUDA devices: {n}')
except:
    print('  CUDA: not available (CPU mode)')
" 2>/dev/null || warn "OpenCV python binding check failed"

# Check Pangolin
info "Pangolin verification:"
if pkg-config --exists pangolin 2>/dev/null; then
  echo "  Version: $(pkg-config --modversion pangolin 2>/dev/null || echo 'unknown')"
  echo "  Libs: $(pkg-config --libs pangolin 2>/dev/null | tr ' ' '\n' | head -5)"
else
  warn "Pangolin pkg-config not found (may still work via cmake)"
fi

# Check .so dependencies resolve
info "Library dependency check:"
for LIB in libORB_SLAM3.so libDBoW2.so libg2o.so; do
  if [[ -f "${INSTALL_PREFIX}/lib/${LIB}" ]]; then
    MISSING=$(ldd "${INSTALL_PREFIX}/lib/${LIB}" 2>/dev/null | grep "not found" || true)
    if [[ -z "${MISSING}" ]]; then
      echo "  ${LIB}: OK (all dependencies resolved)"
    else
      warn "${LIB}: missing dependencies:"
      echo "${MISSING}"
    fi
  fi
done

# =============================================================================
# Done!
# =============================================================================
echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  ORB-SLAM3 installation complete!${NC}"
echo -e "${GREEN}  Install prefix: ${INSTALL_PREFIX}${NC}"
echo -e "${GREEN}  GCC ${GCC_VERSION} / C++${CXX_STANDARD} / ${ARCH}${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${CYAN}Quick start:${NC}"
echo "  # Load environment"
echo "  source ${INSTALL_PREFIX}/setup.sh"
echo ""
echo "  # Test with EuRoC dataset (download first)"
echo "  bash ${INSTALL_PREFIX}/examples/download_test_data.sh"
echo ""
echo "  # Test with RealSense D456 (live camera)"
echo "  bash ${INSTALL_PREFIX}/bin/test_realsense_mono.sh"
echo "  bash ${INSTALL_PREFIX}/bin/test_realsense_stereo_inertial.sh"
echo "  bash ${INSTALL_PREFIX}/bin/test_realsense_rgbd.sh"
echo ""
echo "  # Build the ROS2 package"
echo "  cd /workspaces/isaac_ros-dev"
echo "  colcon build --packages-select isaac_ros_visual_slam_orb"
echo ""
