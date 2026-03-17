cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_visual_slam_orb LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-error)
endif()
# ORB-SLAM3 headers include Pangolin GL code with CUDA 12 incompatibilities.
# Since we never use the viewer, suppress these as non-fatal.
add_compile_options(-fpermissive)
# Pangolin headers require GL extension prototypes to be declared
add_compile_definitions(GL_GLEXT_PROTOTYPES)

# ---------------------------------------------------------------------------
# Find ament / ROS2 dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ---------------------------------------------------------------------------
# Eigen3
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# ---------------------------------------------------------------------------
# OpenCV (4.x, optionally with CUDA modules)
# ---------------------------------------------------------------------------
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d features2d)

# Detect optional CUDA OpenCV modules and define compile-time guards
foreach(_mod cudaimgproc cudawarping cudafilters cudafeatures2d)
  if(OpenCV_${_mod}_FOUND OR TARGET opencv_${_mod})
    message(STATUS "OpenCV CUDA module found: ${_mod}")
    add_compile_definitions(HAVE_OPENCV_${_mod})
  endif()
endforeach()

# ---------------------------------------------------------------------------
# CUDA (for ORB kernels compiled with nvcc)
# ---------------------------------------------------------------------------
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
  enable_language(CUDA)
  add_compile_definitions(HAVE_CUDA)
  message(STATUS "CUDA ${CUDAToolkit_VERSION} found — GPU paths enabled.")
else()
  message(STATUS "CUDA not found — CPU-only build.")
endif()

# ---------------------------------------------------------------------------
# OpenGL / GLEW (required by ORB-SLAM3 via Pangolin)
# ---------------------------------------------------------------------------
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# ---------------------------------------------------------------------------
# Pangolin
# ---------------------------------------------------------------------------
find_package(Pangolin REQUIRED)

# ---------------------------------------------------------------------------
# ORB-SLAM3 (installed to /opt/orb_slam3 by scripts/install_orb_slam3.sh)
# ---------------------------------------------------------------------------
set(ORB_SLAM3_ROOT "/opt/orb_slam3" CACHE PATH "ORB-SLAM3 install root")

find_library(ORB_SLAM3_LIB ORB_SLAM3
  PATHS ${ORB_SLAM3_ROOT}/lib
  NO_DEFAULT_PATH)
find_library(DBOW2_LIB DBoW2
  PATHS ${ORB_SLAM3_ROOT}/lib
  NO_DEFAULT_PATH)
find_library(G2O_LIB g2o
  PATHS ${ORB_SLAM3_ROOT}/lib
  NO_DEFAULT_PATH)

if(NOT ORB_SLAM3_LIB)
  message(FATAL_ERROR
    "ORB-SLAM3 library not found in ${ORB_SLAM3_ROOT}/lib.\n"
    "Run:  bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh")
endif()

find_path(ORB_SLAM3_INCLUDE System.h
  PATHS ${ORB_SLAM3_ROOT}/include
  NO_DEFAULT_PATH)

message(STATUS "ORB-SLAM3 lib:     ${ORB_SLAM3_LIB}")
message(STATUS "ORB-SLAM3 include: ${ORB_SLAM3_INCLUDE}")

# Sophus (header-only, installed alongside ORB-SLAM3)
find_path(SOPHUS_INCLUDE sophus/se3.hpp
  PATHS ${ORB_SLAM3_ROOT}/include/Thirdparty/Sophus
        ${ORB_SLAM3_ROOT}/include/Sophus
        /usr/local/include
  NO_DEFAULT_PATH)
if(NOT SOPHUS_INCLUDE)
  message(FATAL_ERROR "Sophus headers not found.")
endif()

# DBoW3 / g2o bundled inside ORB-SLAM3 Thirdparty directory
set(ORB_SLAM3_THIRDPARTY "${ORB_SLAM3_ROOT}/include/Thirdparty")

# ---------------------------------------------------------------------------
# isaac_common include (MessageStreamSynchronizer, MessageBuffer)
# ---------------------------------------------------------------------------
find_package(isaac_common REQUIRED)

# ---------------------------------------------------------------------------
# Shared library: visual_slam_orb_node
# ---------------------------------------------------------------------------
set(SOURCES
  src/impl/cuda_image_pipeline.cpp
  src/impl/cuda_orb_extractor.cpp
  src/impl/ros_orb_conversion.cpp
  src/orb_slam3_backend.cpp
  src/visual_slam_node.cpp
)

ament_auto_add_library(visual_slam_orb_node SHARED ${SOURCES})

target_include_directories(visual_slam_orb_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# ORB-SLAM3 and its Thirdparty headers trigger many warnings from upstream code.
# Mark them as SYSTEM includes to suppress warnings-as-errors.
target_include_directories(visual_slam_orb_node SYSTEM PUBLIC
  ${ORB_SLAM3_INCLUDE}
  ${ORB_SLAM3_INCLUDE}/CameraModels
  ${ORB_SLAM3_THIRDPARTY}
  ${ORB_SLAM3_THIRDPARTY}/Sophus
  ${SOPHUS_INCLUDE}
  /usr/local/include
)

target_link_libraries(visual_slam_orb_node
  ${ORB_SLAM3_LIB}
  ${DBOW2_LIB}
  ${G2O_LIB}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  OpenGL::GL
  GLEW::GLEW
)

if(CUDAToolkit_FOUND)
  target_link_libraries(visual_slam_orb_node CUDA::cudart)
endif()

rclcpp_components_register_nodes(
  visual_slam_orb_node
  "isaac_ros::visual_slam_orb::VisualSlamNode")

# ---------------------------------------------------------------------------
# Standalone executable
# ---------------------------------------------------------------------------
ament_auto_add_executable(isaac_ros_visual_slam_orb
  src/visual_slam_main.cpp)
target_link_libraries(isaac_ros_visual_slam_orb visual_slam_orb_node)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS visual_slam_orb_node isaac_ros_visual_slam_orb
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------------------------------------------------------------------
ament_auto_package()
