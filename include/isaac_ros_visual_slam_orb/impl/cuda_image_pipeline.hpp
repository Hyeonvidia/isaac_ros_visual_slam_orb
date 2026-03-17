// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

#include <opencv2/core/mat.hpp>

// Conditional CUDA-OpenCV support.
// When OpenCV is built with CUDA (as in the isaac_ros Docker image),
// we process entirely on GPU.  The CPU fallback keeps the code buildable
// even without CUDA OpenCV modules.
#if defined(HAVE_OPENCV_CUDAIMGPROC)
#  define VSLAM_ORB_CUDA_PIPELINE 1
#  include <opencv2/core/cuda.hpp>
#  include <opencv2/cudaimgproc.hpp>
#  include <opencv2/cudawarping.hpp>
#  include <opencv2/cudafilters.hpp>
#else
#  define VSLAM_ORB_CUDA_PIPELINE 0
#endif

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// CudaImagePipeline
//
// Converts a raw camera frame (either from a GPU pointer delivered by
// NitrosImageView, or a CPU cv::Mat) into a MONO8 cv::Mat ready for the
// ORB extractor.
//
// Steps (when CUDA OpenCV is available):
//   1. Wrap raw GPU buffer → cv::cuda::GpuMat  (zero-copy)
//   2. cvtColor  RGB8/BGR8 → GRAY8             (on GPU)
//   3. Gaussian blur (denoising, optional)      (on GPU)
//   4. Remap (undistort + rectify, optional)    (on GPU)
//   5. Download to CPU cv::Mat                  (one DMA transfer)
//
// When CUDA OpenCV is NOT available:
//   - Steps 2-4 are performed on CPU.
// ---------------------------------------------------------------------------
class CudaImagePipeline
{
public:
  struct Config
  {
    bool enable_denoising{true};   // apply 3×3 Gaussian blur
    bool rectify{true};            // apply undistort + stereo rectification
    // encoding of the incoming raw frame; accepted values:
    //   "mono8", "rgb8", "bgr8", "rgb16", "bgr16"
    std::string input_encoding{"mono8"};
  };

  explicit CudaImagePipeline(const Config & cfg);
  ~CudaImagePipeline();

  // Pre-compute the rectification maps from the camera matrix K (3×3, row-major),
  // the distortion coefficients D, and the stereo rectification rotation R (3×3)
  // and new projection matrix P (3×4).  Must be called before Process().
  void SetRectifyMaps(
    const cv::Mat & K, const cv::Mat & D,
    const cv::Mat & R, const cv::Mat & P,
    cv::Size image_size);

  // Process a GPU-side raw frame (pointer obtained from
  // NitrosImageView::GetGpuData()) into an undistorted/rectified MONO8 Mat.
  // width, height, stride are in pixels / bytes respectively.
  cv::Mat ProcessGpu(
    const void * gpu_ptr,
    int width, int height, int stride);

  // Process a CPU-side frame.  Useful as a fallback and for unit tests.
  cv::Mat ProcessCpu(const cv::Mat & input);

private:
  Config cfg_;

  // Rectification maps (CPU and GPU variants)
  cv::Mat map1_cpu_, map2_cpu_;

#if VSLAM_ORB_CUDA_PIPELINE
  cv::cuda::GpuMat map1_gpu_, map2_gpu_;
  cv::Ptr<cv::cuda::Filter> gaussian_filter_;
  cv::cuda::Stream stream_;
#endif

  bool maps_ready_{false};
};

}  // namespace isaac_ros::visual_slam_orb
