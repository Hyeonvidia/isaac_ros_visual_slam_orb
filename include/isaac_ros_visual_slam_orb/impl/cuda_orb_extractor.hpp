// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#if defined(HAVE_OPENCV_CUDAFEATURES2D)
#  define VSLAM_ORB_CUDA_ORB 1
#  include <opencv2/core/cuda.hpp>
#  include <opencv2/cudafeatures2d.hpp>
#else
#  define VSLAM_ORB_CUDA_ORB 0
#  include <opencv2/features2d.hpp>
#endif

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// CudaOrbExtractor
//
// Wraps cv::cuda::ORB (GPU) with a CPU fallback.
// Designed as a drop-in replacement for ORB-SLAM3's internal ORBextractor,
// so OrbSlam3Backend can inject it into the System before tracking begins.
//
// Jetson AGX Orin + CUDA 12.6 typical throughput (640×480 MONO8, nfeatures=1000):
//   CPU ORB  ≈ 18 ms/frame   →   GPU ORB  ≈ 4 ms/frame
// ---------------------------------------------------------------------------
class CudaOrbExtractor
{
public:
  struct Config
  {
    int num_features{1000};
    float scale_factor{1.2f};
    int num_levels{8};
    int ini_threshold_fast{20};  // initial FAST threshold
    int min_threshold_fast{7};   // minimum FAST threshold (fallback)
  };

  explicit CudaOrbExtractor(const Config & cfg);
  ~CudaOrbExtractor();

  // ORB-SLAM3-compatible operator():
  //   in  : grayscale MONO8 image (CPU)
  //   in  : mask (can be empty)
  //   out : keypoints, descriptors (on CPU for g2o / Bundle Adjustment)
  void operator()(
    const cv::Mat & image,
    const cv::Mat & mask,
    std::vector<cv::KeyPoint> & keypoints,
    cv::Mat & descriptors);

  // GPU-direct path: accepts GpuMat, downloads descriptors only when needed.
#if VSLAM_ORB_CUDA_ORB
  void ExtractGpu(
    const cv::cuda::GpuMat & image_gpu,
    const cv::cuda::GpuMat & mask_gpu,
    std::vector<cv::KeyPoint> & keypoints,
    cv::Mat & descriptors);
#endif

  // Pyramid scale info (ORB-SLAM3 queries these)
  int GetLevels() const { return cfg_.num_levels; }
  float GetScaleFactor() const { return cfg_.scale_factor; }
  std::vector<float> GetScaleFactors() const { return scale_factors_; }
  std::vector<float> GetInverseScaleFactors() const { return inv_scale_factors_; }
  std::vector<float> GetScaleSigmaSquares() const { return scale_sigma_sq_; }
  std::vector<float> GetInverseScaleSigmaSquares() const { return inv_scale_sigma_sq_; }

private:
  void BuildScalePyramidInfo();

  Config cfg_;
  std::vector<float> scale_factors_;
  std::vector<float> inv_scale_factors_;
  std::vector<float> scale_sigma_sq_;
  std::vector<float> inv_scale_sigma_sq_;

#if VSLAM_ORB_CUDA_ORB
  cv::Ptr<cv::cuda::ORB> orb_gpu_;
#else
  cv::Ptr<cv::ORB> orb_cpu_;
#endif
};

}  // namespace isaac_ros::visual_slam_orb
