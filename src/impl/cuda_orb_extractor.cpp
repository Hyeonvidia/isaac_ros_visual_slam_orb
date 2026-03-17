// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_visual_slam_orb/impl/cuda_orb_extractor.hpp"

#include <cmath>

namespace isaac_ros::visual_slam_orb
{

CudaOrbExtractor::CudaOrbExtractor(const Config & cfg)
: cfg_(cfg)
{
  BuildScalePyramidInfo();

#if VSLAM_ORB_CUDA_ORB
  // cv::cuda::ORB signature (OpenCV 4.x):
  //   create(nfeatures, scaleFactor, nlevels, edgeThreshold,
  //          firstLevel, WTA_K, scoreType, patchSize, fastThreshold,
  //          blurForDescriptor)
  orb_gpu_ = cv::cuda::ORB::create(
    cfg_.num_features,
    cfg_.scale_factor,
    cfg_.num_levels,
    /*edgeThreshold=*/31,
    /*firstLevel=*/0,
    /*WTA_K=*/2,
    cv::ORB::HARRIS_SCORE,
    /*patchSize=*/31,
    cfg_.ini_threshold_fast,
    /*blurForDescriptor=*/true);
#else
  orb_cpu_ = cv::ORB::create(
    cfg_.num_features,
    cfg_.scale_factor,
    cfg_.num_levels,
    /*edgeThreshold=*/31,
    /*firstLevel=*/0,
    /*WTA_K=*/2,
    cv::ORB::HARRIS_SCORE,
    /*patchSize=*/31,
    cfg_.ini_threshold_fast);
#endif
}

CudaOrbExtractor::~CudaOrbExtractor() = default;

void CudaOrbExtractor::BuildScalePyramidInfo()
{
  scale_factors_.resize(cfg_.num_levels);
  inv_scale_factors_.resize(cfg_.num_levels);
  scale_sigma_sq_.resize(cfg_.num_levels);
  inv_scale_sigma_sq_.resize(cfg_.num_levels);

  scale_factors_[0] = 1.0f;
  for (int i = 1; i < cfg_.num_levels; ++i) {
    scale_factors_[i] = scale_factors_[i - 1] * cfg_.scale_factor;
  }
  for (int i = 0; i < cfg_.num_levels; ++i) {
    inv_scale_factors_[i] = 1.0f / scale_factors_[i];
    scale_sigma_sq_[i] = scale_factors_[i] * scale_factors_[i];
    inv_scale_sigma_sq_[i] = 1.0f / scale_sigma_sq_[i];
  }
}

// ---------------------------------------------------------------------------
// operator() — primary interface for ORB-SLAM3 compatibility
// ---------------------------------------------------------------------------
void CudaOrbExtractor::operator()(
  const cv::Mat & image,
  const cv::Mat & mask,
  std::vector<cv::KeyPoint> & keypoints,
  cv::Mat & descriptors)
{
#if VSLAM_ORB_CUDA_ORB
  // Upload to GPU, run, download descriptors
  cv::cuda::GpuMat image_gpu, mask_gpu, desc_gpu;
  image_gpu.upload(image);
  if (!mask.empty()) {
    mask_gpu.upload(mask);
  }

  orb_gpu_->detectAndCompute(image_gpu, mask_gpu, keypoints, desc_gpu);
  desc_gpu.download(descriptors);

  // FAST threshold fallback: if too few features were found, retry with
  // lower threshold (mirrors ORB-SLAM3's INI/MIN threshold logic)
  if (static_cast<int>(keypoints.size()) < cfg_.num_features / 5) {
    orb_gpu_->setFastThreshold(cfg_.min_threshold_fast);
    orb_gpu_->detectAndCompute(image_gpu, mask_gpu, keypoints, desc_gpu);
    desc_gpu.download(descriptors);
    orb_gpu_->setFastThreshold(cfg_.ini_threshold_fast);  // restore
  }

#else
  orb_cpu_->detectAndCompute(image, mask, keypoints, descriptors);

  if (static_cast<int>(keypoints.size()) < cfg_.num_features / 5) {
    auto orb_low = cv::ORB::create(
      cfg_.num_features, cfg_.scale_factor, cfg_.num_levels,
      31, 0, 2, cv::ORB::HARRIS_SCORE, 31, cfg_.min_threshold_fast);
    orb_low->detectAndCompute(image, mask, keypoints, descriptors);
  }
#endif
}

// ---------------------------------------------------------------------------
#if VSLAM_ORB_CUDA_ORB
void CudaOrbExtractor::ExtractGpu(
  const cv::cuda::GpuMat & image_gpu,
  const cv::cuda::GpuMat & mask_gpu,
  std::vector<cv::KeyPoint> & keypoints,
  cv::Mat & descriptors)
{
  cv::cuda::GpuMat desc_gpu;
  orb_gpu_->detectAndCompute(image_gpu, mask_gpu, keypoints, desc_gpu);
  desc_gpu.download(descriptors);
}
#endif

}  // namespace isaac_ros::visual_slam_orb
