// Copyright (c) 2025, Isaac ROS Visual SLAM ORB Contributors
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_visual_slam_orb/impl/cuda_image_pipeline.hpp"

#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace isaac_ros::visual_slam_orb
{

// ---------------------------------------------------------------------------
// Helper: map ROS image encoding string → OpenCV cvtColor code
// ---------------------------------------------------------------------------
static int EncodingToCvtCode(const std::string & enc)
{
  if (enc == "rgb8")  { return cv::COLOR_RGB2GRAY; }
  if (enc == "bgr8")  { return cv::COLOR_BGR2GRAY; }
  if (enc == "rgba8") { return cv::COLOR_RGBA2GRAY; }
  if (enc == "bgra8") { return cv::COLOR_BGRA2GRAY; }
  if (enc == "mono8") { return -1; }           // already grayscale
  if (enc == "rgb16") { return cv::COLOR_RGB2GRAY; }
  if (enc == "bgr16") { return cv::COLOR_BGR2GRAY; }
  if (enc == "mono16"){ return -1; }
  throw std::invalid_argument("CudaImagePipeline: unsupported encoding: " + enc);
}

// ---------------------------------------------------------------------------
CudaImagePipeline::CudaImagePipeline(const Config & cfg)
: cfg_(cfg)
{
#if VSLAM_ORB_CUDA_PIPELINE
  if (cfg_.enable_denoising) {
    gaussian_filter_ = cv::cuda::createGaussianFilter(
      CV_8UC1, CV_8UC1, cv::Size(3, 3), 0.0);
  }
#endif
}

CudaImagePipeline::~CudaImagePipeline() = default;

// ---------------------------------------------------------------------------
void CudaImagePipeline::SetRectifyMaps(
  const cv::Mat & K, const cv::Mat & D,
  const cv::Mat & R, const cv::Mat & P,
  cv::Size image_size)
{
  cv::initUndistortRectifyMap(
    K, D, R, P, image_size, CV_32FC1, map1_cpu_, map2_cpu_);

#if VSLAM_ORB_CUDA_PIPELINE
  map1_gpu_.upload(map1_cpu_);
  map2_gpu_.upload(map2_cpu_);
#endif

  maps_ready_ = true;
}

// ---------------------------------------------------------------------------
cv::Mat CudaImagePipeline::ProcessGpu(
  const void * gpu_ptr,
  int width, int height, int stride)
{
#if VSLAM_ORB_CUDA_PIPELINE
  // 1. Wrap the raw GPU buffer — zero copy
  const int cvtype = (cfg_.input_encoding == "mono8") ? CV_8UC1 : CV_8UC3;
  cv::cuda::GpuMat raw(
    height, width, cvtype,
    const_cast<void *>(gpu_ptr),
    static_cast<size_t>(stride));

  // 2. Convert to grayscale on GPU
  cv::cuda::GpuMat gray;
  const int code = EncodingToCvtCode(cfg_.input_encoding);
  if (code < 0) {
    raw.copyTo(gray, stream_);
  } else {
    cv::cuda::cvtColor(raw, gray, code, 0, stream_);
  }

  // 3. Denoise (Gaussian blur) on GPU
  if (cfg_.enable_denoising) {
    cv::cuda::GpuMat blurred;
    gaussian_filter_->apply(gray, blurred, stream_);
    gray = std::move(blurred);
  }

  // 4. Undistort + rectify on GPU
  cv::cuda::GpuMat rectified;
  if (cfg_.rectify && maps_ready_) {
    cv::cuda::remap(gray, rectified, map1_gpu_, map2_gpu_,
      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), stream_);
  } else {
    rectified = std::move(gray);
  }

  // 5. Download to CPU (single DMA transfer)
  cv::Mat out;
  rectified.download(out, stream_);
  stream_.waitForCompletion();
  return out;

#else
  // CPU fallback: create a Mat header over the GPU pointer — this triggers
  // an implicit cudaMemcpy when OpenCV accesses it.
  // NOTE: This path is slow; prefer building OpenCV with CUDA.
  const int cvtype = (cfg_.input_encoding == "mono8") ? CV_8UC1 : CV_8UC3;
  cv::Mat raw(height, width, cvtype, const_cast<void *>(gpu_ptr), stride);
  cv::Mat cpu_copy;
  raw.copyTo(cpu_copy);   // forces device→host copy via unified memory / cudaMemcpy
  return ProcessCpu(cpu_copy);
#endif
}

// ---------------------------------------------------------------------------
cv::Mat CudaImagePipeline::ProcessCpu(const cv::Mat & input)
{
  cv::Mat gray;

  // 1. Convert to grayscale
  const int code = EncodingToCvtCode(cfg_.input_encoding);
  if (code < 0) {
    gray = input.clone();
  } else {
    cv::cvtColor(input, gray, code);
  }

  // 2. Denoise
  if (cfg_.enable_denoising) {
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.0);
  }

  // 3. Undistort + rectify
  if (cfg_.rectify && maps_ready_) {
    cv::Mat rectified;
    cv::remap(gray, rectified, map1_cpu_, map2_cpu_,
      cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return rectified;
  }

  return gray;
}

}  // namespace isaac_ros::visual_slam_orb
