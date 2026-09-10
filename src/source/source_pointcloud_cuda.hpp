/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include "source/source.hpp"
#include "msg/rs_msg/lidar_point_cloud_msg.hpp"

#ifdef ROS2_FOUND

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cuda_runtime_api.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace robosense
{
namespace lidar
{

// Raw per-lidar point cloud destination published through the CUDA blackboard.
// Points stay on the GPU end to end: the host-side LidarPointCloudMsg is packed into
// device memory and the negotiated publisher hands over the device pointer via the blackboard.
class DestinationPointCloudCuda : virtual public DestinationPointCloud
{
public:

  void init(const YAML::Node& config) override;
  void start() override;
  void stop() override;
  void sendPointCloud(const LidarPointCloudMsg& msg) override;
  ~DestinationPointCloudCuda() override;

private:

  void spin();

  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_started_{false};

  std::shared_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>> pub_;

  std::string cuda_send_topic_;
  std::string frame_id_;
  bool send_by_rows_;
  size_t pool_slots_;
  size_t pool_slot_bytes_;
  std::unique_ptr<cuda_blackboard::GpuPointCloudMemoryPool> pool_;
};

inline void DestinationPointCloudCuda::init(const YAML::Node& config)
{
  yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, false);

  bool dense_points;
  yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
  if (dense_points)
  {
    send_by_rows_ = false;
  }

  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "rslidar");

  yamlRead<std::string>(config["ros"],
      "cuda_send_point_cloud_topic", cuda_send_topic_, "rslidar_points_raw_cuda");

  yamlRead<size_t>(config["ros"], "cuda_point_cloud_pool_slots", pool_slots_, 4);
  yamlRead<size_t>(config["ros"], "cuda_point_cloud_pool_slot_bytes", pool_slot_bytes_, 16 * 1024 * 1024);

  cuda_blackboard::GpuPointCloudMemoryPoolConfig pool_config;
  pool_config.slot_count = pool_slots_;
  pool_config.device_slot_bytes = pool_slot_bytes_;
  pool_ = std::make_unique<cuda_blackboard::GpuPointCloudMemoryPool>(pool_config);

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "rslidar_points_cuda_destination_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));

  pub_ = std::make_shared<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *node_ptr_, cuda_send_topic_);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_ptr_);

  // This node must spin for the negotiated publisher inside it to process the
  // subscribers' SupportedTypes reports and run negotiate(). Neither the standalone
  // executable nor the composable-node path ever calls start() on destinations, so
  // start the spin thread here (idempotent) to keep the CUDA negotiation alive.
  start();
}

inline void DestinationPointCloudCuda::start()
{
  if (!executor_ || spin_started_.exchange(true))
  {
    return;
  }
  spin_thread_ = std::thread(std::bind(&DestinationPointCloudCuda::spin, this));
}

inline void DestinationPointCloudCuda::stop()
{
  if (executor_)
  {
    executor_->cancel();
  }
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
}

inline DestinationPointCloudCuda::~DestinationPointCloudCuda()
{
  stop();
}

inline void DestinationPointCloudCuda::spin()
{
  executor_->spin();
}

inline void DestinationPointCloudCuda::sendPointCloud(const LidarPointCloudMsg& msg)
{
  if (!pub_ || !pool_ || msg.points.empty())
  {
    return;
  }

  const size_t point_count = msg.points.size();
  const size_t point_step = 16;  // x/y/z(float) + intensity/return_type(uint8) + channel(uint16) + pad
  const size_t bytes = point_count * point_step;
  if (bytes > pool_->device_slot_bytes())
  {
    RCLCPP_WARN(
      node_ptr_->get_logger(), "CUDA point cloud %s needs %zu bytes, larger than pool slot %zu bytes. Skipping.",
      cuda_send_topic_.c_str(), bytes, pool_->device_slot_bytes());
    return;
  }

  auto lease = pool_->try_acquire(bytes);
  if (!lease)
  {
    RCLCPP_WARN(
      node_ptr_->get_logger(), "CUDA point cloud pool exhausted for %s. Skipping frame.",
      cuda_send_topic_.c_str());
    return;
  }

  // Pack to a dense host buffer (16B per point), then copy to device memory in one shot.
  std::vector<uint8_t> host_data(bytes);
  for (size_t i = 0; i < point_count; ++i)
  {
    const LidarPointCloudMsg::PointT& p = msg.points[i];
    float x = p.x;
    float y = p.y;
    float z = p.z;
    uint8_t intensity = p.intensity;
    uint8_t return_type = p.feature;
    uint16_t channel = p.ring;

    uint8_t* dst = host_data.data() + i * point_step;
    std::memcpy(dst + 0, &x, sizeof(float));
    std::memcpy(dst + 4, &y, sizeof(float));
    std::memcpy(dst + 8, &z, sizeof(float));
    std::memcpy(dst + 12, &intensity, sizeof(uint8_t));
    std::memcpy(dst + 13, &return_type, sizeof(uint8_t));
    std::memcpy(dst + 14, &channel, sizeof(uint16_t));
  }

  cudaError_t err = cudaMemcpyAsync(lease->slot().device_data, host_data.data(), bytes,
      cudaMemcpyHostToDevice, 0);
  if (err != cudaSuccess || cudaStreamSynchronize(0) != cudaSuccess)
  {
    RCLCPP_WARN(
      node_ptr_->get_logger(), "Failed to upload CUDA point cloud to device memory for %s",
      cuda_send_topic_.c_str());
    return;
  }

  sensor_msgs::msg::PointCloud2 metadata;
  metadata.header.frame_id = frame_id_;
  metadata.header.stamp.sec = static_cast<uint32_t>(std::floor(msg.timestamp));
  metadata.header.stamp.nanosec = static_cast<uint32_t>(std::round((msg.timestamp - std::floor(msg.timestamp)) * 1e9));
  metadata.height = 1;
  metadata.width = static_cast<uint32_t>(point_count);
  metadata.is_dense = msg.is_dense;
  metadata.is_bigendian = false;
  metadata.point_step = static_cast<uint32_t>(point_step);
  metadata.row_step = metadata.point_step * metadata.width;

  addPointField(metadata, "x", 1, sensor_msgs::msg::PointField::FLOAT32, 0);
  addPointField(metadata, "y", 1, sensor_msgs::msg::PointField::FLOAT32, 4);
  addPointField(metadata, "z", 1, sensor_msgs::msg::PointField::FLOAT32, 8);
  addPointField(metadata, "intensity", 1, sensor_msgs::msg::PointField::UINT8, 12);
  addPointField(metadata, "return_type", 1, sensor_msgs::msg::PointField::UINT8, 13);
  addPointField(metadata, "channel", 1, sensor_msgs::msg::PointField::UINT16, 14);

  pub_->publish(std::make_unique<cuda_blackboard::CudaPointCloud2>(metadata, std::move(*lease)));
}

}  // namespace lidar
}  // namespace robosense

#endif  // ROS2_FOUND
