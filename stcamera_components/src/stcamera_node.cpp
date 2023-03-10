/******************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2023, OMRON SENTECH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *   * Neither the names of OMRON SENTECH nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "stcamera_node.hpp"
#include "impl/stcamera_node_impl.hpp"
#include "stcamera_msgs/msg/device_connection.hpp"

#include "stheader.hpp"

namespace stcamera
{
  using namespace std;
  using namespace std::placeholders;

  StCameraNode::StCameraNode(const rclcpp::NodeOptions& options):
    Node(STCAMERA_NODE_NAME, options),
    st_camera_node_impl_(new StCameraNodeImpl(this))
  {
    const rclcpp::Logger::Level logger_level = param_.getLoggerLevel(this);
    get_logger().set_level(logger_level);
    RCLCPP_INFO(get_logger(), "Setting severity threshold to %d", (int)logger_level);

    init();
  }

  StCameraNode::~StCameraNode()
  {
    delete st_camera_node_impl_;
  }

  void StCameraNode::init()
  {
    st_camera_node_impl_->initSystemsAndInterfaces();
    initPublishers();
    initServices();
    timer_ = create_wall_timer(1s, std::bind(&StCameraNodeImpl::DetectingAndOpenningDevices, st_camera_node_impl_));
  }

  void StCameraNode::initServices()
  {
    const std::string prefix = "~/";
    srv_get_device_list_ = create_service<stcamera_msgs::srv::GetDeviceList>(prefix + std::string(STSRV_G_device_list), std::bind(&StCameraNodeImpl::getDeviceListCallback, st_camera_node_impl_, _1, _2));
    srv_get_module_list_ = create_service<stcamera_msgs::srv::GetModuleList>(prefix + std::string(STSRV_G_module_list), std::bind(&StCameraNodeImpl::getModuleListCallback, st_camera_node_impl_, _1, _2));
    srv_get_sdk_info_ = create_service<stcamera_msgs::srv::GetSDKInfo>(prefix + std::string(STSRV_G_sdk_info), std::bind(&StCameraNodeImpl::getSDKInfoCallback, st_camera_node_impl_, _1, _2));
  }
  void StCameraNode::initPublishers()
  {
    const std::string prefix = "~/";
    msg_device_connection_ = create_publisher<stcamera_msgs::msg::DeviceConnection>(prefix + std::string(STMSG_device_connection), STCAMERA_QUEUE_SIZE);
  }
} // end of namespace stcamera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(stcamera::StCameraNode)