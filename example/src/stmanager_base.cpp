/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2023, OMRON SENTECH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of OMRON SENTECH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
#include "../include/stmanager_base.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace stcamera
{
  CStManagerBase::CStManagerBase(const std::string &pub_name_space, const std::string &pub_node_name, const rclcpp::NodeOptions & options) : 
      CStManagerBase(pub_name_space, pub_node_name, "", options)
      {

      }
  CStManagerBase::CStManagerBase(const std::string &pub_name_space, const std::string &pub_node_name, const std::string& my_name_space , const rclcpp::NodeOptions & options) : 
      Node("stmanager_base", my_name_space, options), pub_name_space_(pub_name_space), pub_node_name_(pub_node_name)
    {
    }
  CStManagerBase::~CStManagerBase()
    {
    }

  // GetDeviceList.srv
  EOSError_t CStManagerBase::getDeviceList(std::vector<stcamera_msgs::msg::DeviceConnection> &device_connection_list)
  {
    typedef stcamera_msgs::srv::GetDeviceList service_t;
    rclcpp::Client<service_t>::SharedPtr client = 
      create_client<service_t>(pub_name_space_ + pub_node_name_ + "/get_device_list");

    auto req = std::make_shared<service_t::Request>();
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return(OSError_UnknownError);
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    auto future = client->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(GetPtr(), future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      std::shared_ptr<service_t::Response> res = future.get();
      device_connection_list = res->device_list;
      const EOSError_t err = (EOSError_t)res->error_info.error_code;
      if(err != OSError_Success)
      {
        RCLCPP_ERROR(get_logger(), res->error_info.description.c_str());
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_device_list");
    }
    return(OSError_UnknownError);
  }

  // GetModuleList.srv
  EOSError_t CStManagerBase::getModuleList(std::vector<std::string> &genicam_module_list)
  {
    typedef stcamera_msgs::srv::GetModuleList service_t;
    rclcpp::Client<service_t>::SharedPtr client = 
      create_client<service_t>(pub_name_space_ + pub_node_name_ + "/get_module_list");

    auto req = std::make_shared<service_t::Request>();
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return(OSError_UnknownError);
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    auto future = client->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(GetPtr(), future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      std::shared_ptr<service_t::Response> res = future.get();
      const EOSError_t err = (EOSError_t)res->error_info.error_code;
      if(err != OSError_Success)
      {
        RCLCPP_ERROR(get_logger(), res->error_info.description.c_str());
      }
      else
      {
        genicam_module_list = res->genicam_module_list;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_module_list");
    }
    return(OSError_UnknownError);
  }

  // GetSDKInfo.srv
  EOSError_t CStManagerBase::getSDKInfo(std::string &sdk_version, std::vector<stcamera_msgs::msg::GenTLInfo> &gentl_info_list)
  {
    typedef stcamera_msgs::srv::GetSDKInfo service_t;
    rclcpp::Client<service_t>::SharedPtr client = 
      create_client<service_t>(pub_name_space_ + pub_node_name_ + "/get_sdk_info");

    auto req = std::make_shared<service_t::Request>();
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return(OSError_UnknownError);
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
    auto future = client->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(GetPtr(), future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      std::shared_ptr<service_t::Response> res = future.get();
      const EOSError_t err = (EOSError_t)res->error_info.error_code;
      if(err != OSError_Success)
      {
        RCLCPP_ERROR(get_logger(), res->error_info.description.c_str());
      }
      else
      {
        sdk_version = res->sdk_version;
        gentl_info_list = res->gentl_info_list;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_sdk_info");
    }
    return(OSError_UnknownError);
  }
}
