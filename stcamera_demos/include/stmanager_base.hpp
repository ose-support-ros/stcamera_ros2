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

#ifndef STMANAGER_BASE_HPP
#define STMANAGER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include "stcamera_components/stheader.hpp"
#include "stcamera_components/stcamera_msgs.hpp"

namespace stcamera
{
  class CStManagerBase : public rclcpp::Node
  {
    public:
      CStManagerBase(const std::string &pub_name_space, const std::string &pub_node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
      CStManagerBase(const std::string &pub_name_space, const std::string &pub_node_name, const std::string& my_name_space , const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
      virtual ~CStManagerBase();


    public:
      //Services
      EOSError_t getDeviceList(std::vector<stcamera_msgs::msg::DeviceConnection> &device_connection_list);  // GetDeviceList.srv
      EOSError_t getModuleList(std::vector<std::string> &genicam_module_list);  // GetModuleList.srv
      EOSError_t getSDKInfo(std::string &sdk_version, std::vector<stcamera_msgs::msg::GenTLInfo> &gentl_info_list); // GetSDKInfo.srv
     protected:
      std::shared_ptr<rclcpp::Node> GetPtr(){return shared_from_this();}
    protected:
      const std::string pub_name_space_;
      const std::string pub_node_name_;
  };
}
#endif //STMANAGER_BASE_HPP