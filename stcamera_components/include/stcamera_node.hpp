/******************************************************************************
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

/** \file stcamera_node.hpp
 *  \brief Class implementation for stcamera
 *
 *  This class implements the stcamera.
 */

#ifndef STCAMERA_STCAMERA_NODE_H
#define STCAMERA_STCAMERA_NODE_H

#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

#include "stparameter.hpp"
#include "stcamera_interface.hpp"

#include "stcamera_msgs/msg/gen_tl_info.hpp"
#include "stcamera_msgs/msg/device_connection.hpp"

#include "stcamera_msgs/srv/get_device_list.hpp"
#include "stcamera_msgs/srv/get_module_list.hpp"
#include "stcamera_msgs/srv/get_sdk_info.hpp"

namespace stcamera
{
  class StCameraNodeImpl;
  /** Map key: camera namespace; value: message structure DeviceConnection */
  typedef std::map<std::string, stcamera_msgs::msg::DeviceConnection> 
      MapDeviceConnection;

  /** \class StCameraNode
   *  \brief Class implementation for stcamera
   *
   *  This class implements the stcamera.
   */ 
  class StCameraNode: public rclcpp::Node
  {
    public:

      /** Default Constructor.
       */
      COMPOSITION_PUBLIC
      explicit StCameraNode(const rclcpp::NodeOptions& options);

      /** Destructor.
       */
      virtual ~StCameraNode();

    protected:
      /** StParameter instance */
      StParameter param_;

      /** Publisher for announcing any device lost or newly found device that
       * is allowed to connect. */
      rclcpp::Publisher<stcamera_msgs::msg::DeviceConnection>::SharedPtr msg_device_connection_;

      /** Server for get_device_list */
      rclcpp::Service<stcamera_msgs::srv::GetDeviceList>::SharedPtr       srv_get_device_list_;
      /** Server for get_module_list */
      rclcpp::Service<stcamera_msgs::srv::GetModuleList>::SharedPtr      srv_get_module_list_; 
      /** Server for get_sdk_info */
      rclcpp::Service<stcamera_msgs::srv::GetSDKInfo>::SharedPtr      srv_get_sdk_info_;
    private:
      rclcpp::TimerBase::SharedPtr timer_;

      StCameraNodeImpl* st_camera_node_impl_;
      friend StCameraNodeImpl;
    protected:

      /** Initialization. This function is called by constructor.
       * In the initialization, list of allowed camera to connect is retrieved
       * and the GenTL modules are initialized. 
       */
      virtual void init();
      void initServices();
      void initPublishers();

  };
}
#endif
