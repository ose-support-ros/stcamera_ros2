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

#ifndef STCAMERA_BASE_HPP
#define STCAMERA_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "stcamera_components/stheader.hpp"
#include "stcamera_components/stcamera_msgs.hpp"

namespace stcamera
{
  class CStCameraBase : public rclcpp::Node
  {
  public:
    CStCameraBase(const std::string & camera_name_space, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    CStCameraBase(const std::string & camera_name_space, const std::string  & name_space, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~CStCameraBase();
    void init();
    protected:
      const std::string camera_name_space_;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
      rclcpp::Subscription<stcamera_msgs::msg::Chunk>::SharedPtr chunk_subscriber_;
      rclcpp::Subscription<stcamera_msgs::msg::Event>::SharedPtr event_subscriber_;
      int sent_triggers_;
      
      rclcpp::TimerBase::SharedPtr trigger_timer_;

    protected:
      template<typename service_t, typename value_t>
      EOSError_t readNodeValue(const std::string &genicam_module, const std::string &genicam_node, value_t &value, const std::string &service_name);
      template<typename service_t, typename value_t>
      EOSError_t writeNodeValue(const std::string &genicam_module, const std::string &genicam_node, value_t &value, const std::string &service_name);
    public:
      //Services
      EOSError_t enableChunk(const std::string &chunk_name, bool value); // EnableChunk.srv
      EOSError_t enableEventAcquisition(const std::string &genicam_module, bool value); // EnableEventAcquisition.srv
      EOSError_t enableEventNode(const std::string &genicam_module, const std::string &genicam_event, const std::string &genicam_node, bool value, const std::string &topic = "");  // EnableEventNode.srv
      EOSError_t enableImageAcquisition(bool value); // EnableImageAcquisition.srv
      EOSError_t enableTrigger(const std::string &trigger_selector, const std::string &trigger_source, bool enabled, double delay = 0);  // EnableTrigger.srv
      EOSError_t executeNode(const std::string &genicam_module, const std::string &genicam_node); // ExecuteNode.srv
      EOSError_t getChunkList(std::vector<std::string> &chunk_name_list, std::vector<bool> &chunk_enabled_list);  // GetChunkList.srv
      EOSError_t getEnumList(const std::string &genicam_module, const std::string &genicam_node, std::vector<std::string> &enum_value_str_list, std::vector<std::string> &enum_node_list, std::vector<int64_t> &enum_value_int_list);  // GetEnumList.srv
      EOSError_t getEventAcquisitionStatusList(std::vector<std::string> &genicam_module_list, std::vector<bool> &enabled_list);  // GetEventAcquisitionStatusList.srv
      EOSError_t getEventNodeStatusList(const std::string &genicam_module, std::vector<stcamera_msgs::msg::GenICamEvent> &event_node_list);  // GetEventNodeStatusList.srv
      EOSError_t getGenICamNodeInfo(const std::string &genicam_module, const std::string &genicam_node, stcamera_msgs::srv::GetGenICamNodeInfo::Response &res);  // GetGenICamNodeInfo.srv
      EOSError_t getImageAcquisitionStatus(bool &value); // GetImageAcquisitionStatus.srv
      EOSError_t getTriggerList(std::vector<std::string> &trigger_selector_list, std::vector<bool> &trigger_mode_list, std::vector<std::string> &trigger_source_list, std::vector<double> &trigger_delayus_list); // GetTriggerList.srv
      EOSError_t readNode(const std::string &genicam_module, const std::string &genicam_node, std::string &value, std::string &interface_type);  // ReadNode.srv
      EOSError_t readNodeBool(const std::string &genicam_module, const std::string &genicam_node, bool &value);  // ReadNodeBool.srv
      EOSError_t readNodeEnum(const std::string &genicam_module, const std::string &genicam_node, std::string &value_str, int64_t &value_int);  // ReadNodeEnum.srv
      EOSError_t readNodeFloat(const std::string &genicam_module, const std::string &genicam_node, double &value);  // ReadNodeFloat.srv
      EOSError_t readNodeInt(const std::string &genicam_module, const std::string &genicam_node, int64_t &value);  // ReadNodeInt.srv
      EOSError_t readNodePort(const std::string &genicam_module, const std::string &genicam_node, int64_t address, int64_t length, std::vector<uint8_t> &data);  // ReadNodePort.srv
      EOSError_t readNodeRegister(const std::string &genicam_module, const std::string &genicam_node, int64_t length, std::vector<uint8_t> &data);  // ReadNodeRegister.srv
      EOSError_t readNodeRegisterInfo(const std::string &genicam_module, const std::string &genicam_node, int64_t &address, int64_t &length);  // ReadNodeRegisterInfo.srv
      EOSError_t readNodeString(const std::string &genicam_module, const std::string &genicam_node, std::string &value);  // ReadNodeString.srv
      EOSError_t sendSoftTrigger(std::shared_ptr<rclcpp::Node> nh, const std::string &trigger_selector);  // SendSoftTrigger.srv
      EOSError_t writeNode(const std::string &genicam_module, const std::string &genicam_node, const std::string &value); // WriteNode.srv
      EOSError_t writeNodeBool(const std::string &genicam_module, const std::string &genicam_node, bool value); // WriteNodeBool.srv
      EOSError_t writeNodeEnumInt(const std::string &genicam_module, const std::string &genicam_node, int64_t value); // WriteNodeEnumInt.srv
      EOSError_t writeNodeEnumStr(const std::string &genicam_module, const std::string &genicam_node, const std::string &value); // WriteNodeEnumStr.srv
      EOSError_t writeNodeFloat(const std::string &genicam_module, const std::string &genicam_node, double value); // WriteNodeFloat.srv
      EOSError_t writeNodeInt(const std::string &genicam_module, const std::string &genicam_node, int64_t value); // WriteNodeInt.srv
      EOSError_t writeNodePort(const std::string &genicam_module, const std::string &genicam_node, int64_t address, int64_t length, const std::vector<uint8_t> &data); // WriteNodePort.srv
      EOSError_t writeNodeRegister(const std::string &genicam_module, const std::string &genicam_node, int64_t length, const std::vector<uint8_t> &data); // WriteNodeRegister.srv
      EOSError_t writeNodeString(const std::string &genicam_module, const std::string &genicam_node, const std::string &value); // WriteNodeString.srv
    public:
      EOSError_t readNodeFloatRange(const std::string &genicam_module, const std::string &genicam_node, double &value_min, double &value_max);
      EOSError_t readNodeIntRange(const std::string &genicam_module, const std::string &genicam_node, int64_t &value_min, int64_t &value_max);
      // Set the status of image acquisition
      EOSError_t setImageAcquisition(bool enabled);

    public:

      // Image callback
      void imageCallback(sensor_msgs::msg::Image::SharedPtr msg)
      {
        onRcvImage(msg);
      }
    
      // Chunk callback
      void chunkCallback(stcamera_msgs::msg::Chunk::SharedPtr msg)
      {
        onRcvChunk(msg);
      }
    
      // Event callback
      void eventCallback(stcamera_msgs::msg::Event::SharedPtr msg)
      {
        onRcvEvent(msg);
      }
    protected:
      std::shared_ptr<rclcpp::Node> GetPtr()
      {
        return(shared_from_this());
      }
    protected:
      virtual void onRcvImage(sensor_msgs::msg::Image::SharedPtr msg) = 0;
      virtual void onRcvChunk(stcamera_msgs::msg::Chunk::SharedPtr msg) = 0;
      virtual void onRcvEvent(stcamera_msgs::msg::Event::SharedPtr msg) = 0;

  };
}
#endif //STCAMERA_BASE_HPP