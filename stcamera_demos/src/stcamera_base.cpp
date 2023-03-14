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
#include "../include/stcamera_base.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace stcamera
{
  CStCameraBase::CStCameraBase(const std::string & camera_name_space, const rclcpp::NodeOptions & options) : 
    CStCameraBase(camera_name_space, "", options)
  {

  }

  CStCameraBase::CStCameraBase(const std::string & camera_name_space, const std::string  & name_space, const rclcpp::NodeOptions & options) : 
    Node("StCamera", name_space, options), camera_name_space_(camera_name_space),
      sent_triggers_(0)
  {
  }
  CStCameraBase::~CStCameraBase()
  {
  }
  void CStCameraBase::init()
  {
      setImageAcquisition(false);
      
      // Subscribe to the connected device
      const std::string ns = camera_name_space_ + "/image_raw";
      rclcpp::QoS video_qos(10);
      //video_qos.best_effort();
      video_qos.reliable();
      video_qos.durability_volatile();
      
      std::string image_topic_name = declare_parameter<std::string>("image_topic_name", ns);
      image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(image_topic_name, video_qos, std::bind(&CStCameraBase::imageCallback, this, _1));

      const std::string chunk_ns = camera_name_space_ + "/chunk";
      chunk_subscriber_ = create_subscription<stcamera_msgs::msg::Chunk>(chunk_ns, video_qos, std::bind(&CStCameraBase::chunkCallback, this, _1));
      
      const std::string event_ns = camera_name_space_ + "/event";
      event_subscriber_ = create_subscription<stcamera_msgs::msg::Event>(event_ns, video_qos, std::bind(&CStCameraBase::eventCallback, this, _1));
  }
  // EnableChunk.srv
  EOSError_t CStCameraBase::enableChunk(const std::string &chunk_name, bool value)
  {
    typedef stcamera_msgs::srv::EnableChunk service_t;
    const std::string ns = camera_name_space_ + "/enable_chunk";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->chunk_name = chunk_name;
    req->value = value;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service enable_chunk");
    }
    return(OSError_UnknownError);
  }
  // EnableEventAcquisition.srv
  EOSError_t CStCameraBase::enableEventAcquisition(const std::string &genicam_module, bool value)
  {
    typedef stcamera_msgs::srv::EnableEventAcquisition service_t;
    const std::string ns = camera_name_space_ + "/enable_event_acquisition";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->value = value;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service enable_event_acquisition");
    }
    return(OSError_UnknownError);
  }
  // EnableEventNode.srv
  EOSError_t CStCameraBase::enableEventNode(const std::string &genicam_module, const std::string &genicam_event, const std::string &genicam_node, bool value, const std::string &topic)
  {
    typedef stcamera_msgs::srv::EnableEventNode service_t;
    const std::string ns = camera_name_space_ + "/enable_event_node";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->event_name = genicam_event;
    req->callback_node = genicam_node;
    req->event_topic_name = topic;
    req->value = value;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service enable_event_node");
    }
    return(OSError_UnknownError);
  }
  // EnableImageAcquisition.srv
  EOSError_t CStCameraBase::enableImageAcquisition(bool value)
  {
    typedef stcamera_msgs::srv::EnableImageAcquisition service_t;
    const std::string ns = camera_name_space_ + "/enable_image_acquisition";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->value = value;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service enable_image_acquisition");
    }
    return(OSError_UnknownError);
  }

  // EnableTrigger.srv
  EOSError_t CStCameraBase::enableTrigger(const std::string &trigger_selector, const std::string &trigger_source, bool enabled, double delay)
  {
    typedef stcamera_msgs::srv::EnableTrigger service_t;
    const std::string ns = camera_name_space_ + "/enable_trigger";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->trigger_selector = trigger_selector;
    req->trigger_source = trigger_source;
    req->trigger_delayus = delay;
    req->value= enabled;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service enable_trigger");
    }
    return(OSError_UnknownError);
  }

  // ExecuteNode.srv
  EOSError_t CStCameraBase::executeNode(const std::string &genicam_module, const std::string &genicam_node)
  {
    typedef stcamera_msgs::srv::ExecuteNode service_t;
    const std::string ns = camera_name_space_ + "/execute_node";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service execute_node");
    }
    return(OSError_UnknownError);
  }


  // GetChunkList.srv
  EOSError_t CStCameraBase::getChunkList(std::vector<std::string> &chunk_name_list, std::vector<bool> &chunk_enabled_list)
  {
    typedef stcamera_msgs::srv::GetChunkList service_t;
    const std::string ns = camera_name_space_ + "/get_chunk_list";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

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
        chunk_name_list = res->chunk_name_list;
        chunk_enabled_list = res->chunk_enabled_list;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_chunk_list");
    }
    return(OSError_UnknownError);
  }
  
  // GetEnumList.srv
  EOSError_t CStCameraBase::getEnumList(const std::string &genicam_module, const std::string &genicam_node, std::vector<std::string> &enum_value_str_list, std::vector<std::string> &enum_node_list, std::vector<int64_t> &enum_value_int_list)
  {

    typedef stcamera_msgs::srv::GetEnumList service_t;
    const std::string ns = camera_name_space_ + "/get_enum_list";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
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
        enum_value_str_list = res->enum_value_str_list;
        enum_node_list = res->enum_node_list;
        enum_value_int_list = res->enum_value_int_list;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_enum_list");
    }
    return(OSError_UnknownError);
  }
  // GetEventAcquisitionStatusList.srv
  EOSError_t CStCameraBase::getEventAcquisitionStatusList(std::vector<std::string> &genicam_module_list, std::vector<bool> &enabled_list)
  {
    typedef stcamera_msgs::srv::GetEventAcquisitionStatusList service_t;
    const std::string ns = camera_name_space_ + "/get_event_acquisition_status_list";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

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
        enabled_list = res->enabled_list;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_event_acquisition_status_list");
    }
    return(OSError_UnknownError);
  }
  
  // GetEventNodeStatusList.srv
  EOSError_t CStCameraBase::getEventNodeStatusList(const std::string &genicam_module, std::vector<stcamera_msgs::msg::GenICamEvent> &event_node_list)
  {
    typedef stcamera_msgs::srv::GetEventNodeStatusList service_t;
    const std::string ns = camera_name_space_ + "/get_event_node_status_list";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
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
        event_node_list = res->event_node_list;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_event_node_status_list");
    }
    return(OSError_UnknownError);
  }

  // GetGenICamNodeInfo.srv
  EOSError_t CStCameraBase::getGenICamNodeInfo(const std::string &genicam_module, const std::string &genicam_node, stcamera_msgs::srv::GetGenICamNodeInfo::Response &res)
  {
    typedef stcamera_msgs::srv::GetGenICamNodeInfo service_t;
    const std::string ns = camera_name_space_ + "/get_genicam_node_info";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
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
      std::shared_ptr<service_t::Response> p_res = future.get();
      res = *p_res;
      const EOSError_t err = (EOSError_t)p_res->error_info.error_code;
      if(err != OSError_Success)
      {
        RCLCPP_ERROR(get_logger(), p_res->error_info.description.c_str());
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_genicam_node_info");
    }
    return(OSError_UnknownError);
  }

  // GetImageAcquisitionStatus.srv
  EOSError_t CStCameraBase::getImageAcquisitionStatus(bool &value)
  {
    typedef stcamera_msgs::srv::GetImageAcquisitionStatus service_t;
    const std::string ns = camera_name_space_ + "/get_image_acquisition_status";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

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
        value = res->value;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_image_acquisition_status");
    }
    return(OSError_UnknownError);
  }

  // GetTriggerList.srv
  EOSError_t CStCameraBase::getTriggerList(std::vector<std::string> &trigger_selector_list, std::vector<bool> &trigger_mode_list, std::vector<std::string> &trigger_source_list, std::vector<double> &trigger_delayus_list)
  {
    typedef stcamera_msgs::srv::GetTriggerList service_t;
    const std::string ns = camera_name_space_ + "/get_trigger_list";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

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
        trigger_selector_list = res->trigger_selector_list;
        trigger_mode_list = res->trigger_mode_list;
        trigger_source_list = res->trigger_source_list;
        trigger_delayus_list = res->trigger_delayus_list;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service get_trigger_list");
    }
    return(OSError_UnknownError);
  }
  // ReadNode.srv
  EOSError_t CStCameraBase::readNode(const std::string &genicam_module, const std::string &genicam_node, std::string &value, std::string &interface_type)
  {
    typedef stcamera_msgs::srv::ReadNode service_t;
    const std::string ns = camera_name_space_ + "/read_node";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
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
        value = res->value;
        interface_type = res->interface_type;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service read_node");
    }
    return(OSError_UnknownError);
  }


  template<typename service_t, typename value_t>
  EOSError_t CStCameraBase::readNodeValue(const std::string &genicam_module, const std::string &genicam_node, value_t &value, const std::string &service_name)
  {
    typedef typename rclcpp::Client<service_t>::SharedPtr client_ptr_t;
    typedef typename service_t::Request request_t;
    typedef typename service_t::Response response_t;
    const std::string ns = camera_name_space_ + "/" + service_name;
    client_ptr_t client = create_client<service_t>(ns);

    auto req = std::make_shared<request_t>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
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
      std::shared_ptr<response_t> res = future.get();
      const EOSError_t err = (EOSError_t)res->error_info.error_code;
      if(err != OSError_Success)
      {
        RCLCPP_ERROR(get_logger(), res->error_info.description.c_str());
      }
      else
      {
        value = res->value;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service %s", service_name.c_str());
    }
    return(OSError_UnknownError);
  }
  // ReadNodeBool.srv
  EOSError_t CStCameraBase::readNodeBool(const std::string &genicam_module, const std::string &genicam_node, bool &value)
  {
    return(readNodeValue<stcamera_msgs::srv::ReadNodeBool, bool>(genicam_module, genicam_node, value, "read_node_bool"));
  }

  // ReadNodeEnum.srv
  EOSError_t CStCameraBase::readNodeEnum(const std::string &genicam_module, const std::string &genicam_node, std::string &value_str, int64_t &value_int)
  {
    typedef stcamera_msgs::srv::ReadNodeEnum service_t;
    const std::string ns = camera_name_space_ + "/read_node_enum";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
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
        value_str = res->value_str;
        value_int = res->value_int;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service read_node_enum");
    }
    return(OSError_UnknownError);
  }
  //ReadNodeFloat.srv
  EOSError_t CStCameraBase::readNodeFloat(const std::string &genicam_module, const std::string &genicam_node, double &value)
  {
    return(readNodeValue<stcamera_msgs::srv::ReadNodeFloat, double>(genicam_module, genicam_node, value, "read_node_float"));
  }
  // ReadNodeInt.srv
  EOSError_t CStCameraBase::readNodeInt(const std::string &genicam_module, const std::string &genicam_node, int64_t &value)
  {
    return(readNodeValue<stcamera_msgs::srv::ReadNodeInt, int64_t>(genicam_module, genicam_node, value, "read_node_int"));
  }
  // ReadNodePort.srv
  EOSError_t CStCameraBase::readNodePort(const std::string &genicam_module, const std::string &genicam_node, int64_t address, int64_t length, std::vector<uint8_t> &data)
  {
    typedef stcamera_msgs::srv::ReadNodePort service_t;
    const std::string ns = camera_name_space_ + "/read_node_port";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
    req->address = address;
    req->length = length;
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
        data = res->data;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service read_node_port");
    }
    return(OSError_UnknownError);
  }
  // ReadNodeRegister.srv
  EOSError_t CStCameraBase::readNodeRegister(const std::string &genicam_module, const std::string &genicam_node, int64_t length, std::vector<uint8_t> &data)
  {
    typedef stcamera_msgs::srv::ReadNodeRegister service_t;
    const std::string ns = camera_name_space_ + "/read_node_register";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
    req->length = length;
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
        data = res->data;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service read_node_port");
    }
    return(OSError_UnknownError);
  }
  // ReadNodeRegisterInfo.srv
  EOSError_t CStCameraBase::readNodeRegisterInfo(const std::string &genicam_module, const std::string &genicam_node, int64_t &address, int64_t &length)
  {
    typedef stcamera_msgs::srv::ReadNodeRegisterInfo service_t;
    const std::string ns = camera_name_space_ + "/read_node_register_info";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
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
        address = res->address;
        length = res->length;
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service read_node_port");
    }
    return(OSError_UnknownError);
  }
  // ReadNodeString.srv
  EOSError_t CStCameraBase::readNodeString(const std::string &genicam_module, const std::string &genicam_node, std::string &value)
  {
    return(readNodeValue<stcamera_msgs::srv::ReadNodeString, std::string>(genicam_module, genicam_node, value, "read_node_string"));
  }

  // SendSoftTrigger.srv
  EOSError_t CStCameraBase::sendSoftTrigger(std::shared_ptr<rclcpp::Node> nh, const std::string &trigger_selector)
  { 
    typedef stcamera_msgs::srv::SendSoftTrigger service_t;
    const std::string ns = camera_name_space_ + "/send_soft_trigger";
    rclcpp::Client<service_t>::SharedPtr client = nh->create_client<service_t>(ns, rmw_qos_profile_services_default);

    auto req = std::make_shared<service_t::Request>();
    req->trigger_selector = trigger_selector;
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
    if (rclcpp::spin_until_future_complete(nh, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      std::shared_ptr<service_t::Response> res = future.get();
      const EOSError_t err = (EOSError_t)res->error_info.error_code;
      if(err != OSError_Success)
      {
        RCLCPP_ERROR(get_logger(), res->error_info.description.c_str());
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service send_soft_trigger");
    }
    return(OSError_UnknownError);
  }

  // WriteNode.srv
  EOSError_t CStCameraBase::writeNode(const std::string &genicam_module, const std::string &genicam_node, const std::string &value)
  {
    typedef stcamera_msgs::srv::WriteNode service_t;
    const std::string ns = camera_name_space_ + "/write_node";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
    req->value = value;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service write_node");
    }
    return(OSError_UnknownError);
  }

  template<typename service_t, typename value_t>
  EOSError_t CStCameraBase::writeNodeValue(const std::string &genicam_module, const std::string &genicam_node, value_t &value, const std::string &service_name)
  {
    typedef typename rclcpp::Client<service_t>::SharedPtr client_ptr_t;
    typedef typename service_t::Request request_t;
    typedef typename service_t::Response response_t;
    const std::string ns = camera_name_space_ + "/" + service_name;
    client_ptr_t client = create_client<service_t>(ns);

    auto req = std::make_shared<request_t>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
    req->value = value;
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
      std::shared_ptr<response_t> res = future.get();
      const EOSError_t err = (EOSError_t)res->error_info.error_code;
      if(err != OSError_Success)
      {
        RCLCPP_ERROR(get_logger(), res->error_info.description.c_str());
      }
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service %s", service_name.c_str());
    }
    return(OSError_UnknownError);
  }

  // WriteNodeBool.srv
  EOSError_t CStCameraBase::writeNodeBool(const std::string &genicam_module, const std::string &genicam_node, bool value)
  {
    return(writeNodeValue<stcamera_msgs::srv::WriteNodeBool, bool>(genicam_module, genicam_node, value, "write_node_bool"));
  }
  // WriteNodeEnumInt.srv
  EOSError_t CStCameraBase::writeNodeEnumInt(const std::string &genicam_module, const std::string &genicam_node, int64_t value)
  {
    return(writeNodeValue<stcamera_msgs::srv::WriteNodeEnumInt, int64_t>(genicam_module, genicam_node, value, "write_node_enum_int"));
  }

  // WriteNodeEnumStr.srv
  EOSError_t CStCameraBase::writeNodeEnumStr(const std::string &genicam_module, const std::string &genicam_node, const std::string &value)
  {
    return(writeNodeValue<stcamera_msgs::srv::WriteNodeEnumStr, const std::string &>(genicam_module, genicam_node, value, "write_node_enum_str"));
  }

  // WriteNodeFloat.srv
  EOSError_t CStCameraBase::writeNodeFloat(const std::string &genicam_module, const std::string &genicam_node, double value)
  {
    return(writeNodeValue<stcamera_msgs::srv::WriteNodeFloat, double>(genicam_module, genicam_node, value, "write_node_float"));
  }
  //  WriteNodeInt.srv
  EOSError_t CStCameraBase::writeNodeInt(const std::string &genicam_module, const std::string &genicam_node, int64_t value)
  {
    return(writeNodeValue<stcamera_msgs::srv::WriteNodeInt, int64_t>(genicam_module, genicam_node, value, "write_node_int"));
  }
  // WriteNodePort.srv
  EOSError_t CStCameraBase::writeNodePort(const std::string &genicam_module, const std::string &genicam_node, int64_t address, int64_t length, const std::vector<uint8_t> &data)
  {
    typedef stcamera_msgs::srv::WriteNodePort service_t;
    const std::string ns = camera_name_space_ + "/write_node_port";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
    req->address = address;
    req->length = length;
    req->data = data;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service write_node_port");
    }
    return(OSError_UnknownError);
  }
  
  // WriteNodeRegister.srv
  EOSError_t CStCameraBase::writeNodeRegister(const std::string &genicam_module, const std::string &genicam_node, int64_t length, const std::vector<uint8_t> &data)
  {
    typedef stcamera_msgs::srv::WriteNodeRegister service_t;
    const std::string ns = camera_name_space_ + "/write_node_register";
    rclcpp::Client<service_t>::SharedPtr client = create_client<service_t>(ns);

    auto req = std::make_shared<service_t::Request>();
    req->genicam_module = genicam_module;
    req->genicam_node = genicam_node;
    req->length = length;
    req->data = data;
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
      return(err);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service write_node_register");
    }
    return(OSError_UnknownError);
  }

  // WriteNodeString.srv
  EOSError_t CStCameraBase::writeNodeString(const std::string &genicam_module, const std::string &genicam_node, const std::string &value)
  {
    return(writeNodeValue<stcamera_msgs::srv::WriteNodeString, const std::string &>(genicam_module, genicam_node, value, "write_node_string"));
  }

  EOSError_t CStCameraBase::readNodeFloatRange(const std::string &genicam_module, const std::string &genicam_node, double &value_min, double &value_max)
  {
    stcamera_msgs::srv::GetGenICamNodeInfo::Response res;
    EOSError_t eError = getGenICamNodeInfo(genicam_module, genicam_node, res);
    if(eError == OSError_Success)
    {
      value_min = std::stod(res.min_value);
      value_max = std::stod(res.max_value);
    }
    return(eError);
  }

  EOSError_t CStCameraBase::readNodeIntRange(const std::string &genicam_module, const std::string &genicam_node, int64_t &value_min, int64_t &value_max)
  {
    stcamera_msgs::srv::GetGenICamNodeInfo::Response res;
    EOSError_t eError = getGenICamNodeInfo(genicam_module, genicam_node, res);
    if(eError == OSError_Success)
    {
      value_min = std::stoll(res.min_value);
      value_max = std::stoll(res.max_value);
    }
    return(eError);
  }

  // Set the status of image acquisition
  EOSError_t CStCameraBase::setImageAcquisition(bool enabled)
  {
    bool status;
    EOSError_t eError = getImageAcquisitionStatus(status);
    if(eError != OSError_Success) return(eError);
    if (status ^ enabled)
    {
      eError = enableImageAcquisition(enabled);
    }
    return(eError);
  }

}
