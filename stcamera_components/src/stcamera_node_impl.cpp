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
#include "impl/stcamera_interface_impl.hpp"
#include "stcamera_msgs/msg/device_connection.hpp"

#include "stheader.hpp"

namespace stcamera
{
  using namespace std;
  using namespace std::placeholders;

  /** Helper function to fill the structure of DeviceConnection.
   * \param[in] p_tlinfo: Pointer to the IStSystemInfo of the device.
   * \param[in] p_iface: Pointer to the IStInterface of the device.
   * \param[in] p_devinfo: Pointer to the IStDeviceInfo of the device.
   * \param[in] device_namespace: Namespace of the device. 
   * \param[in] connected: whether the device is connected or not.
   * \return  Filled DeviceConnection.
   */
   void fillDeviceConnectionData(
          stcamera_msgs::msg::DeviceConnection &msg, rclcpp::Clock &clock, 
          const StApi::IStSystemInfo *p_tlinfo, StApi::IStInterface* p_iface,
          const StApi::IStDeviceInfo *p_devinfo, std::string device_namespace = "",
          bool connected = false)
  {
    msg.timestamp = clock.now();
    msg.device_id = p_devinfo->GetID().c_str();
    msg.device_model = p_devinfo->GetModel().c_str();
    msg.device_serial = p_devinfo->GetSerialNumber().c_str();
    msg.device_namespace = device_namespace;
    msg.device_tltype = p_devinfo->GetTLType().c_str();

    if (msg.device_tltype.compare(TLTypeGEVName) == 0)
    {
      try
      {
        GenApi::CNodeMapPtr p_nodemap(p_iface->GetIStPort()->GetINodeMap());
        GenApi::CIntegerPtr p_integer_ip(
            p_nodemap->GetNode("GevInterfaceSubnetIPAddress"));
        GenApi::CIntegerPtr p_integer_subnet(
            p_nodemap->GetNode("GevInterfaceSubnetMask"));
        GenApi::CIntegerPtr p_integer_gw(
            p_nodemap->GetNode("GevInterfaceGateway"));
        msg.device_tl_specific_field.push_back("if_ip_address");
        msg.device_tl_specific_value.push_back(
            p_integer_ip->ToString().c_str());
        msg.device_tl_specific_field.push_back("if_ip_mask");
        msg.device_tl_specific_value.
            push_back(p_integer_subnet->ToString().c_str());
        msg.device_tl_specific_field.push_back("if_ip_gateway");
        msg.device_tl_specific_value.push_back(
            p_integer_gw->ToString().c_str());

        GenApi::CIntegerPtr pDeviceSelector(
            p_nodemap->GetNode("DeviceSelector"));
        const int64_t nMaxIndex = pDeviceSelector->GetMax();
        for (int64_t k = 0; k <= nMaxIndex; k++)
        {
          pDeviceSelector->SetValue(k);
          GenApi::CStringPtr pIStringDeviceID(p_nodemap->GetNode("DeviceID"));
          GenICam::gcstring strDeviceID = pIStringDeviceID->GetValue();
          
          if (strDeviceID.compare(msg.device_id.c_str()) != 0)
          {
            continue;
          }
          GenApi::CIntegerPtr p_integer_ip(
              p_nodemap->GetNode("GevDeviceIPAddress"));
          GenApi::CIntegerPtr p_integer_subnet(
              p_nodemap->GetNode("GevDeviceSubnetMask"));
          GenApi::CIntegerPtr p_integer_gw(
              p_nodemap->GetNode("GevDeviceGateway"));
          msg.device_tl_specific_field.push_back("dev_ip_address");
          msg.device_tl_specific_value.
              push_back(p_integer_ip->ToString().c_str());
          msg.device_tl_specific_field.push_back("dev_ip_mask");
          msg.device_tl_specific_value.
              push_back(p_integer_subnet->ToString().c_str());
          msg.device_tl_specific_field.push_back("dev_ip_gateway");
          msg.device_tl_specific_value.
              push_back(p_integer_gw->ToString().c_str());
          break; 
        }
      }
      catch(...)
      {
      }
    }
    msg.connected = connected;

    // gentl info
    char gentl_ver[8];
    snprintf(gentl_ver,8,"%d.%d", p_tlinfo->GetGenTLVersionMajor(),
        p_tlinfo->GetGenTLVersionMinor());
    msg.device_gentl_info.vendor = p_tlinfo->GetVendor();
    msg.device_gentl_info.version = gentl_ver;
    msg.device_gentl_info.producer_version = p_tlinfo->GetVersion();
    msg.device_gentl_info.tltype = p_tlinfo->GetTLType();
    msg.device_gentl_info.full_path = p_tlinfo->GetPathName();
  }

  StCameraNodeImpl::StCameraNodeImpl(StCameraNode *p_stcamera_node) : 
    clock_(TIME_SOURCE),
    p_stcamera_node_(p_stcamera_node),
    stcamera_node_text_("StCameraNode")
  {

  }
  StCameraNodeImpl::~StCameraNodeImpl()
  {

    std::lock_guard<std::mutex> lock(mtx_map_camera_);

		for (MapCameraInterface::iterator it = map_camera_.begin();
				it != map_camera_.end();)
		{
			if (it->second != nullptr) 
      {
        delete it->second;
      }
      it = map_camera_.erase(it);
		}

  }

  void StCameraNodeImpl::initSystemsAndInterfaces()
  {
    std::vector<std::string> camera_to_connect;
    p_stcamera_node_->param_.loadCameraList(p_stcamera_node_, camera_to_connect);

    std::string allowed="Allowed camera: ";
    if (p_stcamera_node_->param_.connectAllCamera()) 
    {
      allowed += "all";
    }
    else if (p_stcamera_node_->param_.connectFirstCameraOnly()) 
    {
      allowed += "First found camera";
    }
    else 
    {
      std::lock_guard<std::mutex> lock(mtx_map_camera_);
      
      // connect to certain camera only
      for (size_t i = 0; i < camera_to_connect.size(); i++)
      {
        // set interface to null for the allowed camera:
        map_camera_[p_stcamera_node_->param_.getNamespace(camera_to_connect[i])] = nullptr;
        allowed += camera_to_connect[i] + " ";
      }
    }
    RCLCPP_DEBUG(p_stcamera_node_->get_logger(), "Allowed to connect: %s", allowed.c_str());

    for (uint32_t i = StApi::StSystemVendor_Sentech; 
        i < StApi::StSystemVendor_Count; i++)
    {
      StApi::EStSystemVendor_t vendor = (StApi::EStSystemVendor_t)i;
      try
      {
        stapi_systems_.Register(StApi::CreateIStSystem(vendor, StApi::StInterfaceType_All));
      }
      catch(const GenICam::GenericException &x)
      {
        if (vendor == StApi::StSystemVendor_Sentech)
        {
          RCLCPP_ERROR(p_stcamera_node_->get_logger(), "%s %s %d: Unable to initialize OMRON SENTECH GenTL Producer: %s",
              __FILE__,__func__,__LINE__, x.GetDescription());
        }
      }
    }

    for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
    {
      StApi::IStSystem *p_tl = stapi_systems_[i];
      p_tl->UpdateInterfaceList();
    }

  }
  bool StCameraNodeImpl::initializeCamera(StApi::IStInterface *p_iface,
      const StApi::IStDeviceInfo *p_devinfo)
  {
    if (p_iface == nullptr || p_devinfo == nullptr)
    {
      return false;
    }
    RCLCPP_DEBUG(p_stcamera_node_->get_logger(), "StCameraNodeImpl::initializeCamera");

    std::string device_id = std::string(p_devinfo->GetID());
    RCLCPP_DEBUG(p_stcamera_node_->get_logger(), "  Device ID:%s", device_id.c_str());
    std::string device_displayname = std::string(p_devinfo->GetDisplayName());
    RCLCPP_DEBUG(p_stcamera_node_->get_logger(), "  Display Name:%s", device_displayname.c_str());
    std::string device_userdefinedname = std::string(p_devinfo->GetUserDefinedName() );
    RCLCPP_DEBUG(p_stcamera_node_->get_logger(), "  User Defined Name:%s", device_userdefinedname.c_str());
    std::string key = "";
    if (p_stcamera_node_->param_.connectAllCamera() || p_stcamera_node_->param_.connectFirstCameraOnly())
    {
      key = p_stcamera_node_->param_.getNamespace(device_id);
    }
    else
    {
      bool found = false;
      for (MapCameraInterface::iterator it = map_camera_.begin(); 
          it != map_camera_.end(); it++)
      {
        RCLCPP_DEBUG(p_stcamera_node_->get_logger(), "  map_camera_[]:%s", it->first.c_str());
        key = p_stcamera_node_->param_.getNamespace(device_id);
        if (it->first.compare(key) == 0) 
        {
          found = true;
          break;
        }
        key = p_stcamera_node_->param_.getNamespace(device_displayname);
        if (it->first.compare(key) == 0) 
        {
          found = true;
          break;
        }
        key = p_stcamera_node_->param_.getNamespace(device_userdefinedname);
        if (it->first.compare(key) == 0) 
        {
          found = true;
          break;
        }
      }
      if (!found) key = "";
    }

    if (key.empty()) 
    {
      return false;
    }

    try
    {
      StApi::IStDeviceReleasable *dev = p_iface->CreateIStDevice(
          GenICam::gcstring(device_id.c_str()),
          GenTL::DEVICE_ACCESS_CONTROL);

      StCameraInterface *pcif = new StCameraInterfaceImpl(
          dev, p_stcamera_node_, key, &p_stcamera_node_->param_, clock_);
      map_camera_[key] = pcif;

      // store and publish connection msg
      stcamera_msgs::msg::DeviceConnection msg;
      fillDeviceConnectionData(msg, clock_, 
        dev->GetIStInterface()->GetIStSystem()->GetIStSystemInfo(), dev->GetIStInterface(),
        dev->GetIStDeviceInfo(), key, true);

      map_connection_[key] = msg;

      p_stcamera_node_->msg_device_connection_->publish(msg);
   
      RCLCPP_INFO(p_stcamera_node_->get_logger(),"%s %s %d: %s (%s) is successfully initialized as %s",
          __FILE__,__func__,__LINE__,
          device_id.c_str(), device_displayname.c_str(), key.c_str());
      return true;
    }
    catch(GenICam::GenericException &x)
    {
      RCLCPP_ERROR(p_stcamera_node_->get_logger(), "%s %s %d: Unable to open device %s (%s): %s",
          __FILE__,__func__,__LINE__, device_id.c_str(), 
          device_displayname.c_str(), x.GetDescription());
    }
    return false;
  }
  void StCameraNodeImpl::DetectingAndOpenningDevices()
  {
    RCLCPP_DEBUG(p_stcamera_node_->get_logger(),"StCameraNode::DetectingAndOpenningDevices Start");
    size_t count = 0;
    std::lock_guard<std::mutex> lock(mtx_map_camera_);
    for (MapCameraInterface::iterator it = map_camera_.begin(); 
        it != map_camera_.end(); it++)
    {
      StCameraInterface *c = it->second;
      if (c != nullptr)
      {
        // check device availability
        if (c->deviceIsLost())
        {
          MapDeviceConnection::iterator it2 = map_connection_.find(it->first);
          if (it2 != map_connection_.end())
          {
            it2->second.connected = false;
            it2->second.timestamp = clock_.now();
            p_stcamera_node_->msg_device_connection_->publish(it2->second);
          }
          map_camera_[it->first] = nullptr;
          RCLCPP_INFO(p_stcamera_node_->get_logger(),"Device %s is disconnected.", it2->first.c_str());
          delete c;
        }
        else
        {
          count++;
        }
      }
    }

    bool connect_first_camera_only = p_stcamera_node_->param_.connectFirstCameraOnly();
    if (connect_first_camera_only)
    {
      if (count > 0)
      {
        return;
      }
    }
    else if (!p_stcamera_node_->param_.connectAllCamera())
    {
      bool all_connected = true;
      for (MapCameraInterface::iterator it = map_camera_.begin(); 
          it != map_camera_.end(); it++)
      {
        if (it->second == nullptr)
        {
          all_connected = false;
          break;
        }
      }
      if (all_connected) 
      {
        return;
      }
    }
    //else RCLCPP_INFO(p_stcamera_node_->get_logger(), "Connect to any devices");
    
    // search for camera
    try
    {
      std::lock_guard<std::mutex> lock(mtx_update_device_list_);
      for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
      {
        StApi::IStSystem *p_tl = stapi_systems_[i];
        size_t ifcount = p_tl->GetInterfaceCount();  
        for (size_t j = 0; j < ifcount; j++)
        {
          StApi::IStInterface *p_iface = p_tl->GetIStInterface(j);
          p_iface->UpdateDeviceList();  
  
          size_t devcount = p_iface->GetDeviceCount();
          for (size_t k = 0; k < devcount; k++)
          {
            if (p_iface->IsDeviceAvailable(k, GenTL::DEVICE_ACCESS_CONTROL))
            {
              const StApi::IStDeviceInfo *p_devinfo = 
                  p_iface->GetIStDeviceInfo(k);
              bool init_ok = initializeCamera(p_iface, p_devinfo);
              if (init_ok && connect_first_camera_only) 
              {
                return;
              }
              //RCLCPP_INFO(p_stcamera_node_->get_logger(),"connected %d", init_ok);
            }
          } // end loop device
        } // end loop interface
      } // end loop system
    }
    catch(...)
    {
    }
    RCLCPP_DEBUG(p_stcamera_node_->get_logger(),"StCameraNode::DetectingAndOpenningDevices End");
  }

  bool StCameraNodeImpl::getModuleListCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetModuleList_Request> /*req*/,
      std::shared_ptr<stcamera_msgs::srv::GetModuleList_Response> res)
  {
    res->genicam_module_list.push_back("System");
    res->genicam_module_list.push_back("Interface");
    res->genicam_module_list.push_back("LocalDevice");
    res->genicam_module_list.push_back("RemoteDevice");
    res->genicam_module_list.push_back("DataStream");
    //"StreamBuffer" is not included.
    RETURN_SUCCESS(res);
  }

  bool StCameraNodeImpl::getDeviceListCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetDeviceList_Request> /*req*/,
      std::shared_ptr<stcamera_msgs::srv::GetDeviceList_Response> res)
  {
    std::lock_guard<std::mutex> lock(mtx_update_device_list_);
    try
    {
      for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
      {
        StApi::IStSystem *p_tl = stapi_systems_[i];
        size_t ifcount = p_tl->GetInterfaceCount(); 
        for (size_t j = 0; j < ifcount; j++)
        {
          StApi::IStInterface *p_iface = p_tl->GetIStInterface(j);
          p_iface->UpdateDeviceList();  
          size_t devcount = p_iface->GetDeviceCount(); 
          for (size_t k = 0; k < devcount; k++)
          {
            const StApi::IStDeviceInfo *p_devinfo = 
                p_iface->GetIStDeviceInfo(k);
            std::string device_id = std::string(p_devinfo->GetID());
            std::string device_sn = std::string(p_devinfo->GetDisplayName());
            std::string key_id = p_stcamera_node_->param_.getNamespace(device_id);
            std::string key_sn = p_stcamera_node_->param_.getNamespace(device_sn);
            MapDeviceConnection::iterator it = map_connection_.find(key_id);
            if (it == map_connection_.end())
            {
              it = map_connection_.find(key_sn);
            }
            if (it != map_connection_.end())
            {
              res->device_list.push_back(it->second);
              continue;
            }
            stcamera_msgs::msg::DeviceConnection msg;
            fillDeviceConnectionData(msg, clock_, p_tl->GetIStSystemInfo(), p_iface,
               p_devinfo);
            res->device_list.push_back(msg);
          } // end loop device
        } // end loop interface
      } // end loop system
    }
    catch(...)
    {
    }
    RETURN_SUCCESS(res);
  }

  bool StCameraNodeImpl::getSDKInfoCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetSDKInfo_Request> /*req*/,
      std::shared_ptr<stcamera_msgs::srv::GetSDKInfo_Response> res)
  {
    res->sdk_version = StApi::GetStApiVersionText();

    for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
    {
      stcamera_msgs::msg::GenTLInfo data;
      const StApi::IStSystemInfo *info = stapi_systems_[i]->GetIStSystemInfo();
      char gentl_ver[8];
      snprintf(gentl_ver,8,"%d.%d", info->GetGenTLVersionMajor(),
          info->GetGenTLVersionMinor());
      data.vendor = info->GetVendor();
      data.version = gentl_ver;
      data.producer_version = info->GetVersion();
      data.tltype = info->GetTLType();
      data.full_path = info->GetPathName();
      res->gentl_info_list.push_back(data);
    }
    RETURN_SUCCESS(res);
  }
} // end of namespace stcamera
