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

#include "impl/stcamera_interface_impl.hpp"

#include "stcamera_msgs/msg/element_bool.hpp"
#include "stcamera_msgs/msg/element_float64.hpp"
#include "stcamera_msgs/msg/element_int64.hpp"
#include "stcamera_msgs/msg/element_string.hpp"
#include "stcamera_msgs/msg/gen_i_cam_event.hpp"

namespace stcamera
{
  using namespace StApi;
  using namespace std::placeholders;

  StCameraInterfaceImpl::StCameraInterfaceImpl(
      StApi::IStDeviceReleasable *dev,
      rclcpp::Node *parent_nh, 
      const std::string &camera_namespace, 
      StParameter *param,
      rclcpp::Clock &clock,
      uint32_t queue_size):
    StCameraInterface(parent_nh, camera_namespace, param, clock),
    tl_dev_(dev),
    camera_info_manager_(nh_.get(), camera_namespace),
    queue_size_(queue_size),
    bool_event_system_(false),
    bool_event_interface_(false),
    bool_event_device_(false),
    bool_event_datastream_(false),
    destroyed_(false)
#if ENABLED_STAPI_ZERO_COPY
    , image_allocator_(camera_namespace)
    , p_stimage_buffer_(CreateIStImageBuffer(&image_allocator_))
    , p_stpixel_format_converter_(CreateIStConverter(StConverterType_PixelFormat))
#endif //ENABLED_STAPI_ZERO_COPY
  {
    RCLCPP_INFO(get_logger(),"StCameraInterface::StCameraInterface Openning device %s", camera_namespace_.c_str());
    initServices();
    initPublishers();

    std::lock_guard<std::mutex> lock1(mtx_acquisition_);
    std::lock_guard<std::mutex> lock2(mtx_event_);

    // initialize default_event
    std::string default_event = STMSG_event;
    def_event_ = nh_->create_publisher<stcamera_msgs::msg::Event>(default_event, queue_size_);
    map_msg_event_.insert(std::make_pair(default_event, def_event_));

    initDeviceAndDataStream();

    // register event callback for GenTL modules Interface, and System
    ist_registered_callback_interface_ = StApi::RegisterCallback(tl_dev_->GetIStInterface(), *this,
        &StCameraInterfaceImpl::eventInterfaceCB,
        (void *)NULL);
    ist_registered_callback_system_ = StApi::RegisterCallback(tl_dev_->GetIStInterface()->GetIStSystem(), *this,
        &StCameraInterfaceImpl::eventSystemCB,
        (void *)NULL);

    // register device lost
    registerDeviceLostEvent();

    // check if chunk is enabled. 
    activateChunks();
   
    initializeCameraInfo();

    // start acquisition
    bool_acquisition_is_started_ = true;
    tl_ds_->StartAcquisition();
    tl_dev_->AcquisitionStart(); 
  }

  StCameraInterfaceImpl::~StCameraInterfaceImpl()
  {
    RCLCPP_INFO(get_logger(),"StCameraInterface::~StCameraInterface Closing device %s", camera_namespace_.c_str());
    destroyed_ = true;

    tl_dev_->StopEventAcquisitionThread();

    std::lock_guard<std::mutex> lock(mtx_event_);
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);

    if (!tl_dev_->IsDeviceLost())
    {
      if (bool_acquisition_is_started_)
      {
        tl_dev_->AcquisitionStop(); 
        tl_ds_->StopAcquisition();
        bool_acquisition_is_started_ = false;
      }
    }
    // Deregister all node callback
    MapCallback *cblist[] = 
    {
      &map_event_system_, 
      &map_event_interface_, 
      &map_event_localdevice_,
      &map_event_remotedevice_, 
      &map_event_datastream_
    }; 
    for (int i = 0; i < (int)(sizeof(cblist) / sizeof(cblist[0])); i++)
    {
      MapCallback *cb = cblist[i];
      for (MapCallback::iterator it = cb->begin(); it != cb->end(); it++)
      {
        StApi::IStRegisteredCallbackReleasable *cbf = it->second.cb_;
        cbf->Release();
      }
      cb->clear();
    }

    map_msg_event_.clear();

    ist_registered_callback_datastream_.Reset(nullptr);
    ist_registered_callback_device_.Reset(nullptr);
    ist_registered_callback_interface_.Reset(nullptr);
    ist_registered_callback_system_.Reset(nullptr);

    tl_ds_.Reset(nullptr);
    tl_dev_.Reset(nullptr);
    RCLCPP_INFO(get_logger(),"Closing device %s done.", camera_namespace_.c_str());
  }

  void StCameraInterfaceImpl::initDeviceAndDataStream()
  {
    // initialize Data stream and register acquisition callback
    tl_ds_.Reset(tl_dev_->CreateIStDataStream(0, &image_allocator_));
    ist_registered_callback_datastream_ = StApi::RegisterCallback(tl_ds_, *this, 
        &StCameraInterfaceImpl::eventDataStreamCB, (void *)NULL);

    // register event callback for GenTL modules Device, Interface, and System
    ist_registered_callback_device_ = StApi::RegisterCallback(tl_dev_, *this,
        &StCameraInterfaceImpl::eventDeviceCB, (void *)NULL);
  }
  void StCameraInterfaceImpl::registerDeviceLostEvent()
  {
    if (tl_dev_->GetIStInterface()->GetIStSystem()->GetStSystemVendor() == StApi::StSystemVendor_Sentech)
    {
      try
      {
        StCallback_t stc;
        stc.topic_name_ = STMSG_event;
        stc.event_name_ = "DeviceLost";
        std::string callback_node = "EventDeviceLost";
        GenApi::CNodeMapPtr p(tl_dev_->GetLocalIStPort()->GetINodeMap());
        GenApi::CNodePtr node_callback(p->GetNode(GenICam::gcstring(callback_node.c_str())));
        stc.cb_ = StApi::RegisterCallback(node_callback, *this,
            &StCameraInterfaceImpl::eventGenApiNodeCB,
            (void*)(&map_event_localdevice_), GenApi::cbPostInsideLock);
        GenApi::CEnumerationPtr p_event_selector(p->GetNode(GenICam::gcstring("EventSelector")));
        *p_event_selector = stc.event_name_.c_str();
        GenApi::CEnumerationPtr p_event_notif(p->GetNode(GenICam::gcstring("EventNotification")));
        *p_event_notif = "On";
        tl_dev_->StartEventAcquisitionThread();
        map_event_localdevice_[callback_node] = stc;
        bool_event_device_ = true; 
      }
      catch(GenICam::GenericException &x)
      {
        RCLCPP_ERROR(get_logger(), "%s %s %d: \n\t%s GenICam error when checking chunk: %s",
            __FILE__,__func__,__LINE__, 
            camera_namespace_.c_str(), x.GetDescription());
      }
    }
  }
  bool StCameraInterfaceImpl::deviceIsLost()
  {
    return (tl_dev_->IsDeviceLost());
  }

  void StCameraInterfaceImpl::eventSystemCB(StApi::IStCallbackParamBase *p, void * /*pvContext*/)
  {
    if (destroyed_) return;
    stcamera_msgs::msg::Event msg;
    msg.timestamp = clock_.now();
    msg.genicam_module = "System";
    if (p->GetCallbackType() == StApi::StCallbackType_GenTLEvent_SystemError)
    {
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
    }
    publishEventDefault(msg); 
  }
  void StCameraInterfaceImpl::activateChunks()
  {
    std::lock_guard<std::mutex> lock3(mtx_chunk_);
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();
      GenApi::INode *node_sel = mp->GetNode( GenICam::gcstring("ChunkSelector"));
      GenApi::INode *node_enable = mp->GetNode(GenICam::gcstring("ChunkEnable"));
      GenApi::INode *node_chunk_active = mp->GetNode(
          GenICam::gcstring("ChunkModeActive"));
      if (node_sel && node_enable && node_chunk_active)
      {
        GenApi::CBooleanPtr chunk_active(node_chunk_active);
        GenApi::CEnumerationPtr chunk_selector(node_sel);
        GenApi::NodeList_t nodelist;
        bool revert_chunk_mode = (false == chunk_active->GetValue());
        if (revert_chunk_mode) 
        {
          chunk_active->SetValue(true);
        }
        chunk_selector->GetEntries(nodelist);
        for (GenApi::NodeList_t::iterator it = nodelist.begin();
            it != nodelist.end(); it++)
        {
          if (GenApi::IsAvailable(*it))
          {
            GenApi::CEnumEntryPtr enum_entry(*it);
            std::string chunk_name = enum_entry->GetSymbolic().c_str();
            chunk_selector->SetIntValue(enum_entry->GetValue());
            GenApi::CBooleanPtr chunk_enable(node_enable);
            if (GenApi::IsReadable(chunk_enable) && 
                chunk_enable->GetValue() == true)
            {
              GenICam::gcstring chunk_value_name("Chunk");
              chunk_value_name.append(chunk_name.c_str());
              GenApi::CNodePtr p_chunk_value(mp->GetNode(
                    GenICam::gcstring(chunk_value_name)));
              if (!p_chunk_value) continue;
              MapChunk::iterator itm = map_chunk_.find(p_chunk_value
                  ->GetName().c_str());
              if (itm == map_chunk_.end() || itm->second == nullptr)
              {
                map_chunk_[p_chunk_value->GetName().c_str()] = p_chunk_value;
              }
            }
          }
        }
        if (revert_chunk_mode) 
        {
          chunk_active->SetValue(false);
        }
      }
    }
    catch(GenICam::GenericException &x)
    {
      RCLCPP_ERROR(get_logger(), "%s %s %d: \n\t%s GenICam error when checking chunk: %s",
          __FILE__,__func__,__LINE__, 
          camera_namespace_.c_str(), x.GetDescription());
    }
  }

  void StCameraInterfaceImpl::eventInterfaceCB(StApi::IStCallbackParamBase *p, void */*pvContext*/)
  {
    if (destroyed_) return;
    stcamera_msgs::msg::Event msg;
    msg.timestamp = clock_.now();
    msg.genicam_module = "Interface";
    if (p->GetCallbackType() == StApi::StCallbackType_GenTLEvent_InterfaceError)
    {
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
    }
    publishEventDefault(msg); 
  }

  void StCameraInterfaceImpl::eventDeviceCB(StApi::IStCallbackParamBase *p, void */*pvContext*/)
  {
    if (destroyed_) return;
    if (tl_dev_ && tl_dev_->IsDeviceLost()) return;
    stcamera_msgs::msg::Event msg;
    msg.timestamp = clock_.now();
    msg.genicam_module = "LocalDevice";
    if (p->GetCallbackType() == StApi::StCallbackType_GenTLEvent_DeviceError)
    {
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
    }
    publishEventDefault(msg); 
  }

  void StCameraInterfaceImpl::eventGenApiNodeCB(GenApi::INode *p, void *pvContext)
  {
    if (destroyed_) return;
    MapCallback *cblist[5] = 
    {
      &map_event_system_, 
      &map_event_interface_, 
      &map_event_localdevice_,
      &map_event_remotedevice_, 
      &map_event_datastream_
    }; 
    std::string cbModule[5] = 
    {
      "System", "Interface", "LocalDevice", "RemoteDevice", "DataStream"
    };
    MapCallback *cb = nullptr;
    int index = -1;
    std::string callback_node = p->GetName().c_str();

    // find the correct module
    for (int i = 0; i < 5; i ++)
    {
      if (pvContext == (void*)(&(*cblist[i])))
      {
        cb = cblist[i];
        index = i;
        break;
      }
    }
    if (-1 == index) return; // not found (shall never happen)
    StCallback_t &stc = (*cb)[callback_node];
    stcamera_msgs::msg::Event msg;
    msg.timestamp = clock_.now();
    msg.genicam_module = cbModule[index];
    msg.event_name = stc.event_name_;
    msg.callback_node = callback_node;
    if (GenApi::IsReadable(p))
    {
      GenApi::CValuePtr pValue(p);
      msg.event_data = pValue->ToString();
    }
    MapPublisher::iterator it = map_msg_event_.find(stc.topic_name_);
    if (it != map_msg_event_.end()) // publish to custom topic
    {
      it->second->publish(msg);
      return;
    }
    publishEventDefault(msg);  // publish to default topic
  }

  void StCameraInterfaceImpl::eventDataStreamCB(StApi::IStCallbackParamBase *p, 
                                            void */*pvContext*/)
  {
    if (destroyed_) return;
    if (p->GetCallbackType() == 
        StApi::StCallbackType_GenTLEvent_DataStreamError)
    {
      stcamera_msgs::msg::Event msg;
      msg.timestamp = clock_.now();
      msg.genicam_module = "DataStream";
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
      publishEventDefault(msg);
      return;
    }
  
    if(p->GetCallbackType() == 
        StApi::StCallbackType_GenTLEvent_DataStreamNewBuffer)
    {
      try
      {
        StApi::CIStStreamBufferPtr p_streambuffer(tl_ds_->RetrieveBuffer(0));

        if (it_campub_.getNumSubscribers() == 0 && 
            msg_chunk_->get_subscription_count() == 0) 
        {
          return;
        }

        const StApi::IStStreamBufferInfo *p_streambufferInfo = p_streambuffer->GetIStStreamBufferInfo();
        rclcpp::Time timestamp;
        try
        {
          const uint64_t nTimestamp = p_streambufferInfo->GetTimestampNS();
          timestamp = rclcpp::Time((int64_t)nTimestamp);
        }
        catch(...)
        {
          try
          {
            timestamp = rclcpp::Time((double)p_streambufferInfo->GetTimestamp()/1000000.0);
          }
          catch(...)
          {

          }
        }

        stcamera_msgs::msg::Chunk chunkdata;
        bool has_chunk = false;
        if (map_chunk_.size() > 0)
        {
          has_chunk = true;
          GenApi::CNodeMapPtr cmp(p_streambuffer->GetChunkINodeMap());
          for (MapChunk::iterator it = map_chunk_.begin(); 
              it != map_chunk_.end(); it++)
          {
            GenApi::INode *node = it->second;
            if (node == nullptr) continue;
            switch(node->GetPrincipalInterfaceType())
            {
              case(GenApi::intfIInteger):
              {
                GenApi::CIntegerPtr ptr(node);
                stcamera_msgs::msg::ElementInt64 data;
                data.value = ptr->GetValue();
                data.name = it->first;
                chunkdata.int64_list.push_back(data);
                break;
              }
              case GenApi::intfIFloat:
              {
                GenApi::CFloatPtr ptr(node);
                stcamera_msgs::msg::ElementFloat64 data;
                data.value = ptr->GetValue();
                data.name = it->first;
                chunkdata.float64_list.push_back(data);
                break; 
              }
              case GenApi::intfIBoolean:
              {
                GenApi::CBooleanPtr ptr(node);
                stcamera_msgs::msg::ElementBool data;
                data.value = ptr->GetValue();
                data.name = it->first;
                chunkdata.bool_list.push_back(data);
                break; 
              }
              case GenApi::intfIString:
              {
                GenApi::CStringPtr ptr(node);
                stcamera_msgs::msg::ElementString data;
                data.value = ptr->GetValue();
                data.name = it->first;
                chunkdata.string_list.push_back(data);
                break; 
              }
              case GenApi::intfIValue:
              {
                GenApi::CValuePtr ptr(node);
                stcamera_msgs::msg::ElementString data;
                data.value = ptr->ToString();
                data.name = it->first;
                chunkdata.string_list.push_back(data);
                break;
              }
              case(GenApi::intfIBase):
              case(GenApi::intfICommand):
              case(GenApi::intfIRegister):
              case(GenApi::intfICategory):
              case(GenApi::intfIEnumeration):
              case(GenApi::intfIEnumEntry):
              case(GenApi::intfIPort):
                break;
            }
          }
        }

        if (p_streambufferInfo->IsImagePresent())
        {
          StApi::IStImage *p_stimage = p_streambuffer->GetIStImage();
          const StApi::EStPixelFormatNamingConvention_t ePFNC = 
              p_stimage->GetImagePixelFormat();

          std::string encoding = "";
          StApi::EStPixelFormatNamingConvention_t eDestPFNC;
          switch(ePFNC)
          {
            case(StPFNC_Mono1p):
            case(StPFNC_Mono2p):
            case(StPFNC_Mono4p):
            case(StPFNC_Mono8):
              eDestPFNC = StPFNC_Mono8;
              encoding = sensor_msgs::image_encodings::MONO8;
              break;
            case(StPFNC_Mono10):
            case(StPFNC_Mono10p):
            case(StPFNC_Mono10Packed):
            case(StPFNC_Mono12):
            case(StPFNC_Mono12p):
            case(StPFNC_Mono12Packed):
            case(StPFNC_Mono14):
            case(StPFNC_Mono14p):
            case(StPFNC_Mono16):
              eDestPFNC = StPFNC_Mono16;
              encoding = sensor_msgs::image_encodings::MONO16;
              break;
            case(StPFNC_BayerBG4p):
            case(StPFNC_BayerBG8):
              eDestPFNC = StPFNC_BayerBG8;
              encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
              break;
            case(StPFNC_BayerBG10):
            case(StPFNC_BayerBG10p):
            case(StPFNC_BayerBG10Packed):
            case(StPFNC_BayerBG12):
            case(StPFNC_BayerBG12p):
            case(StPFNC_BayerBG12Packed):
            case(StPFNC_BayerBG14):
            case(StPFNC_BayerBG14p):
            case(StPFNC_BayerBG16):
              eDestPFNC = StPFNC_BayerBG16;
              encoding = sensor_msgs::image_encodings::BAYER_BGGR16;
              break;
            case(StPFNC_BayerGB4p):
            case(StPFNC_BayerGB8):
              eDestPFNC = StPFNC_BayerGB8;
              encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
              break;
            case(StPFNC_BayerGB10):
            case(StPFNC_BayerGB10p):
            case(StPFNC_BayerGB10Packed):
            case(StPFNC_BayerGB12):
            case(StPFNC_BayerGB12p):
            case(StPFNC_BayerGB12Packed):
            case(StPFNC_BayerGB14):
            case(StPFNC_BayerGB14p):
            case(StPFNC_BayerGB16):
              eDestPFNC = StPFNC_BayerGB16;
              encoding = sensor_msgs::image_encodings::BAYER_GBRG16;
              break;
            case(StPFNC_BayerRG4p):
            case(StPFNC_BayerRG8):
              eDestPFNC = StPFNC_BayerRG8;
              encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
              break;
            case(StPFNC_BayerRG10):
            case(StPFNC_BayerRG10p):
            case(StPFNC_BayerRG10Packed):
            case(StPFNC_BayerRG12):
            case(StPFNC_BayerRG12p):
            case(StPFNC_BayerRG12Packed):
            case(StPFNC_BayerRG14):
            case(StPFNC_BayerRG14p):
            case(StPFNC_BayerRG16):
              eDestPFNC = StPFNC_BayerRG16;
              encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
              break;
            case(StPFNC_BayerGR4p):
            case(StPFNC_BayerGR8):
              eDestPFNC = StPFNC_BayerGR8;
              encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
              break;
            case(StPFNC_BayerGR10):
            case(StPFNC_BayerGR10p):
            case(StPFNC_BayerGR10Packed):
            case(StPFNC_BayerGR12):
            case(StPFNC_BayerGR12p):
            case(StPFNC_BayerGR12Packed):
            case(StPFNC_BayerGR14):
            case(StPFNC_BayerGR14p):
            case(StPFNC_BayerGR16):
              eDestPFNC = StPFNC_BayerGR16;
              encoding = sensor_msgs::image_encodings::BAYER_GRBG16;
              break;
            case(StPFNC_BGR8):
              eDestPFNC = StPFNC_BGR8;
              encoding = sensor_msgs::image_encodings::BGR8;
              break;
            case(StPFNC_BGRa8):
              eDestPFNC = StPFNC_BGRa8;
              encoding = sensor_msgs::image_encodings::BGRA8;
              break;
            case(StPFNC_BGR10):
            case(StPFNC_BGR10p):
            case(StPFNC_CBGR10p32):
            case(StPFNC_BGR12):
            case(StPFNC_BGR12p):
            case(StPFNC_BGR14):
            case(StPFNC_BGR16):
              eDestPFNC = StPFNC_BGR16;
              encoding = sensor_msgs::image_encodings::BGR16;
              break;
            case(StPFNC_BGRa10):
            case(StPFNC_BGRa12):
            case(StPFNC_BGRa14):
            case(StPFNC_BGRa16):
              eDestPFNC = StPFNC_BGRa16;
              encoding = sensor_msgs::image_encodings::BGRA16;
              break;
            case(StPFNC_RGB8):
            case(StPFNC_RGB8_Planar):
              eDestPFNC = StPFNC_RGB8;
              encoding = sensor_msgs::image_encodings::RGB8;
              break;
            case(StPFNC_RGBa8):
              eDestPFNC = StPFNC_RGBa8;
              encoding = sensor_msgs::image_encodings::RGBA8;
              break;
            case(StPFNC_RGB10):
            case(StPFNC_RGB10p):
            case(StPFNC_RGB10p32):
            case(StPFNC_RGB12):
            case(StPFNC_RGB12p):
            case(StPFNC_RGB14):
            case(StPFNC_RGB16):
            case(StPFNC_RGB10_Planar):
            case(StPFNC_RGB12_Planar):
            case(StPFNC_RGB16_Planar):
              eDestPFNC = StPFNC_RGB16;
              encoding = sensor_msgs::image_encodings::RGB16;
              break;
            case(StPFNC_RGBa10):
            case(StPFNC_RGBa12):
            case(StPFNC_RGBa14):
            case(StPFNC_RGBa16):
              eDestPFNC = StPFNC_RGBa16;
              encoding = sensor_msgs::image_encodings::RGBA16;
              break;
            case(StPFNC_Unknown):
            case(StPFNC_Mono32):
            case(StPFNC_YCbCr8):
            case(StPFNC_YCbCr411_8):
            case(StPFNC_YCbCr422_8):
            case(StPFNC_YCbCr709_422_8):
            case(StPFNC_YCbCr601_422_8):
            case(StPFNC_YCbCr411_8_CbYYCrYY):
            case(StPFNC_YCbCr601_411_8_CbYYCrYY):
            case(StPFNC_YCbCr709_411_8_CbYYCrYY):
            case(StPFNC_YCbCr422_8_CbYCrY):
            case(StPFNC_YCbCr601_422_8_CbYCrY):
            case(StPFNC_YCbCr709_422_8_CbYCrY):
            case(StPFNC_YCbCr8_CbYCr):
            case(StPFNC_YCbCr601_8_CbYCr):
            case(StPFNC_YCbCr709_8_CbYCr):
            case(StPFNC_YUV411_8_UYYVYY):
            case(StPFNC_YUV422_8_UYVY):
            case(StPFNC_YUV8_UYV):
            case(StPFNC_YUV422_8):
            case(StPFNC_Data8):
            case(StPFNC_Data8s):
            case(StPFNC_Data16):
            case(StPFNC_Data16s):
            case(StPFNC_Data32):
            case(StPFNC_Data32f):
            case(StPFNC_Data32s):
            case(StPFNC_Data64):
            case(StPFNC_Data64f):
            case(StPFNC_Data64s):
            case(StPFNC_Pol1Mono8):
            case(StPFNC_Pol1MonoX8):
            case(StPFNC_Pol1MonoY8):
            case(StPFNC_Pol1MonoXY8):
            case(StPFNC_Pol1Mono10):
            case(StPFNC_Pol1MonoX10):
            case(StPFNC_Pol1MonoY10):
            case(StPFNC_Pol1MonoXY10):
            case(StPFNC_Pol1Mono12):
            case(StPFNC_Pol1MonoX12):
            case(StPFNC_Pol1MonoY12):
            case(StPFNC_Pol1MonoXY12):
            case(StPFNC_Pol1BayerRG8):
            case(StPFNC_Pol1BayerRGX8):
            case(StPFNC_Pol1BayerRGY8):
            case(StPFNC_Pol1BayerRGXY8):
            case(StPFNC_Pol1BayerRG10):
            case(StPFNC_Pol1BayerRGX10):
            case(StPFNC_Pol1BayerRGY10):
            case(StPFNC_Pol1BayerRGXY10):
            case(StPFNC_Pol1BayerRG12):
            case(StPFNC_Pol1BayerRGX12):
            case(StPFNC_Pol1BayerRGY12):
            case(StPFNC_Pol1BayerRGXY12):
            case(StPFNC_Pol1Mono10p):
            case(StPFNC_Pol1MonoX10p):
            case(StPFNC_Pol1MonoY10p):
            case(StPFNC_Pol1MonoXY10p):
            case(StPFNC_Pol1Mono12p):
            case(StPFNC_Pol1MonoX12p):
            case(StPFNC_Pol1MonoY12p):
            case(StPFNC_Pol1MonoXY12p):
            case(StPFNC_Pol1BayerRG10p):
            case(StPFNC_Pol1BayerRGX10p):
            case(StPFNC_Pol1BayerRGY10p):
            case(StPFNC_Pol1BayerRGXY10p):
            case(StPFNC_Pol1BayerRG12p):
            case(StPFNC_Pol1BayerRGX12p):
            case(StPFNC_Pol1BayerRGY12p):
            case(StPFNC_Pol1BayerRGXY12p):
            case(StPFNC_Pol1MonoC8):
            case(StPFNC_Pol1MonoXC8):
            case(StPFNC_Pol1MonoYC8):
            case(StPFNC_Pol1MonoXYC8):
            case(StPFNC_Pol1MonoC10):
            case(StPFNC_Pol1MonoXC10):
            case(StPFNC_Pol1MonoYC10):
            case(StPFNC_Pol1MonoXYC10):
            case(StPFNC_Pol1MonoC12):
            case(StPFNC_Pol1MonoXC12):
            case(StPFNC_Pol1MonoYC12):
            case(StPFNC_Pol1MonoXYC12):
            case(StPFNC_Pol1BayerRGC8):
            case(StPFNC_Pol1BayerRGXC8):
            case(StPFNC_Pol1BayerRGYC8):
            case(StPFNC_Pol1BayerRGXYC8):
            case(StPFNC_Pol1BayerRGC10):
            case(StPFNC_Pol1BayerRGXC10):
            case(StPFNC_Pol1BayerRGYC10):
            case(StPFNC_Pol1BayerRGXYC10):
            case(StPFNC_Pol1BayerRGC12):
            case(StPFNC_Pol1BayerRGXC12):
            case(StPFNC_Pol1BayerRGYC12):
            case(StPFNC_Pol1BayerRGXYC12):
            case(StPFNC_Pol1MonoC10p):
            case(StPFNC_Pol1MonoXC10p):
            case(StPFNC_Pol1MonoYC10p):
            case(StPFNC_Pol1MonoXYC10p):
            case(StPFNC_Pol1MonoC12p):
            case(StPFNC_Pol1MonoXC12p):
            case(StPFNC_Pol1MonoYC12p):
            case(StPFNC_Pol1MonoXYC12p):
            case(StPFNC_Pol1BayerRGC10p):
            case(StPFNC_Pol1BayerRGXC10p):
            case(StPFNC_Pol1BayerRGYC10p):
            case(StPFNC_Pol1BayerRGXYC10p):
            case(StPFNC_Pol1BayerRGC12p):
            case(StPFNC_Pol1BayerRGXC12p):
            case(StPFNC_Pol1BayerRGYC12p):
            case(StPFNC_Pol1BayerRGXYC12p):
            case(StPFNC_CYCbCr420_8_YY_Cb_Cr_Planar):
              break;
          }
          if (encoding.empty())
          {
            RCLCPP_WARN(get_logger(), "%s %s %d: %s: %ld: %ld x %ld: unknown encoding %d",
                __FILE__,__func__,__LINE__,
                camera_namespace_.c_str(),
                p_streambuffer->GetIStStreamBufferInfo()->GetFrameID(),
                p_stimage->GetImageWidth(), 
                p_stimage->GetImageHeight(), 
                (int)ePFNC);
            return;
          }

          const StApi::IStPixelFormatInfo *const p_pixelformat_info = 
              StApi::GetIStPixelFormatInfo(ePFNC);
          if (p_pixelformat_info->IsMono() || p_pixelformat_info->IsBayer() || p_pixelformat_info->IsColor())
          {
            //image data

#if ENABLED_STAPI_ZERO_COPY
            if(ePFNC != eDestPFNC)
            {
              p_stpixel_format_converter_->SetDestinationPixelFormat(eDestPFNC);
              p_stpixel_format_converter_->Convert(p_stimage, p_stimage_buffer_);
              p_stimage = p_stimage_buffer_->GetIStImage();
            }
            sensor_msgs::msg::Image* pimage = image_allocator_.get_image(p_stimage->GetImageBuffer());
#else
            sensor_msgs::msg::Image* pimage = &image_;
            pimage->header.frame_id = camera_namespace_;
#endif //ENABLED_STAPI_ZERO_COPY

            pimage->header.stamp = timestamp;
            pimage->width = p_stimage->GetImageWidth();
            pimage->height = p_stimage->GetImageHeight();
            pimage->step = p_stimage->GetImageLinePitch();
            pimage->encoding = encoding;
            pimage->is_bigendian = (p_streambuffer->GetIStStreamBufferInfo()->GetPixelEndianness() == GenTL::PIXELENDIANNESS_BIG);
            //camera info data
            camera_info_.header.stamp = timestamp;
            if (it_campub_.getNumSubscribers() > 0)
            {
#if ENABLED_STAPI_ZERO_COPY
#else
              const size_t buffer_size = image_.height * image_.step;
              image_.data.resize(buffer_size);
              memcpy(&image_.data[0], p_stimage->GetImageBuffer(), buffer_size);
#endif //ENABLED_STAPI_ZERO_COPY
              //std::cout << "Grabbed:" << timestamp.nanoseconds() << std::endl;
              it_campub_.publish(*pimage, camera_info_);
            }


          }
        }
        if (has_chunk && msg_chunk_->get_subscription_count() > 0)
        {
          chunkdata.timestamp = timestamp;
          msg_chunk_->publish(chunkdata);
        }
      }
      catch(const StApi::CStGenTLErrorException &x)
      {
        RCLCPP_ERROR(get_logger(), "%s %s %d: \n\t%s GenTL error when retrieving payload: %d %s", 
            __FILE__,__func__,__LINE__, 
            camera_namespace_.c_str(), (EOSError_t)x.GetError(), x.GetDescription());
      }
      catch(GenICam::GenericException &x)
      {
        RCLCPP_ERROR(get_logger(), "%s %s %d: \n\t%s GenICam error when retrieving payload: %s",
            __FILE__,__func__,__LINE__, 
            camera_namespace_.c_str(), x.GetDescription());
      }
    }
  }

  // read node
  bool StCameraInterfaceImpl::readNodeCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNode_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNode_Response> res)
  {
    try
    {
      GenApi::INodeMap* mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      res->interface_type = GenApi::GetInterfaceName(node).c_str();
      switch(node->GetPrincipalInterfaceType())
      {
        case GenApi::intfIInteger:
        {
          GenApi::CIntegerPtr ptr(node);
          res->value = ptr->ToString();
          break;
        }
        case GenApi::intfIBoolean:
        {
          GenApi::CBooleanPtr ptr(node);
          res->value = ptr->ToString();
          break;
        }
        case GenApi::intfIFloat:
        {
          GenApi::CFloatPtr ptr(node);
          res->value = ptr->ToString();
          break;
        }
        case GenApi::intfIString:
        {
          GenApi::CStringPtr ptr(node);
          res->value = ptr->GetValue().c_str();
          break;
        }
        case GenApi::intfIRegister:
        {
          GenApi::CRegisterPtr ptr(node);
          res->value = ptr->ToString();
          break;
        }
        case GenApi::intfICategory:
        {
          GenApi::CCategoryPtr ptr(node);
          res->value = ptr->ToString();
          break;
        }
        case GenApi::intfIEnumeration:
        {
          GenApi::CEnumerationPtr ptr(node);
          res->value = ptr->ToString();
          break;
        }
        default:
        {
          RETURN_ERR_RES(OSError_NodeError, OSERROR_STR_NODE_ERROR, res);
        }
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::readNodeBoolCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodeBool_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodeBool_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CBooleanPtr ptr(node);
      res->value = ptr->GetValue();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::readNodeEnumCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodeEnum_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodeEnum_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CEnumerationPtr ptr(node);
      res->value_int = ptr->GetIntValue();
      res->value_str = ptr->GetEntry(res->value_int)->GetSymbolic();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::readNodeIntCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodeInt_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodeInt_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CIntegerPtr ptr(node);
      res->value = ptr->GetValue();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }
  bool StCameraInterfaceImpl::readNodeFloatCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodeFloat_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodeFloat_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CFloatPtr ptr(node);
      res->value = ptr->GetValue();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::readNodePortCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodePort_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodePort_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CPortPtr ptr(node);
      res->data.resize(req->length);
      ptr->Read(&(res->data[0]), req->address, req->length);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::readNodeRegisterCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodeRegister_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodeRegister_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CRegisterPtr ptr(node);
      res->data.resize(req->length);
      ptr->Get(&(res->data[0]), req->length);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::readNodeRegisterInfoCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodeRegisterInfo_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodeRegisterInfo_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CRegisterPtr ptr(node);
      res->length = ptr->GetLength();
      res->address = ptr->GetAddress();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::readNodeStringCallback(
      const std::shared_ptr<stcamera_msgs::srv::ReadNodeString_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ReadNodeString_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CStringPtr ptr(node);
      res->value = ptr->GetValue();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  // write node
  bool StCameraInterfaceImpl::writeNodeCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNode_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNode_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      switch(node->GetPrincipalInterfaceType())
      {
        case GenApi::intfIInteger:
        {
          GenApi::CIntegerPtr ptr(node);
          ptr->SetValue(std::stoll(req->value));
          break;
        }
        case GenApi::intfIBoolean:
        {
          GenApi::CBooleanPtr ptr(node);
          std::string true_str = "true";
          std::string false_str = "false";
          bool true_value = false;
          bool false_value = false;
          if (req->value.size() == true_str.size())
          {
            true_value = true;
            for (std::string::const_iterator c1 = true_str.begin(), 
                c2 = req->value.begin(); c1 != true_str.end(); ++c1, ++c2) 
            {
              if (tolower(*c1) != tolower(*c2)) 
              {
                true_value = false;
                break;
              }
            }
            if (true_value)
            {
              ptr->SetValue(true);
              break;
            }
          }
          if (req->value.size() == false_str.size())
          {
            false_value = true;
            for (std::string::const_iterator c1 = false_str.begin(), 
                c2 = req->value.begin(); c1 != false_str.end(); ++c1, ++c2) 
            {
              if (tolower(*c1) != tolower(*c2)) 
              {
                false_value = false;
                break;
              }
            }
            if (false_value)
            {
              ptr->SetValue(false);
              break;
            }
          }
          RETURN_ERR_RES(OSError_NodeError, OSERROR_STR_NODE_ERROR, res);
        }
        case GenApi::intfIFloat:
        {
          GenApi::CFloatPtr ptr(node);
          ptr->SetValue(std::stod(req->value));
          break;
        }
        case GenApi::intfIString:
        {
          GenApi::CStringPtr ptr(node);
          ptr->SetValue(GenICam::gcstring(req->value.c_str()));
          break;
        }
        case GenApi::intfIEnumeration:
        {
          GenApi::CEnumerationPtr ptr(node);
          *ptr = req->value.c_str();
          break;
        }
        default:
        {
          RETURN_ERR_RES(OSError_NodeError, OSERROR_STR_NODE_ERROR, res);
        }
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::writeNodeBoolCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodeBool_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodeBool_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CBooleanPtr ptr(node);
      ptr->SetValue(req->value);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::writeNodeEnumIntCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumInt_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumInt_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CEnumerationPtr ptr(node);
      ptr->SetIntValue(req->value);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }
  bool StCameraInterfaceImpl::writeNodeEnumStrCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumStr_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumStr_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CEnumerationPtr ptr(node);
      *ptr = req->value.c_str();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::writeNodeIntCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodeInt_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodeInt_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CIntegerPtr ptr(node);
      ptr->SetValue(req->value);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::writeNodeFloatCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodeFloat_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodeFloat_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CFloatPtr ptr(node);
      ptr->SetValue(req->value);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }
  bool StCameraInterfaceImpl::writeNodeRegisterCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodeRegister_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodeRegister_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CRegisterPtr ptr(node);
      ptr->Set(&(req->data[0]), req->length);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }
  bool StCameraInterfaceImpl::writeNodePortCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodePort_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodePort_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CPortPtr ptr(node);
      ptr->Write(&(req->data[0]), req->address, req->length);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }
  bool StCameraInterfaceImpl::writeNodeStringCallback(
      const std::shared_ptr<stcamera_msgs::srv::WriteNodeString_Request> req,
      std::shared_ptr<stcamera_msgs::srv::WriteNodeString_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CStringPtr ptr(node);

      const GenICam::gcstring str = req->value.c_str();
      ptr->SetValue(str);
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::executeNodeCallback(
      const std::shared_ptr<stcamera_msgs::srv::ExecuteNode_Request> req,
      std::shared_ptr<stcamera_msgs::srv::ExecuteNode_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CCommandPtr ptr(node);
      ptr->Execute();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  // enable
  bool StCameraInterfaceImpl::enableChunkCallback(
      const std::shared_ptr<stcamera_msgs::srv::EnableChunk_Request> req,
      std::shared_ptr<stcamera_msgs::srv::EnableChunk_Response> res)
  {
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);
    if (bool_acquisition_is_started_)
    {
      RETURN_ERR_RES(OSError_ImageAcquisitionAlreadyOnError, OSERROR_STR_IMAGE_ACQ_ALREADY_ON_ERROR, res);
    }

    std::lock_guard<std::mutex> lock2(mtx_chunk_);

    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      GenApi::CEnumerationPtr chunk_selector(mp->GetNode(
            GenICam::gcstring("ChunkSelector")));
      GenApi::CBooleanPtr chunk_enable(mp->GetNode(
            GenICam::gcstring("ChunkEnable")));
      GenApi::CBooleanPtr chunk_active(mp->GetNode(
            GenICam::gcstring("ChunkModeActive"))); 
      if (!chunk_selector.IsValid() || !chunk_enable.IsValid() ||
          !chunk_active.IsValid())
      {
        RETURN_ERR_RES(OSError_ChunkNotSupportedError, OSERROR_STR_CHUNK_NOT_SUPPORTED_ERROR, res);
      }

      bool revert_chunk_mode = (false == chunk_active->GetValue());
      if (revert_chunk_mode) 
      {
        chunk_active->SetValue(true);
      }

      if (req->chunk_name.empty()) // All chunks
      {
        GenApi::NodeList_t chunk_list;
        chunk_selector->GetEntries(chunk_list);
        for (GenApi::NodeList_t::iterator itr = chunk_list.begin(); 
            itr != chunk_list.end(); ++itr)
        {
          GenApi::CEnumEntryPtr chunk_list_entry(*itr);
          if (GenApi::IsAvailable(chunk_list_entry))
          {
            chunk_selector->SetIntValue(chunk_list_entry->GetValue());
            if (GenApi::IsWritable(chunk_enable))
            {
              chunk_enable->SetValue(req->value);
              GenApi::CNodePtr chunk_value_node(mp->GetNode(
                  "Chunk" + chunk_list_entry->GetSymbolic()));
              if (chunk_value_node && req->value)
              {
                MapChunk::iterator itm = map_chunk_.find(chunk_value_node
                    ->GetName().c_str());
                if (itm == map_chunk_.end() || itm->second == nullptr)
                {
                  map_chunk_[chunk_value_node->GetName().c_str()] = 
                      chunk_value_node;
                }
              }
            }
          }
        }
        if (!req->value) 
        {
          map_chunk_.clear();
          chunk_active->SetValue(false);
        }
      }
      else
      {
        bool is_writable = true;
        bool all_chunk_disabled;
        GenApi::NodeList_t nodelist;
        GenApi::CEnumEntryPtr chunk_list_entry(chunk_selector
            ->GetEntryByName(GenICam::gcstring(req->chunk_name.c_str())));
        if (chunk_list_entry.IsValid() &&
            GenApi::IsReadable(chunk_list_entry))
        {
          chunk_selector->SetIntValue(chunk_list_entry->GetValue());
          if (!GenApi::IsWritable(chunk_enable))
          {
            is_writable = false;
          }
        }

        if (!is_writable)
        {
          if (revert_chunk_mode) 
          {
            chunk_active->SetValue(false);
          }
          RETURN_ERR_RES(OSError_ChunkInaccessibleError, OSERROR_STR_CHUNK_INACCESSIBLE_ERROR, res);
        }

        chunk_enable->SetValue(req->value);

        GenICam::gcstring chunk_value_name("Chunk");
        chunk_value_name.append(req->chunk_name.c_str());
        GenApi::CNodePtr chunk_value_node(mp->GetNode(chunk_value_name));
        if (chunk_value_node)
        {
          MapChunk::iterator itm = map_chunk_.find(chunk_value_node
              ->GetName().c_str());
          if (req->value)
          {
            if (itm == map_chunk_.end() || itm->second == nullptr)
            {
              map_chunk_[chunk_value_node->GetName().c_str()] = 
                  chunk_value_node;
            }
          }
        }
        
        // Disable chunk active if only Image is enabled.
        all_chunk_disabled = true;
        chunk_selector->GetEntries(nodelist);
        for (GenApi::NodeList_t::iterator it = nodelist.begin();
            it != nodelist.end(); it++)
        {
          if (GenApi::IsAvailable(*it))
          {
            GenApi::CEnumEntryPtr enum_entry(*it);
            if (enum_entry->GetValue() == 0) continue; // skip image.
            chunk_selector->SetIntValue(enum_entry->GetValue());
            if (chunk_enable->GetValue() == true)     
            {
              all_chunk_disabled = false;
              break;
            }
          }
        }
        if (all_chunk_disabled)
        {
          chunk_active->SetValue(false);
        }
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::enableTriggerCallback(
      const std::shared_ptr<stcamera_msgs::srv::EnableTrigger_Request> req,
      std::shared_ptr<stcamera_msgs::srv::EnableTrigger_Response> res)
  {
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      // Node for selector and source
      GenApi::CEnumerationPtr trigger_selector(mp->GetNode(GenICam::gcstring("TriggerSelector")));
      if (!trigger_selector.IsValid())
      {
        RETURN_ERR_RES(OSError_TriggerNotSupportedError,OSERROR_STR_TRIGGER_NOT_SUPPORTED_ERROR, res); 
      }
      if(GenApi::IsWritable(trigger_selector))
      {
        *trigger_selector = req->trigger_selector.c_str();
      }
      else
      {
        RETURN_ERR_RES(OSError_TriggerInaccessibleError, OSERROR_STR_TRIGGER_INACCESSIBLE_ERROR, res); 
      }
      // set mode
      GenApi::CEnumerationPtr trigger_mode = mp->GetNode(GenICam::gcstring("TriggerMode"));
      if(GenApi::IsWritable(trigger_mode))
      {
        *trigger_mode = req->value ? "On" : "Off";
      }
      else
      {
        RETURN_ERR_RES(OSError_TriggerInaccessibleError, OSERROR_STR_TRIGGER_INACCESSIBLE_ERROR, res); 
      }

      GenApi::CEnumerationPtr trigger_source(mp->GetNode(GenICam::gcstring("TriggerSource")));
      if(GenApi::IsWritable(trigger_source))
      {
        *trigger_source = req->trigger_source.c_str();
      }
      // set delay only if supported
      GenApi::CFloatPtr trigger_delay = mp->GetNode(GenICam::gcstring("TriggerDelay"));
      if(GenApi::IsWritable(trigger_delay))
      {
        trigger_delay->SetValue(req->trigger_delayus);
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::enableEventNodeCallback(
      const std::shared_ptr<stcamera_msgs::srv::EnableEventNode_Request> req,
      std::shared_ptr<stcamera_msgs::srv::EnableEventNode_Response> res)
  {
    std::lock_guard<std::mutex> lock(mtx_event_);
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *nodeCallback = mp->GetNode(GenICam::gcstring(req->callback_node.c_str()));
      CHECK_NULLPTR_RES(nodeCallback, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CEnumerationPtr event_selector(mp->GetNode(
          GenICam::gcstring("EventSelector")));
      GenApi::CEnumerationPtr event_notif(mp->GetNode(
          GenICam::gcstring("EventNotification"))); 
      if (!event_selector.IsValid() || !event_notif.IsValid() ||
          !GenApi::IsWritable(event_selector))
      {
        RETURN_ERR_RES(OSError_EventNotSupportedError, OSERROR_STR_EVENT_NOT_SUPPORTED_ERROR, res); 
      }

      *event_selector = req->event_name.c_str();
      if (!GenApi::IsWritable(event_notif))
      {
        RETURN_ERR_RES(OSError_EventInaccessibleError, OSERROR_STR_EVENT_INACCESSIBLE_ERROR_STR, res); 
      }
      *event_notif = req->value ? "On" : "Off";
      
      GenApi::CNodePtr node_callback(nodeCallback);
      MapCallback *cb = getCallbackMap(req->genicam_module);
      MapCallback::iterator it = cb->find(req->callback_node);
      std::string topic_name = STMSG_event;
      if ((!req->event_topic_name.empty()) && (req->event_topic_name.compare("None") != 0))
      {
        topic_name += "/" + req->event_topic_name;
      }

      if (req->value) // set enabled = true
      {
        if (it == cb->end()) // not found in registered list
        {
          MapPublisher::iterator itpub = map_msg_event_.find(topic_name);
          if (itpub == map_msg_event_.end())
          {
            rclcpp::Publisher<stcamera_msgs::msg::Event>::SharedPtr pub = nh_->create_publisher<stcamera_msgs::msg::Event>(topic_name, queue_size_);
            map_msg_event_.insert(std::make_pair(topic_name, pub));
          }
          StCallback_t stc;
          stc.topic_name_ = topic_name;
          stc.event_name_ = req->event_name;
          stc.cb_ = StApi::RegisterCallback(node_callback, *this,
              &StCameraInterfaceImpl::eventGenApiNodeCB,
              (void*)(&(*cb)), GenApi::cbPostInsideLock);
          (*cb)[req->callback_node] = stc;
          RETURN_SUCCESS(res);
        }
        // already registered
        RETURN_ERR_RES(OSError_EventAlreadyOnError, OSERROR_STR_EVENT_ALREADY_ON_ERROR, res);
      }

      // set enabled = false
      if (it != cb->end()) // found in registered list
      {
        StApi::IStRegisteredCallbackReleasable *cbf = it->second.cb_;
        cbf->Release();
        cb->erase(it);
        if (topic_name.compare(std::string(camera_namespace_ + 
            "/" + STMSG_event)) != 0) // not using default event topic
        {
          // remove publisher if no other event callback is using the topic
          MapCallback *cblist[5] = 
          {
            &map_event_system_, 
            &map_event_interface_, 
            &map_event_localdevice_,
            &map_event_remotedevice_, 
            &map_event_datastream_
          }; 
          for (int i = 0; i < 5; i++)
          {
            for (MapCallback::iterator itcb = (*cblist[i]).begin();
                itcb != (*cblist[i]).end(); itcb++)
            { // if other event is using the topic, do nothing and return.
              if (itcb->second.topic_name_.compare(topic_name) == 0)
              {
                RETURN_SUCCESS(res);
              }
            } 
          } 
          MapPublisher::iterator itpub = map_msg_event_.find(topic_name);
          if (itpub != map_msg_event_.end())
          {
            map_msg_event_.erase(itpub);
          } 
        }
        RETURN_SUCCESS(res);
      }
      // not found in registered list
      RETURN_ERR_RES(OSError_EventAlreadyOffError, OSERROR_STR_EVENT_ALREADY_OFF_ERROR, res);
    }
    CATCH_COMMON_ERR_RES(res);
  }

  bool StCameraInterfaceImpl::enableImageAcquisitionCallback(
      const std::shared_ptr<stcamera_msgs::srv::EnableImageAcquisition_Request> req,
      std::shared_ptr<stcamera_msgs::srv::EnableImageAcquisition_Response> res)
  {
    std::lock_guard<std::mutex> lock(mtx_acquisition_);
    try
    {
      if (req->value)
      {
        //already started, return directly
        if (bool_acquisition_is_started_) 
        {
          RETURN_ERR_RES(OSError_ImageAcquisitionAlreadyOnError, OSERROR_STR_IMAGE_ACQ_ALREADY_ON_ERROR, res);
        }
        initializeCameraInfo();
        tl_ds_->StartAcquisition();
        tl_dev_->AcquisitionStart();
        bool_acquisition_is_started_ = true;
      }
      else
      {
        if (!bool_acquisition_is_started_)
        {
          RETURN_ERR_RES(OSError_ImageAcquisitionAlreadyOffError, OSERROR_STR_IMAGE_ACQ_ALREADY_OFF_ERROR, res);
        }
        tl_dev_->AcquisitionStop();
        tl_ds_->StopAcquisition();
        bool_acquisition_is_started_ = false;
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::enableEventAcquisitionCallback(
      const std::shared_ptr<stcamera_msgs::srv::EnableEventAcquisition_Request> req,
      std::shared_ptr<stcamera_msgs::srv::EnableEventAcquisition_Response> res)
  {
    std::lock_guard<std::mutex> lock(mtx_event_);
    try
    {
        StApi::IStEventCtrl *event_control = nullptr;
        bool *event_value = nullptr;
        if (req->genicam_module.compare("System") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(
              tl_dev_->GetIStInterface()->GetIStSystem());
          event_value = &bool_event_system_;
        }
        else if (req->genicam_module.compare("Interface") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(
              tl_dev_->GetIStInterface());
          event_value = &bool_event_interface_;
        }
        else if (req->genicam_module.compare("LocalDevice") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(&(*tl_dev_));
              //(StApi::IStDevice*)tl_dev_);
          event_value = &bool_event_device_;
        }
        else if (req->genicam_module.compare("DataStream") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(&(*tl_ds_));
          event_value = &bool_event_datastream_;
        }
        else if (req->genicam_module.compare("RemoteDevice") == 0)
        {
          RETURN_ERR_RES(OSError_ModuleError, OSERROR_STR_REMOTE_DEVUCE_EVENT_ALWAYS_ENABLED_ERROR, res);
        }
        else 
        {
          RETURN_ERR_RES(OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);
        }
 
        if (event_control != nullptr)
        {
          if (req->value) // set enable
          {
            if (*event_value)
            {
              RETURN_ERR_RES(OSError_EventAlreadyOnError, OSERROR_STR_EVENT_ALREADY_ON_ERROR, res);
            }
            event_control->StartEventAcquisitionThread();
            *event_value = true;
          }
          else
          {
            // set disable
            if (!(*event_value))
            {
              RETURN_ERR_RES(OSError_EventAlreadyOffError, OSERROR_STR_EVENT_ALREADY_OFF_ERROR, res);
            }
            event_control->StopEventAcquisitionThread();
            *event_value = false;
          }
        }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  // get
  bool StCameraInterfaceImpl::getImageAcquisitionStatusCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetImageAcquisitionStatus_Request> /*req*/,
      std::shared_ptr<stcamera_msgs::srv::GetImageAcquisitionStatus_Response> res)
  {
    std::lock_guard<std::mutex> lock(mtx_acquisition_);
    res->value = bool_acquisition_is_started_;
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::getEventAcquisitionStatusListCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetEventAcquisitionStatusList_Request>/*req*/,
      std::shared_ptr<stcamera_msgs::srv::GetEventAcquisitionStatusList_Response> res)
  {
    std::lock_guard<std::mutex> lock(mtx_event_);
    res->genicam_module_list.push_back("System");
    res->genicam_module_list.push_back("Interface");
    res->genicam_module_list.push_back("LocalDevice");
    res->genicam_module_list.push_back("RemoteDevice");
    res->genicam_module_list.push_back("DataStream");
    res->enabled_list.push_back(bool_event_system_); 
    res->enabled_list.push_back(bool_event_interface_); 
    res->enabled_list.push_back(bool_event_device_); //Local == Remote
    res->enabled_list.push_back(bool_event_device_); //Remote == Local
    res->enabled_list.push_back(bool_event_datastream_); 
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::getEventNodeStatusListCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetEventNodeStatusList_Request> req,
      std::shared_ptr<stcamera_msgs::srv::GetEventNodeStatusList_Response> res)
  {
    GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

    try
    {
      GenApi::CEnumerationPtr event_selector(mp->GetNode(
          GenICam::gcstring("EventSelector"))); 
      GenApi::CEnumerationPtr event_notif(mp->GetNode(
          GenICam::gcstring("EventNotification")));
      if (!event_selector.IsValid() || !event_notif.IsValid())
      {
        RETURN_ERR_RES(OSError_EventNotSupportedError, OSERROR_STR_EVENT_NOT_SUPPORTED_ERROR, res); 
      }
      if (GenApi::IsReadable(event_selector))
      {
        GenApi::NodeList_t nodelist;
        event_selector->GetEntries(nodelist);
        for (GenApi::NodeList_t::iterator it = nodelist.begin();
            it != nodelist.end(); it++)
        {
          if (!GenApi::IsAvailable(*it)) continue;

          GenApi::CEnumEntryPtr enum_entry(*it);
          event_selector->SetIntValue(enum_entry->GetValue());
          stcamera_msgs::msg::GenICamEvent genicam_event;
          genicam_event.name = std::string(enum_entry->GetSymbolic().c_str());
          genicam_event.enabled = (event_notif->GetCurrentEntry()
              ->GetSymbolic().compare("On") == 0 ? true : false);

          //list the callback node:
          std::string nodeData = "Event" + genicam_event.name + "Data";
          GenApi::CNodePtr event_category_node(mp->GetNode(
                GenICam::gcstring(nodeData.c_str())));
          if (event_category_node.IsValid())
          {
            MapCallback *cb = getCallbackMap(req->genicam_module);
            GenApi::FeatureList_t features;
            GenApi::CCategoryPtr event_category(event_category_node);
            event_category->GetFeatures(features);
            for (GenApi::FeatureList_t::iterator it = features.begin();
                it != features.end(); it++)
            {
              GenApi::INode *node = (*it)->GetNode();
              if (!GenApi::IsImplemented(node)) continue;
              std::string cb_name = node->GetName().c_str();
              genicam_event.callback_node_list.push_back(cb_name.c_str());
              MapCallback::iterator itmap = cb->find(cb_name);
              if (itmap == cb->end() || itmap->second.cb_ == nullptr) 
              {
                genicam_event.callback_enabled_list.push_back(false);
              }
              else
              {
                genicam_event.callback_enabled_list.push_back(true);
              }
            }
          }
          res->event_node_list.push_back(genicam_event);
        }
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::getChunkListCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetChunkList_Request> /*req*/,
      std::shared_ptr<stcamera_msgs::srv::GetChunkList_Response> res)
  {
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);
    if (bool_acquisition_is_started_)
    {
      RETURN_ERR_RES(OSError_ImageAcquisitionAlreadyOnError, OSERROR_STR_IMAGE_ACQ_ALREADY_ON_ERROR, res);
    }

    std::lock_guard<std::mutex> lock2(mtx_chunk_);
    GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();
    try
    {
      GenApi::CEnumerationPtr chunk_selector = mp->GetNode(
          GenICam::gcstring("ChunkSelector"));
      GenApi::NodeList_t nodelist;
      if (!chunk_selector.IsValid())
      {
        RETURN_ERR_RES(OSError_ChunkNotSupportedError, OSERROR_STR_CHUNK_NOT_SUPPORTED_ERROR, res);
      }

      GenApi::CBooleanPtr chunk_active = mp->GetNode(
          GenICam::gcstring("ChunkModeActive"));
      bool revert_chunk_mode = (false == chunk_active->GetValue());
      if (revert_chunk_mode) 
      {
        chunk_active->SetValue(true);
      }
      chunk_selector->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
          it != nodelist.end(); it++)
      {
        if (GenApi::IsAvailable(*it))
        {
          GenApi::CEnumEntryPtr enum_entry(*it);
          res->chunk_name_list.push_back(
              std::string(enum_entry->GetSymbolic().c_str()));
          chunk_selector->SetIntValue(enum_entry->GetValue());
          GenApi::CBooleanPtr enablePtr = mp->GetNode(
              GenICam::gcstring("ChunkEnable"));
          if (enablePtr.IsValid())
          {
            res->chunk_enabled_list.push_back(
                enablePtr->GetValue() ? true : false);
          }
        }
      }
      if (revert_chunk_mode) 
      {
        chunk_active->SetValue(false);
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::getTriggerListCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetTriggerList_Request> /*req*/,
      std::shared_ptr<stcamera_msgs::srv::GetTriggerList_Response> res)
  {
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      // Node for selector and source
      GenApi::CEnumerationPtr enum_sel = mp->GetNode(
          GenICam::gcstring("TriggerSelector"));
      GenApi::CEnumerationPtr enum_src = mp->GetNode(
          GenICam::gcstring("TriggerSource"));
      if (!enum_sel.IsValid() || !enum_src.IsValid())
      {
        RETURN_ERR_RES(OSError_TriggerNotSupportedError, OSERROR_STR_TRIGGER_NOT_SUPPORTED_ERROR, res); 
      }
      
      // Iterate selector
      GenApi::NodeList_t nodelist;
      enum_sel->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
          it != nodelist.end(); it++)
      {
        if (!GenApi::IsAvailable(*it)) continue;
        // Get selector
        GenApi::CEnumEntryPtr enum_entry(*it);
        res->trigger_selector_list.push_back(
            std::string(enum_entry->GetSymbolic().c_str()));
        enum_sel->SetIntValue(enum_entry->GetValue());

        // Get mode
        GenApi::CEnumerationPtr trigger_mode = mp->GetNode(
            GenICam::gcstring("TriggerMode"));
        if (trigger_mode.IsValid())
        {
          res->trigger_mode_list.push_back(trigger_mode->GetCurrentEntry()
              ->GetSymbolic().compare("On") == 0 ? true : false);
        }
        else
        {
          res->trigger_mode_list.push_back(false);
        }

        // Get delayus
        GenApi::CFloatPtr trigger_delay = mp->GetNode(
            GenICam::gcstring("TriggerDelay"));
        if (trigger_mode.IsValid() && GenApi::IsReadable(trigger_delay))
        {
          res->trigger_delayus_list.push_back(trigger_delay->GetValue());
        }
        else
        {
          res->trigger_delayus_list.push_back(0);
        }
      }

      // Iterate source
      nodelist.clear();
      enum_src->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
        it != nodelist.end(); it++)
      {
        if (!GenApi::IsAvailable(*it)) continue;
        // Get source
        GenApi::CEnumEntryPtr enum_entry(*it);
        res->trigger_source_list.push_back(
            std::string(enum_entry->GetSymbolic().c_str()));
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::getEnumListCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetEnumList_Request> req,
      std::shared_ptr<stcamera_msgs::srv::GetEnumList_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str())); 
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);

      GenApi::CEnumerationPtr ptr(node);
      GenApi::NodeList_t nodelist;
      ptr->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
          it != nodelist.end(); it++)
      {
        if (GenApi::IsAvailable(*it))
        {
          GenApi::CEnumEntryPtr enum_entry(*it);
          res->enum_value_int_list.push_back(enum_entry->GetValue());
          res->enum_value_str_list.push_back(
              std::string(enum_entry->GetSymbolic().c_str()));
          res->enum_node_list.push_back((*it)->GetName().c_str());
        }
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::getGenICamNodeInfoCallback(
      const std::shared_ptr<stcamera_msgs::srv::GetGenICamNodeInfo_Request> req,
      std::shared_ptr<stcamera_msgs::srv::GetGenICamNodeInfo_Response> res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req->genicam_module);
      CHECK_NULLPTR_RES(mp, OSError_ModuleError, OSERROR_STR_MODULE_ERROR, res);

      GenApi::INode *node = mp->GetNode(GenICam::gcstring(req->genicam_node.c_str()));
      CHECK_NULLPTR_RES(node, OSError_NodeError, OSERROR_STR_NODE_ERROR, res);
      
      //string name
      res->name = node->GetName().c_str();

      //string description
      res->description = node->GetDescription().c_str();

      //string Tool tip
      res->tool_tip = node->GetToolTip().c_str();

      //string display name
      res->display_name = node->GetDisplayName().c_str();

      //Polling time
      res->polling_time = node->GetPollingTime();

      //string name_space
      GenApi::ENameSpace genicam_namespace = node->GetNameSpace();
      switch(genicam_namespace)
      {
        case(GenApi::Custom): 
          res->name_space = "Custom";
          break;
        case(GenApi::Standard):  
          res->name_space= "Standard"; 
          break;
        case(GenApi::_UndefinedNameSpace): 
        default:
          res->name_space = "";
          break;
      }

      //string interface_type
      res->interface_type = GenApi::GetInterfaceName(node).c_str();

      //string access_mode
      GenApi::EAccessMode genicam_accessmode = node->GetAccessMode();
      switch(genicam_accessmode)
      {
        case(GenApi::NI):  
          res->access_mode = "Not implemented";
          break;
        case(GenApi::NA):  
          res->access_mode = "Not available";
          break;
        case(GenApi::WO):  
          res->access_mode = "Write Only";
          break;
        case(GenApi::RO):  
          res->access_mode = "Read Only";
          break;
        case(GenApi::RW):  
          res->access_mode = "Read and Write";
          break;
        case(GenApi::_CycleDetectAccesMode):
          res->access_mode = "used internally for AccessMode cycle detection"; 
          break;
        case(GenApi::_UndefinedAccesMode):
        default:
          res->access_mode = "";
          break;
      }

      //string is_cachable
      GenApi::EYesNo genicam_accessmode_cacheable = node->IsAccessModeCacheable();
      switch(genicam_accessmode_cacheable)
      {
        case(GenApi::Yes):  
          res->is_cachable = "Yes";
          break;
        case(GenApi::No):  
          res->is_cachable = "No";
          break;
        case(GenApi::_UndefinedYesNo):
        default:
          res->is_cachable = "";
          break;
      }
  
      //string visibility
      GenApi::EVisibility genicam_visibility = node->GetVisibility();
      switch(genicam_visibility)
      {
        case(GenApi::Beginner):  
          res->visibility = "Beginner";
          break;
        case(GenApi::Expert):  
          res->visibility = "Expert";
          break;
        case(GenApi::Guru):  
          res->visibility = "Guru";
          break;
        case(GenApi::Invisible):
          res->visibility = "Invisible";
          break;
        case(GenApi::_UndefinedVisibility):
        default:
          res->visibility = "";
      }

      //string caching_mode
      GenApi::ECachingMode genicam_caching_mode = node->GetCachingMode();
      switch(genicam_caching_mode)
      {
        case(GenApi::NoCache):
          res->caching_mode = "Does not use cache";
          break;
        case(GenApi::WriteThrough):  
          res->caching_mode = "Write to cache and register"; 
          break;
        case(GenApi::WriteAround):
          res->caching_mode = "Write to register, write to cache on read"; 
          break;
        case(GenApi::_UndefinedCachingMode):
        default:
          res->caching_mode = "";
      }

      //bool is_streamable
      res->is_streamable = node->IsStreamable() ? true : false;

      res->is_implemented = GenApi::IsImplemented(node);
      res->is_available = GenApi::IsAvailable(node);
      res->is_readable = GenApi::IsReadable(node);
      res->is_writable = GenApi::IsWritable(node);
      res->is_feature = node->IsFeature() ? true : false;
      if (genicam_accessmode == GenApi::RW || genicam_accessmode == GenApi::RO)
      {
        switch(node->GetPrincipalInterfaceType())
        {
          case GenApi::intfIValue:
          {
            GenApi::CValuePtr ptr(node);
            res->current_value = ptr->ToString();
            break;
          }
          case GenApi::intfIInteger:
          {
            GenApi::CIntegerPtr ptr(node);
            res->current_value = ptr->ToString();
            res->min_value = std::to_string(ptr->GetMin());
            res->max_value = std::to_string(ptr->GetMax());
            res->unit = ptr->GetUnit().c_str();
            try
            {
              res->increment = std::to_string(ptr->GetInc());
            }
            catch(...)
            {
              res->increment = "";
            }
            break;
          }
          case GenApi::intfIBoolean:
          {
            GenApi::CBooleanPtr ptr(node);
            res->current_value = ptr->ToString();
            break;
          }
          case GenApi::intfICommand:
          {
            break;
          }
          case GenApi::intfIFloat:
          {
            GenApi::CFloatPtr ptr(node);
            res->current_value = ptr->ToString();
            res->min_value = std::to_string(ptr->GetMin());
            res->max_value = std::to_string(ptr->GetMax());
            res->unit = ptr->GetUnit().c_str();
            break;
          }
          case GenApi::intfIString:
          {
            GenApi::CStringPtr ptr(node);
            res->current_value = ptr->GetValue().c_str();
            break;
          }
          case GenApi::intfIRegister:
          {
            GenApi::CRegisterPtr ptr(node);
            res->current_value = ptr->ToString();
            break;
          }
          case GenApi::intfICategory:
          {
            GenApi::CCategoryPtr ptr(node);
            res->current_value = ptr->ToString();
            GenApi::FeatureList_t features;
            ptr->GetFeatures(features);
            for (GenApi::FeatureList_t::iterator it = features.begin();
                it != features.end(); it++)
            {
              GenApi::INode *node = (*it)->GetNode();
              if (!GenApi::IsImplemented(node)) continue;
              res->child_node_list.push_back(node->GetName().c_str());
            }
            break;
          }
          case GenApi::intfIEnumeration:
          {
            GenApi::CEnumerationPtr ptr(node);
            res->current_value = ptr->ToString();
            GenApi::NodeList_t nodelist;
            ptr->GetEntries(nodelist);
            for (GenApi::NodeList_t::iterator it = nodelist.begin();
                it != nodelist.end(); it++)
            {
              if (GenApi::IsAvailable(*it))
              {
                GenApi::CEnumEntryPtr enum_entry(*it);
                res->enum_value_int_list.push_back(enum_entry->GetValue());
                res->enum_value_str_list.push_back(
                    std::string(enum_entry->GetSymbolic().c_str()));
              }
            }
            break;
          }
          case GenApi::intfIEnumEntry:
          {
            GenApi::CEnumEntryPtr ptr(node);
            res->current_value = ptr->ToString();
            break;
          }
          case GenApi::intfIPort:
          {
            break;
          }
          default:
          {
            RETURN_ERR_RES(OSError_NodeError, OSERROR_STR_NODE_ERROR, res);
          }
        }
      }
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  bool StCameraInterfaceImpl::sendSoftTriggerCallback(
      const std::shared_ptr<stcamera_msgs::srv::SendSoftTrigger_Request> req,
      std::shared_ptr<stcamera_msgs::srv::SendSoftTrigger_Response> res)
  {
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      //set selector
      GenApi::CEnumerationPtr enum_node = mp->GetNode(GenICam::gcstring("TriggerSelector"));
      *enum_node = req->trigger_selector.c_str();

      //send  
      GenApi::CCommandPtr cmd = mp->GetNode(GenICam::gcstring("TriggerSoftware"));
      cmd->Execute();
    }
    CATCH_COMMON_ERR_RES(res);
    RETURN_SUCCESS(res);
  }

  void StCameraInterfaceImpl::initializeCameraInfo()
  {  
    try
    {
      camera_info_ = camera_info_manager_.getCameraInfo();
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      // frame id
      camera_info_.header.frame_id = camera_namespace_;
      
      // binning x
      GenApi::CIntegerPtr bin_h_val(mp->GetNode(
            GenICam::gcstring("DecimationHorizontal")));
      camera_info_.binning_x = bin_h_val->GetValue();

      // binning y
      GenApi::CIntegerPtr bin_v_val(mp->GetNode(
            GenICam::gcstring("DecimationVertical")));
      camera_info_.binning_y = bin_v_val->GetValue();

      // width
      GenApi::CIntegerPtr width_max_val(mp->GetNode(
            GenICam::gcstring("WidthMax")));
      camera_info_.width = width_max_val->GetValue();

      // height
      GenApi::CIntegerPtr height_max_val(mp->GetNode(
            GenICam::gcstring("HeightMax")));
      camera_info_.height = height_max_val->GetValue();

      // ROI
      GenApi::CNodePtr roi_node(mp->GetNode(
            GenICam::gcstring("RegionSelector")));
      if (GenApi::IsWritable(roi_node))
      {
        GenApi::CEnumerationPtr roi_val(roi_node);
        GenApi::CEnumEntryPtr roi_entry(roi_val->GetEntryByName(
              GenICam::gcstring("Region0")));
        roi_val->SetIntValue(roi_entry->GetValue());
        GenApi::CIntegerPtr width(mp->GetNode(GenICam::gcstring("Width")));
        GenApi::CIntegerPtr height(mp->GetNode(GenICam::gcstring("Height")));
        GenApi::CIntegerPtr offsetx(mp->GetNode(GenICam::gcstring("OffsetX")));
        GenApi::CIntegerPtr offsety(mp->GetNode(GenICam::gcstring("OffsetY")));
        if (GenApi::IsReadable(width))
        {
          camera_info_.roi.x_offset = offsetx->GetValue();
          camera_info_.roi.y_offset = offsety->GetValue();
          if (width->GetValue() != camera_info_.width ||
              height->GetValue() != camera_info_.height)
          {
            camera_info_.roi.width = width->GetValue();
            camera_info_.roi.height = height->GetValue();
          }
        }
        camera_info_.roi.do_rectify = 
          (camera_info_.roi.width > 0 && camera_info_.roi.width < camera_info_.width) ||
          (camera_info_.roi.height > 0 && camera_info_.roi.height < camera_info_.height);
      }
      camera_info_manager_.setCameraInfo(camera_info_);
    }
    catch(...)
    {
    }
  }

  GenApi::INodeMap *StCameraInterfaceImpl::getNodeMap(const std::string &genicam_module)
  {
    if (genicam_module.compare("System") == 0)
    {
      return tl_dev_->GetIStInterface()->GetIStSystem()->GetIStPort()
        ->GetINodeMap();
    }
    if (genicam_module.compare("Interface") == 0)
    {
      return tl_dev_->GetIStInterface()->GetIStPort()->GetINodeMap();
    }
    if (genicam_module.compare("LocalDevice") == 0)
    {
      return tl_dev_->GetLocalIStPort()->GetINodeMap();
    }
    if (genicam_module.compare("RemoteDevice") == 0)
    {
      return tl_dev_->GetRemoteIStPort()->GetINodeMap();
    }
    if (genicam_module.compare("DataStream") == 0)
    {
      return tl_ds_->GetIStPort()->GetINodeMap();
    }
    return nullptr;
  }

  MapCallback *StCameraInterfaceImpl::getCallbackMap(const std::string &genicam_module)
  {
    if (genicam_module.compare("System") == 0)
    {
      return &map_event_system_;
    }
    if (genicam_module.compare("Interface") == 0)
    {
      return &map_event_interface_;
    }
    if (genicam_module.compare("LocalDevice") == 0)
    {
      return &map_event_localdevice_;
    }
    if (genicam_module.compare("RemoteDevice") == 0)
    {
      return &map_event_remotedevice_;
    }
    if (genicam_module.compare("DataStream") == 0)
    {
      return &map_event_datastream_;
    }
    return nullptr;
  }

  void StCameraInterfaceImpl::publishEventDefault(stcamera_msgs::msg::Event &msg)
  {
    std::string default_event = STMSG_event;
    MapPublisher::iterator it = map_msg_event_.find(default_event);
    if (it == map_msg_event_.end())
    {
      rclcpp::Publisher<stcamera_msgs::msg::Event>::SharedPtr pub = nh_->create_publisher<stcamera_msgs::msg::Event>(default_event, queue_size_);
      map_msg_event_.insert(std::make_pair(default_event, pub));
      map_msg_event_[default_event]->publish(msg);
    }
    else
    {
      it->second->publish(msg);
    }
  }
  void StCameraInterfaceImpl::initPublishers()
  {
    msg_chunk_ = nh_->create_publisher<stcamera_msgs::msg::Chunk>(std::string(STMSG_chunk), queue_size_);

    std::string msg_name = nh_->get_effective_namespace();
    msg_name = msg_name + "/";
    msg_name = msg_name + STMSG_image;
    it_campub_ = image_transport::create_camera_publisher(nh_.get(), msg_name);
  }
  void StCameraInterfaceImpl::initServices()
  {
    srv_read_node_ = nh_->create_service<stcamera_msgs::srv::ReadNode>(std::string(STSRV_R_node), std::bind(&StCameraInterfaceImpl::readNodeCallback, this, _1, _2));
    srv_read_node_bool_ = nh_->create_service<stcamera_msgs::srv::ReadNodeBool>(std::string(STSRV_R_node_bool), std::bind(&StCameraInterfaceImpl::readNodeBoolCallback, this, _1, _2));
    srv_read_node_enum_ = nh_->create_service<stcamera_msgs::srv::ReadNodeEnum>(std::string(STSRV_R_node_enum), std::bind(&StCameraInterfaceImpl::readNodeEnumCallback, this, _1, _2));
    srv_read_node_int_ = nh_->create_service<stcamera_msgs::srv::ReadNodeInt>(std::string(STSRV_R_node_int), std::bind(&StCameraInterfaceImpl::readNodeIntCallback, this, _1, _2));
    srv_read_node_float_ = nh_->create_service<stcamera_msgs::srv::ReadNodeFloat>(std::string(STSRV_R_node_float), std::bind(&StCameraInterfaceImpl::readNodeFloatCallback, this, _1, _2));
    srv_read_node_port_ = nh_->create_service<stcamera_msgs::srv::ReadNodePort>(std::string(STSRV_R_node_port), std::bind(&StCameraInterfaceImpl::readNodePortCallback, this, _1, _2));
    srv_read_node_register_ = nh_->create_service<stcamera_msgs::srv::ReadNodeRegister>(std::string(STSRV_R_node_register), std::bind(&StCameraInterfaceImpl::readNodeRegisterCallback, this, _1, _2));
    srv_read_node_register_info_ = nh_->create_service<stcamera_msgs::srv::ReadNodeRegisterInfo>(std::string(STSRV_R_node_register_info), std::bind(&StCameraInterfaceImpl::readNodeRegisterInfoCallback, this, _1, _2));
    srv_read_node_string_ = nh_->create_service<stcamera_msgs::srv::ReadNodeString>(std::string(STSRV_R_node_string), std::bind(&StCameraInterfaceImpl::readNodeStringCallback, this, _1, _2));
    srv_write_node_ = nh_->create_service<stcamera_msgs::srv::WriteNode>(std::string(STSRV_W_node), std::bind(&StCameraInterfaceImpl::writeNodeCallback, this, _1, _2));
    srv_write_node_bool_ = nh_->create_service<stcamera_msgs::srv::WriteNodeBool>(std::string(STSRV_W_node_bool), std::bind(&StCameraInterfaceImpl::writeNodeBoolCallback, this, _1, _2));
    srv_write_node_enum_int_ = nh_->create_service<stcamera_msgs::srv::WriteNodeEnumInt>(std::string(STSRV_W_node_enum_int), std::bind(&StCameraInterfaceImpl::writeNodeEnumIntCallback, this, _1, _2));
    srv_write_node_enum_str_ = nh_->create_service<stcamera_msgs::srv::WriteNodeEnumStr>(std::string(STSRV_W_node_enum_str), std::bind(&StCameraInterfaceImpl::writeNodeEnumStrCallback, this, _1, _2));
    srv_write_node_int_ = nh_->create_service<stcamera_msgs::srv::WriteNodeInt>(std::string(STSRV_W_node_int), std::bind(&StCameraInterfaceImpl::writeNodeIntCallback, this, _1, _2));
    srv_write_node_float_ = nh_->create_service<stcamera_msgs::srv::WriteNodeFloat>(std::string(STSRV_W_node_float), std::bind(&StCameraInterfaceImpl::writeNodeFloatCallback, this, _1, _2));
    srv_write_node_port_ = nh_->create_service<stcamera_msgs::srv::WriteNodePort>(std::string(STSRV_W_node_port), std::bind(&StCameraInterfaceImpl::writeNodePortCallback, this, _1, _2));
    srv_write_node_register_ = nh_->create_service<stcamera_msgs::srv::WriteNodeRegister>(std::string(STSRV_W_node_register), std::bind(&StCameraInterfaceImpl::writeNodeRegisterCallback, this, _1, _2));
    srv_write_node_string_ = nh_->create_service<stcamera_msgs::srv::WriteNodeString>(std::string(STSRV_W_node_string), std::bind(&StCameraInterfaceImpl::writeNodeStringCallback, this, _1, _2));
    srv_execute_node_ = nh_->create_service<stcamera_msgs::srv::ExecuteNode>(std::string(STSRV_W_node_command), std::bind(&StCameraInterfaceImpl::executeNodeCallback, this, _1, _2));
    srv_enable_chunk_ = nh_->create_service<stcamera_msgs::srv::EnableChunk>(std::string(STSRV_E_chunk), std::bind(&StCameraInterfaceImpl::enableChunkCallback, this, _1, _2));
    srv_enable_trigger_ = nh_->create_service<stcamera_msgs::srv::EnableTrigger>(std::string(STSRV_E_trigger), std::bind(&StCameraInterfaceImpl::enableTriggerCallback, this, _1, _2));
    srv_enable_event_node_ = nh_->create_service<stcamera_msgs::srv::EnableEventNode>(std::string(STSRV_E_event_node), std::bind(&StCameraInterfaceImpl::enableEventNodeCallback, this, _1, _2));
    srv_enable_image_acquisition_ = nh_->create_service<stcamera_msgs::srv::EnableImageAcquisition>(std::string(STSRV_E_image_acquisition), std::bind(&StCameraInterfaceImpl::enableImageAcquisitionCallback, this, _1, _2));
    srv_enable_event_acquisition_ = nh_->create_service<stcamera_msgs::srv::EnableEventAcquisition>(std::string(STSRV_E_event_acquisition), std::bind(&StCameraInterfaceImpl::enableEventAcquisitionCallback, this, _1, _2));
    srv_get_image_acquisition_status_ = nh_->create_service<stcamera_msgs::srv::GetImageAcquisitionStatus>(std::string(STSRV_G_image_acquisition_status), std::bind(&StCameraInterfaceImpl::getImageAcquisitionStatusCallback, this, _1, _2));
    srv_get_event_acquisition_status_list_ = nh_->create_service<stcamera_msgs::srv::GetEventAcquisitionStatusList>(std::string(STSRV_G_event_acquisition_status_list), std::bind(&StCameraInterfaceImpl::getEventAcquisitionStatusListCallback, this, _1, _2));
    srv_get_event_node_status_list_ = nh_->create_service<stcamera_msgs::srv::GetEventNodeStatusList>(std::string(STSRV_G_event_node_status_list), std::bind(&StCameraInterfaceImpl::getEventNodeStatusListCallback, this, _1, _2));
    srv_get_chunk_list_ = nh_->create_service<stcamera_msgs::srv::GetChunkList>(std::string(STSRV_G_chunk_list), std::bind(&StCameraInterfaceImpl::getChunkListCallback, this, _1, _2));
    srv_get_trigger_list_ = nh_->create_service<stcamera_msgs::srv::GetTriggerList>(std::string(STSRV_G_trigger_list), std::bind(&StCameraInterfaceImpl::getTriggerListCallback, this, _1, _2));
    srv_get_enum_list_ = nh_->create_service<stcamera_msgs::srv::GetEnumList>(std::string(STSRV_G_enum_list), std::bind(&StCameraInterfaceImpl::getEnumListCallback, this, _1, _2));
    srv_get_genicam_node_info_ = nh_->create_service<stcamera_msgs::srv::GetGenICamNodeInfo>(std::string(STSRV_G_genicam_node_info), std::bind(&StCameraInterfaceImpl::getGenICamNodeInfoCallback, this, _1, _2));
    srv_send_soft_trigger_ = nh_->create_service<stcamera_msgs::srv::SendSoftTrigger>(std::string(STSRV_send_soft_trigger), std::bind(&StCameraInterfaceImpl::sendSoftTriggerCallback, this, _1, _2));
  }
} // end of namespace stcamera

