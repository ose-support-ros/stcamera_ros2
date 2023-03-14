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
#include "../include/stcamera.hpp"
#include <iomanip>

#define TRIGGER_INTERVAL_MS 100
#define TRIGGER_SELECTOR "FrameStart"
#define TRIGGER_SOURCE "Software"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace stcamera
{
    CStCamera::CStCamera(const std::string & camera_name_space) : 
      CStCameraBase(camera_name_space), grabbed_images_(0)
      {

      }
    CStCamera::~CStCamera()
    {

    }

    void CStCamera::softtrigger_timer_callback()
    {
      std::cout << std::endl << "Sending trigger " << std::dec << ++sent_triggers_ << ". Press Ctrl+C to stop sending trigger." << std::endl;
      sendSoftTrigger(node_for_trigger_, TRIGGER_SELECTOR);
    }

    void CStCamera::enabledAllChunks()
    {
      std::vector<std::string> chunkNameList;
      std::vector<bool> chunkEnabledList;
      EOSError_t eError = getChunkList(chunkNameList, chunkEnabledList);
      if(eError != OSError_Success) return;

      for(size_t i = 0; i < chunkNameList.size(); ++i)
      {
        if(!chunkEnabledList[i])
        {
          eError = enableChunk(chunkNameList[i], true);
          if(eError != OSError_Success) return;
        }
      }
    }
    void CStCamera::enabledAllEvents()
    {
      std::vector<std::string> eventNameList;
      std::vector<std::string> eventNodeNameList;
      std::vector<int64_t> eventValueList;
      EOSError_t eError = getEnumList("RemoteDevice", "EventSelector", eventNameList, eventNodeNameList, eventValueList);
      if(eError != OSError_Success) return;

      for(size_t i = 0; i < eventNameList.size(); ++i)
      {
        const std::string eventCallbackNodeName = "Event" + eventNameList[i];
        eError = enableEventNode("RemoteDevice", eventNameList[i], eventCallbackNodeName, true);
        if(eError != OSError_Success) return;
      }

      displayEventNodeStatusList("RemoteDevice");

      eError = enableEventAcquisition("RemoteDevice", true);
      if(eError != OSError_Success)
      {
        std::cerr << "enableEventAcquisition(\"RemoteDevice\", true) returns " << eError << "." << std::endl;
      }

      displayEventAcquisitionStatusList();
    }
    void CStCamera::executeNodeTest()
    {
      int64_t timestamp;
      EOSError_t eError = readNodeInt("RemoteDevice", "TimestampLatchValue", timestamp);
      if (eError == OSError_Success)
      {
        std::cout << "Default TimestampLatchValue: " << timestamp << std::endl;
      }
          
      eError = executeNode("RemoteDevice", "TimestampLatch");
      if (eError == OSError_Success)
      {
        std::cout << "TimestampLatch executed." << std::endl;
      }

      eError = readNodeInt("RemoteDevice", "TimestampLatchValue", timestamp);
      if (eError == OSError_Success)
      {
        std::cout << "Current TimestampLatchValue: " << timestamp << std::endl;
      }
    }
    void CStCamera::displayEventNodeStatusList(const std::string &genicam_module, const std::string &prefix)
    {
      std::vector<stcamera_msgs::msg::GenICamEvent> event_node_list;
      EOSError_t eError = getEventNodeStatusList(genicam_module, event_node_list);
      
      if(eError == OSError_Success)
      {
        std::cout << prefix << "event node status list" << std::endl;
        for(size_t i = 0; i < event_node_list.size(); ++i)
        {
          stcamera_msgs::msg::GenICamEvent &event_node = event_node_list[i];
          std::cout << prefix << "\tevent[" << i << "] " << event_node.name << (event_node.enabled ? " : true" : " : false") << std::endl;
          for(size_t j = 0; j < event_node.callback_node_list.size(); ++j)
          {
            std::cout << prefix << "\t\tcallback[" << j << "] " << event_node.callback_node_list[j] << (event_node.callback_enabled_list[j] ? " : true" : " : false") << std::endl;
          }
        }
      }
    }
    void CStCamera::displayEventAcquisitionStatusList(const std::string &prefix)
    {
      std::vector<std::string> genicam_module_list;
      std::vector<bool> enabled_list;
      EOSError_t eError = getEventAcquisitionStatusList(genicam_module_list, enabled_list);
      if(eError == OSError_Success)
      {
        std::cout << prefix << "event acquisition status list" << std::endl;
        for(size_t i = 0; i < genicam_module_list.size(); ++i)
        {
          std::cout << prefix << "\t" << genicam_module_list[i] << (enabled_list[i] ? " : true" : " : false") << std::endl;
        }
      }
    }

    void CStCamera::ControlNodeFloatTest(const std::string &genicam_node, const std::string &prefix)
    {
      const std::string genicam_module = "RemoteDevice";
      std::cout << prefix << "ControlNodeFloatTest(" << genicam_node.c_str() << ")" << std::endl;
      double value_list[3] = {0};  //min, max, current

      // Read min/max values
      EOSError_t eError = readNodeFloatRange(genicam_module, genicam_node, value_list[0], value_list[1]);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tRange of " << genicam_node.c_str() << ": " << value_list[0] << " - " << value_list[1] << std::endl;
      }

      // Read current value
      eError = readNodeFloat(genicam_module, genicam_node, value_list[2]);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tCurrent " << genicam_node.c_str() << ": " << value_list[2] << std::endl;
      }

      for(size_t i = 0; i < sizeof(value_list) / sizeof(value_list[0]); ++i)
      {
        // Change value 
        if(i == 0)
        {
          eError = writeNodeFloat(genicam_module, genicam_node, value_list[i]);
        }
        else
        {
          eError = writeNode(genicam_module, genicam_node, std::to_string(value_list[i]));
        }
        if (eError == OSError_Success)
        {
          std::cout << prefix << "\tUpdates " << genicam_node.c_str() << " to " << value_list[i] << "." << std::endl;
        }

        // Read value
        double value;
        eError = readNodeFloat(genicam_module, genicam_node, value);
        if (eError == OSError_Success)
        {
          if(value_list[i] == value)
          {
            std::cout << prefix << "\tUpdate was successful." << std::endl;
          }
          else
          {
            std::cerr << prefix << "\tUpdate was failed." << std::endl;
            std::cerr << prefix << "\tCurrent " << genicam_node.c_str() << ": " << value << std::endl;
          }
        }

      }
    }
    void CStCamera::ControlNodeIntTest(const std::string &genicam_node, const std::string &prefix)
    {
      const std::string genicam_module = "RemoteDevice";
      std::cout << prefix << "ControlNodeIntTest(" << genicam_node.c_str() << ")" << std::endl;
      int64_t value_list[3] = {0};  //min, max, current

      // Read min/max values
      EOSError_t eError = readNodeIntRange(genicam_module, genicam_node, value_list[0], value_list[1]);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tRange of " << genicam_node.c_str() << ": " << value_list[0] << " - " << value_list[1] << std::endl;
      }

      // Read current value
      eError = readNodeInt(genicam_module, genicam_node, value_list[2]);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tCurrent " << genicam_node.c_str() << ": " << value_list[2] << std::endl;
      }

      for(size_t i = 0; i < sizeof(value_list) / sizeof(value_list[0]); ++i)
      {
        // Change value 
        if(i == 0)
        {
          eError = writeNodeInt(genicam_module, genicam_node, value_list[i]);
        }
        else
        {
          eError = writeNode(genicam_module, genicam_node, std::to_string(value_list[i]));
        }
        if (eError == OSError_Success)
        {
          std::cout << prefix << "\tUpdates " << genicam_node.c_str() << " to " << value_list[i] << "." << std::endl;
        }

        // Read value
        int64_t value;
        eError = readNodeInt(genicam_module, genicam_node, value);
        if (eError == OSError_Success)
        {
          if(value_list[i] == value)
          {
            std::cout << prefix << "\tUpdate was successful." << std::endl;
          }
          else
          {
            std::cerr << prefix << "\tUpdate was failed." << std::endl;
            std::cerr << prefix << "\tCurrent " << genicam_node.c_str() << ": " << value << std::endl;
          }
        }

      }
    }
    void CStCamera::ControlNodeEnumTest(const std::string &genicam_node, const std::string &prefix)
    {
      const std::string genicam_module = "RemoteDevice";
      std::cout << prefix << "ControlNodeEnumTest(" << genicam_node.c_str() << ")" << std::endl;

      std::vector<std::string> enum_value_str_list;
      std::vector<std::string> enum_node_list;
      std::vector<int64_t> enum_value_int_list;
      EOSError_t eError = getEnumList(genicam_module, genicam_node, enum_value_str_list, enum_node_list, enum_value_int_list);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tGot enum list." << std::endl;
      }
      
      std::string current_value_str;
      int64_t current_value_int;
      eError = readNodeEnum(genicam_module, genicam_node, current_value_str, current_value_int);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tCurrent " << genicam_node.c_str() << ": " << current_value_str.c_str() << "(" << current_value_int << ")" << std::endl;
      }

      //writeNodeEnumInt
      for(size_t i = 0; i < enum_value_int_list.size(); ++i)
      {
        eError = writeNodeEnumInt(genicam_module, genicam_node, enum_value_int_list[i]);
        if (eError == OSError_Success)
        {
          std::cout << prefix << "\tUpdates " << genicam_node.c_str() << " to " << enum_value_int_list[i] << "." << std::endl;
        }

        std::string value_str;
        int64_t value_int;
        eError = readNodeEnum(genicam_module, genicam_node, value_str, value_int);
        if (eError == OSError_Success)
        {
          if((enum_value_int_list[i] == value_int) && (enum_value_str_list[i].compare(value_str) == 0))
          {
            std::cout << prefix << "\tUpdate was successful." << std::endl;
          }
          else
          {
            std::cerr << prefix << "\tUpdate was failed." << std::endl;
            std::cerr << prefix << "\tCurrent " << genicam_node.c_str() << ": " << value_int << std::endl;
          }
        }
      }

      //writeNodeEnumStr
      for(size_t i = 0; i < enum_value_int_list.size(); ++i)
      {
        if(i == 0)
        {
          eError = writeNodeEnumStr(genicam_module, genicam_node, enum_value_str_list[i]);
        }
        else
        {
          eError = writeNode(genicam_module, genicam_node, enum_value_str_list[i]);
        }
        if (eError == OSError_Success)
        {
          std::cout << prefix << "\tUpdates " << genicam_node.c_str() << " to " << enum_value_str_list[i].c_str() << "." << std::endl;
        }

        std::string value_str;
        int64_t value_int;
        eError = readNodeEnum(genicam_module, genicam_node, value_str, value_int);
        if (eError == OSError_Success)
        {
          if((enum_value_int_list[i] == value_int) && (enum_value_str_list[i].compare(value_str) == 0))
          {
            std::cout << prefix << "\tUpdate was successful." << std::endl;
          }
          else
          {
            std::cerr << prefix << "\tUpdate was failed." << std::endl;
            std::cerr << prefix << "\tCurrent " << genicam_node.c_str() << ": " << value_str << std::endl;
          }
        }
      }
      eError = writeNodeEnumInt(genicam_module, genicam_node, current_value_int);
    }
    void CStCamera::ControlNodeBoolTest(const std::string &genicam_node, const std::string &prefix)
    {
      const std::string genicam_module = "RemoteDevice";
      std::cout << prefix << "ControlNodeBoolTest(" << genicam_node.c_str() << ")" << std::endl;
      bool current_value;

      // Read current value
      EOSError_t eError = readNodeBool(genicam_module, genicam_node, current_value);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tCurrent " << genicam_node.c_str() << ": " << (current_value ? "true" : "false") << std::endl;
      }

      for(size_t i = 0; i < 2; ++i)
      {
        const bool target_value = (i == 0) ? !current_value : current_value;

        // Change value 
        if(i == 0)
        {
          eError = writeNodeBool(genicam_module, genicam_node, target_value);
        }
        else
        {
          eError = writeNode(genicam_module, genicam_node, target_value ? "tRuE" : "FaLsE");
        }
        if (eError == OSError_Success)
        {
          std::cout << prefix << "\tUpdates " << genicam_node.c_str() << " to " << (target_value ? "true" : "false") << "." << std::endl;
        }

        // Read value
        bool value;
        eError = readNodeBool(genicam_module, genicam_node, value);
        if (eError == OSError_Success)
        {
          if(target_value == value)
          {
            std::cout << prefix << "\tUpdate was successful." << std::endl;
          }
          else
          {
            std::cerr << prefix << "\tUpdate was failed." << std::endl;
            std::cerr << prefix << "\tCurrent " << genicam_node.c_str() << ": " << (value ? "true" : "false") << std::endl;
          }
        }
      }
    }
    void CStCamera::ControlNodeStringTest(const std::string &genicam_node, const std::string &prefix)
    {
      const std::string genicam_module = "RemoteDevice";
      std::cout << prefix << "ControlNodeStringTest(" << genicam_node.c_str() << ")" << std::endl;
      std::string current_value;

      // Read current value
      EOSError_t eError = readNodeString(genicam_module, genicam_node, current_value);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tCurrent " << genicam_node.c_str() << ": " << current_value.c_str() << std::endl;
      }

      for(size_t i = 0; i < 2; ++i)
      {
        const std::string target_value = (i == 0) ? current_value + "TEST" : current_value;

        // Change value 
        if(i == 0)
        {
          eError = writeNodeString(genicam_module, genicam_node, target_value);
        }
        else
        {
          eError = writeNode(genicam_module, genicam_node, target_value);
        }
        if (eError == OSError_Success)
        {
          std::cout << prefix << "\tUpdates " << genicam_node.c_str() << " to " << target_value.c_str() << "." << std::endl;
        }

        // Read value
        std::string value;
        eError = readNodeString(genicam_module, genicam_node, value);
        if (eError == OSError_Success)
        {
          if(target_value.compare(value) == 0)
          {
            std::cout << prefix << "\tUpdate was successful." << std::endl;
          }
          else
          {
            std::cerr << prefix << "\tUpdate was failed." << std::endl;
            std::cerr << prefix << "\tCurrent " << genicam_node.c_str() << ": " << value.c_str() << std::endl;
          }
        }
      }
    }
    void CStCamera::printBuffer(const std::vector<uint8_t> &buffer, size_t max_print_len, const std::string &prefix)
    {
      if(buffer.size() < max_print_len)
      {
        max_print_len = buffer.size();
      }
      for(size_t n = 0; n < max_print_len; ++n)
      {
        if((n % 16) == 0)
        {
          std::cout << std::endl << prefix;
        }
        std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int16_t)buffer[n];
      }
      std::cout  << std::dec << std::endl;
    }
    void CStCamera::ControlNodeRegisterAndPortTest(const std::string &genicam_node, const std::string &prefix)
    {
      const std::string genicam_module = "RemoteDevice";
      const std::string genicam_port = "Device";
      std::cout << prefix << "ControlNodeRegisterAndPortTest(" << genicam_node.c_str() << ")" << std::endl;

      int64_t address;
      int64_t length;
      EOSError_t eError = readNodeRegisterInfo(genicam_module, genicam_node, address, length);
      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tAdddress: " << address << ", Length: " << length << std::endl;
      }

      std::vector<uint8_t> current_value;
      std::vector<uint8_t> changed_value;
      changed_value.resize(length);

      // Read current value
      eError = readNodeRegister(genicam_module, genicam_node, length, current_value);
      const size_t max_print_len = (size_t)((1024 < length) ? 1024 : length);

      if (eError == OSError_Success)
      {
        std::cout << prefix << "\tCurrent data:";
        printBuffer(current_value, max_print_len, prefix + "\t\t");
      }

      for(size_t i = 0; i < current_value.size(); ++i)
      {
        changed_value[i] = 0xFF ^ current_value[i];
      }

      for(size_t i = 0; i < 2; ++i)
      {
        const std::vector<uint8_t> &target_value = (i == 0) ? changed_value : current_value;
        
        // Change value
        if(i == 0)
        {
          eError = writeNodePort(genicam_module, genicam_port, address, target_value.size(), target_value);
        }
        else
        {
          eError = writeNodeRegister(genicam_module, genicam_node, target_value.size(), target_value);
        }
        if (eError == OSError_Success)
        {
          std::cout << prefix << "\tUpdates " << genicam_node.c_str() << " to " << std::endl;

          std::cout << prefix << "\tdata:";
          printBuffer(target_value, max_print_len, prefix + "\t\t");
        }

        // Read value
        std::vector<uint8_t> value;
        if(i == 0)
        {
          eError = readNodePort(genicam_module, genicam_port, address, length, value);
        }
        else
        {
          eError = readNodeRegister(genicam_module, genicam_node, length, value);
        }
        if (eError == OSError_Success)
        {
          if(memcmp(&target_value[0], &value[0], length) == 0)
          {
            std::cout << prefix << "\tUpdate was successful." << std::endl;
          }
          else
          {
            std::cerr << prefix << "\tUpdate was failed." << std::endl;
            std::cerr << prefix << "\tdata:";
            printBuffer(value, max_print_len, prefix + "\t\t");
          }
        }
      }

    }
    void CStCamera::displayGenICamNodeInfo(const std::string &genicam_module, const std::string &genicam_node, const std::string &prefix)
    {
      stcamera_msgs::srv::GetGenICamNodeInfo::Response res;
      EOSError_t eError = getGenICamNodeInfo(genicam_module, genicam_node, res);
      
      if(eError == OSError_Success)
      {
        std::cout << prefix << genicam_node << std::endl;
        std::cout << prefix << "\tname:" << res.name << std::endl;
        std::cout << prefix << "\tdisplay_name:" << res.display_name << std::endl;
        std::cout << prefix << "\tdescription:" << res.description << std::endl;
        std::cout << prefix << "\ttool_tip:" << res.tool_tip << std::endl;
        std::cout << prefix << "\tname_space:" << res.name_space << std::endl;
        std::cout << prefix << "\tinterface_type:" << res.interface_type << std::endl;
        std::cout << prefix << "\taccess_mode:" << res.access_mode << std::endl;
        std::cout << prefix << "\tis_cachable:" << res.is_cachable << std::endl;
        std::cout << prefix << "\tpolling_time:" << res.polling_time << std::endl;
        std::cout << prefix << "\tvisibility:" << res.visibility << std::endl;
        std::cout << prefix << "\tcaching_mode:" << res.caching_mode << std::endl;
        std::cout << prefix << "\tis_streamable:" << res.is_streamable << std::endl;
        std::cout << prefix << "\tcurrent_value:" << res.current_value << std::endl;
        std::cout << prefix << "\tmin_value:" << res.min_value << std::endl;
        std::cout << prefix << "\tmax_value:" << res.max_value << std::endl;
        std::cout << prefix << "\tincrement:" << res.increment << std::endl;
        std::cout << prefix << "\tunit:" << res.unit << std::endl;
        std::cout << prefix << "\tchild_node_list:" << res.child_node_list.size() << std::endl;
        for(size_t i = 0; i < res.child_node_list.size(); ++i)
        {
          std::cout << prefix << "\t\tchild[" << i << "]:" << res.child_node_list[i] << std::endl;
        }
        std::cout << prefix << "\tis_implemented:" << res.is_implemented << std::endl;
        std::cout << prefix << "\tis_available:" << res.is_available << std::endl;
        std::cout << prefix << "\tis_readable:" << res.is_readable << std::endl;
        std::cout << prefix << "\tis_writable:" << res.is_writable << std::endl;
        std::cout << prefix << "\tis_feature:" << res.is_feature << std::endl;

        std::cout << prefix << "\tenum_value_str_list:" << res.enum_value_str_list.size() << std::endl;
        std::cout << prefix << "\tenum_value_int_list:" << res.enum_value_int_list.size() << std::endl;
        if(0 < res.enum_value_str_list.size())
        {
          std::vector<std::string>  enum_value_str_list;
          std::vector<std::string>  enum_node_list;
          std::vector<int64_t>  enum_value_int_list;
          getEnumList(genicam_module, genicam_node, enum_value_str_list, enum_node_list, enum_value_int_list);
          for(size_t i = 0; i < enum_value_int_list.size(); ++i)
          {
            std::cout << prefix << "\t\tEntry[" << i << "]:" 
              << enum_value_str_list[i] 
              << "[" << enum_node_list[i] << "]"
              << "(" << enum_value_int_list[i] << ")" << std::endl;
            displayGenICamNodeInfo(genicam_module, enum_node_list[i], prefix + "\t\t\t");
          }
        }
      }
    }
    void CStCamera::displayBootstrap(const std::string &device_tl_type)
    {
      const std::string &genicam_module = "RemoteDevice";
      const std::string &genicam_node = "Device";
      const std::string &prefix = "";
        std::cout << prefix << "Bootstrap(" << device_tl_type << ")" << std::endl;
      if(device_tl_type.compare("U3V") == 0)
      {
        std::vector<uint8_t> data;
        EOSError_t eError = readNodePort(genicam_module, genicam_node, 0x0000, 4, data);
        if(eError != OSError_Success) return;
        std::cout << prefix << "\tGenCP Version: " << (((uint16_t)data[3] << 8) + data[2]) << "." << (((uint16_t)data[1] << 8) + data[0]) << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x0004, 64, data);
        if(eError != OSError_Success) return;
        data.resize(65);
        data[64] = 0;
        std::cout << prefix << "\tManufacturer Name: " << (char*)&data[0] << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x0044, 64, data);
        if(eError != OSError_Success) return;
        data.resize(65);
        data[64] = 0;
        std::cout << prefix << "\tModel Name: " << (char*)&data[0] << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x0084, 64, data);
        if(eError != OSError_Success) return;
        data.resize(65);
        data[64] = 0;
        std::cout << prefix << "\tFamily Name: " << (char*)&data[0] << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x00C4, 64, data);
        if(eError != OSError_Success) return;
        data.resize(65);
        data[64] = 0;
        std::cout << prefix << "\tDevice Version: " << (char*)&data[0] << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x0104, 64, data);
        if(eError != OSError_Success) return;
        data.resize(65);
        data[64] = 0;
        std::cout << prefix << "\tManufacturer Info: " << (char*)&data[0] << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x0144, 64, data);
        if(eError != OSError_Success) return;
        data.resize(65);
        data[64] = 0;
        std::cout << prefix << "\tSerial Number: " << (char*)&data[0] << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x0184, 64, data);
        if(eError != OSError_Success) return;
        data.resize(65);
        data[64] = 0;
        std::cout << prefix << "\tUser Defined Name: " << (char*)&data[0] << std::endl;
      }
      else if(device_tl_type.compare("GEV") == 0)
      {
        std::vector<uint8_t> data;
        EOSError_t eError = readNodePort(genicam_module, genicam_node, 0x0000, 4, data);
        if(eError != OSError_Success) return;
        std::cout << prefix << "\tVersion: " << (((uint16_t)data[0] << 8) + data[1]) << "." << (((uint16_t)data[2] << 8) + data[3]) << std::endl;

        eError = readNodePort(genicam_module, genicam_node, 0x0004, 4, data);
        if(eError != OSError_Success) return;
        std::cout << prefix << "\tDevice Mode" << std::endl;
        std::cout << prefix << "\t\tendianess: " << ((data[0] & 0x80) ? "big-endian" : "reserved") << std::endl;
        const char *device_class_list[] = {"Transmitter", "Receiver", "Transceiver", "Peripheral", "reserved"};
        uint16_t device_class_index = (data[0] >> 4) & 0x7;
        if(4 < device_class_index) device_class_index = 4;
        std::cout << prefix << "\t\tdevice_class: " << device_class_list[device_class_index] << "(" << device_class_index << ")" << std::endl;
        
      }
    }
    void CStCamera::displayGenICamNodeTree(const std::string &genicam_module, const std::string &genicam_node, const std::string &prefix)
    {
      stcamera_msgs::srv::GetGenICamNodeInfo::Response res;
      EOSError_t eError = getGenICamNodeInfo(genicam_module, genicam_node, res);
      if(eError == OSError_Success)
      {
        if(res.is_implemented)
        {
          std::cout << prefix << genicam_node << "(" << res.interface_type << ")" << std::endl;
          if(res.interface_type.compare("ICategory") == 0)
          {
            for(size_t i = 0; i < res.child_node_list.size(); ++i)
            {
              displayGenICamNodeTree(genicam_module, res.child_node_list[i], prefix + "\t");
            }
          }
          else if(!res.is_available)
          {
            std::cout << prefix << "\t" << "**** not available"<< std::endl;
          }
          else if(!res.is_readable)
          {
            std::cout << prefix << "\t" << "**** not readable"<< std::endl;
          }
          else if(res.interface_type.compare("IEnumeration") == 0)
          {
            std::string value_str;
            int64_t value_int;
            eError = readNodeEnum(genicam_module, genicam_node, value_str, value_int);
            if(eError == OSError_Success)
            {
              for(size_t i = 0; i < res.enum_value_str_list.size(); ++i)
              {
                const bool is_current = (res.enum_value_int_list[i] == value_int);
                if(is_current)
                {
                  if(value_str.compare(res.enum_value_str_list[i]) != 0)
                  {
                    std::cout << prefix << "\t" << "**** Invalid Current Value:" << value_str << std::endl;
                  }
                }
                std::cout << prefix << "\tEntry[" << i << "]:" << res.enum_value_str_list[i] << "(" << res.enum_value_int_list[i] << (is_current ? ") *current" : ")") << std::endl;
              }
            }
          }
          else if(res.interface_type.compare("IString") == 0)
          {
            std::string value;
            eError = readNodeString(genicam_module, genicam_node, value);
            if(eError == OSError_Success)
            {
              std::cout << prefix << "\tvalue:" << value << std::endl;
            }
          }
          else if(res.interface_type.compare("IInteger") == 0)
          {
            int64_t value;
            eError = readNodeInt(genicam_module, genicam_node, value);
            if(eError == OSError_Success)
            {
              std::cout << prefix << "\tvalue:" << value << std::endl;
            }
          }
          else if(res.interface_type.compare("IFloat") == 0)
          {
            double value;
            eError = readNodeFloat(genicam_module, genicam_node, value);
            if(eError == OSError_Success)
            {
              std::cout << prefix << "\tvalue:" << value << std::endl;
            }
          }
          else if(res.interface_type.compare("IBoolean") == 0)
          {
            bool value;
            eError = readNodeBool(genicam_module, genicam_node, value);
            if(eError == OSError_Success)
            {
              std::cout << prefix << "\tvalue:" << (value ? "true" : "false") << std::endl;
            }
          }
          else if(res.interface_type.compare("IRegister") == 0)
          {
            int64_t address;
            int64_t length;
            eError = readNodeRegisterInfo(genicam_module, genicam_node, address, length);
            if(eError == OSError_Success)
            {
              std::cout << prefix << "\taddress:" << address << std::endl;
              std::cout << prefix << "\tlength:" << length << std::endl;
              int64_t readlen = (32 < length) ? 32 : length;
              std::vector<uint8_t> data;
              eError = readNodeRegister(genicam_module, genicam_node, readlen, data);
              if(eError == OSError_Success)
              {
                std::cout << prefix << "\tdata:";
                printBuffer(data, readlen, prefix);
              }

            }
          }
          else
          {
            std::string value;
            std::string interface_type;
            eError = readNode(genicam_module, genicam_node, value, interface_type);
            if(eError == OSError_Success)
            {
              std::cout << prefix << "\tvalue:" << value << std::endl;
              std::cout << prefix << "\tinterface_type:" << interface_type << std::endl;
            }
          }
        }
      }


    }

    void CStCamera::displayTriggerList(const std::string &prefix)
    {
      std::vector<std::string> trigger_selector_list;
      std::vector<std::string> trigger_source_list;
      std::vector<double> trigger_delayus_list;
      std::vector<bool> trigger_mode_list;
      EOSError_t eError = getTriggerList(trigger_selector_list, trigger_mode_list, trigger_source_list, trigger_delayus_list);
      
      if(eError == OSError_Success)
      {
        for(size_t i = 0; i < trigger_selector_list.size(); ++i)
        {
          std::cout << prefix << "Trigger Selector[" << i << "]:" << trigger_selector_list[i].c_str() << std::endl;
          std::cout << prefix << "\tmode:" << (trigger_mode_list[i] ? "true" : "false") << std::endl;
          std::cout << prefix << "\tsource:" << trigger_source_list[i].c_str() << std::endl;
          std::cout << prefix << "\tdelay[us]:" << trigger_delayus_list[i] << std::endl;
        }
      }
    }
    void CStCamera::start_free_run_mode()
    {
      // Sets the camera to free run mode
      setImageAcquisition(false);
      enableTrigger(TRIGGER_SELECTOR, TRIGGER_SOURCE, false);

      displayTriggerList();

      grabbed_images_ = 0;

      // Free run mode
      std::cout << std::endl << "Free run mode" << std::endl;
      setImageAcquisition(true);
    }

    void CStCamera::start_trigger_mode()
    {
      // Sets the camera to trigger mode
      setImageAcquisition(false);
      enableTrigger(TRIGGER_SELECTOR, TRIGGER_SOURCE, true);

      displayTriggerList();

      grabbed_images_ = 0;

      // Trigger mode
      std::cout << std::endl << "Enable trigger selector: " << TRIGGER_SELECTOR <<
        ", source: " << TRIGGER_SOURCE << std::endl;
      try
      {
        setImageAcquisition(true);

        trigger_timer_ = create_wall_timer(100ms, std::bind(&CStCamera::softtrigger_timer_callback, this));
        node_for_trigger_ = rclcpp::Node::make_shared("trigger");
      }
      catch(...)
      {
      }
    }
    void CStCamera::stop_trigger_mode()
    {
      setImageAcquisition(false);
      enableTrigger(TRIGGER_SELECTOR, TRIGGER_SOURCE, false);
      setImageAcquisition(true);
    }


    void CStCamera::onRcvImage(sensor_msgs::msg::Image::SharedPtr msg)
    {
      ++grabbed_images_;

      std::cout  << std::dec << 
        "Grabbed " << grabbed_images_ << 
        " frame_id: " << msg->header.frame_id << 
        " stamp = " << msg->header.stamp.sec << "." << std::setfill('0') << std::right << std::setw(9) << msg->header.stamp.nanosec << 
        " images. WxH: " << msg->width << " x " << msg->height << 
        " encoding: " << msg->encoding << 
        " step: " << msg->step << 
        std::endl;
      std::cout << "Press Ctrl + C to exit the image acquisition loop." << std::endl;
    }
    void CStCamera::onRcvChunk(stcamera_msgs::msg::Chunk::SharedPtr msg)
    {
      std::cout  << std::dec << 
        "Chunk " << msg->timestamp.sec << "." << std::setfill('0') << std::right << std::setw(9) << msg->timestamp.nanosec << std::endl;
      for(size_t i = 0; i < msg->float64_list.size(); ++i)
      {
        std::cout << "\tfloat64[" << i << "] " << msg->float64_list[i].name << " = " << msg->float64_list[i].value << std::endl;
      }
      for(size_t i = 0; i < msg->int64_list.size(); ++i)
      {
        std::cout << "\tint64[" << i << "] " << msg->int64_list[i].name << " = " << msg->int64_list[i].value << std::endl;
      }
      for(size_t i = 0; i < msg->bool_list.size(); ++i)
      {
        std::cout << "\tbool[" << i << "] " << msg->bool_list[i].name << " = " << (msg->bool_list[i].value ? "true" : "false") << std::endl;
      }
      for(size_t i = 0; i < msg->string_list.size(); ++i)
      {
        std::cout << "\tstring[" << i << "] " << msg->string_list[i].name << " = " << msg->string_list[i].value.c_str() << std::endl;
      }
    }
    void CStCamera::onRcvEvent(stcamera_msgs::msg::Event::SharedPtr msg)
    {
      std::cout << "Event " << msg->genicam_module << " " << msg->event_name << " " << msg->callback_node << " " << msg->event_data << std::endl;
    }
}
