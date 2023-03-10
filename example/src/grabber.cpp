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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "../include/stcamera.hpp"
#include "../include/stmanager.hpp"

#define NODE_NAMESPACE_NAME "/stcamera_launcher"
#define NODE_NAME "/stcameras"

using namespace stcamera;

typedef struct mode_list_t
{
  bool enableTriggerMode;
  bool enableDisplayBootstrap;
  bool enableDisplaySDKInfo;
  bool enableDisplayDeviceList;
  bool enableDisplayModuleList;
  bool enableDisplayGenICamNodeTree;
  bool enableDisplayGenICamInfo;
  bool enabledAllChunks;
  bool enabledAllEvents;
  bool enabledExecuteNodeTest;
  bool enabledWriteNodeIntTest;
  bool enabledWriteNodeFloatTest;
  bool enabledWriteNodeEnumTest;
  bool enabledWriteNodeBoolTest;
  bool enabledWriteNodeStringTest;
  bool enabledWriteNodeRegisterAndPortTest;
}mode_list_t;

void startGrab(const mode_list_t &modeList)
{
    auto grabber = std::make_shared<CStManager>(NODE_NAMESPACE_NAME, NODE_NAME);

    // Display SDK info
    if(modeList.enableDisplaySDKInfo)
    {
      grabber->displaySDKInfo();
    }

    // Display module list
    if(modeList.enableDisplayModuleList)
    {
      grabber->displayModuleList();
    }

    // Display device list
    if(modeList.enableDisplayDeviceList)
    {
      grabber->displayDeviceList();
    }

    // Check connected devices
    std::string device_tltype;
    std::string device_name_space = grabber->getConnectedDeviceNameSpace(device_tltype);
    if(device_name_space.empty()) return;
      
    auto camera_node = std::make_shared<CStCamera>(device_name_space);
    camera_node->init();

    camera_node->setImageAcquisition(false);

    if(modeList.enableDisplayBootstrap)
    {
      camera_node->displayBootstrap(device_tltype);
    }
    if(modeList.enableDisplayGenICamInfo)
    {
      camera_node->displayGenICamNodeInfo("RemoteDevice", "PixelFormat");
    }
    if(modeList.enableDisplayGenICamNodeTree)
    {
      //camera_node->displayGenICamNodeTree("Interface", "Root");
      camera_node->displayGenICamNodeTree("RemoteDevice", "Root");
    }

    if(modeList.enabledAllChunks)
    {
      camera_node->enabledAllChunks(); 
    }
    if(modeList.enabledAllEvents)
    {
      camera_node->enabledAllEvents(); 
    }
    if(modeList.enabledExecuteNodeTest)
    {
      camera_node->executeNodeTest();
    }
    if(modeList.enabledWriteNodeIntTest)
    {
      camera_node->ControlNodeIntTest("Width");
    }

    if(modeList.enabledWriteNodeFloatTest)
    {
      camera_node->ControlNodeFloatTest("Gain");
    }
    if(modeList.enabledWriteNodeEnumTest)
    {
      camera_node->ControlNodeEnumTest("PixelFormat");
    }
    if(modeList.enabledWriteNodeBoolTest)
    {
      camera_node->ControlNodeBoolTest("ReverseY");
    }
    if(modeList.enabledWriteNodeStringTest)
    {
      camera_node->ControlNodeStringTest("DeviceUserID");
    }
    if(modeList.enabledWriteNodeRegisterAndPortTest)
    {
      camera_node->ControlNodeRegisterAndPortTest("DeviceUserMemory");
    }
    if(modeList.enableTriggerMode)
    {
      camera_node->start_trigger_mode(); 
    }
    else
    {
      camera_node->start_free_run_mode(); 
    }
    rclcpp::spin(camera_node);
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  mode_list_t modeList;
  memset(&modeList, 0, sizeof(modeList));
  
  for(int i = 1; i < argc; ++i)
  {
    if(argv[i][0] == '-')
    {
      switch(argv[i][1])
      {
        case('b'):  modeList.enableDisplayBootstrap = true; break;
        case('c'):  modeList.enabledAllChunks = true; break;
        case('d'):  modeList.enableDisplayDeviceList = true; break;
        case('e'):  modeList.enabledAllEvents = true; break;
        case('f'):  modeList.enabledWriteNodeFloatTest = true; break;
        case('i'):  modeList.enableDisplayGenICamInfo = true; break;
        case('l'):  modeList.enableDisplayGenICamNodeTree = true; break;
        case('m'):  modeList.enableDisplayModuleList = true; break;
        case('n'):  modeList.enabledWriteNodeIntTest = true; break;
        case('p'):  modeList.enabledWriteNodeRegisterAndPortTest = true;  break;
        case('r'):  modeList.enabledWriteNodeStringTest = true;  break;
        case('s'):  modeList.enableDisplaySDKInfo = true; break;
        case('t'):  modeList.enableTriggerMode = true;  break;
        case('v'):  modeList.enabledWriteNodeBoolTest = true;  break;
        case('w'):  modeList.enabledWriteNodeEnumTest = true;  break;
        case('x'):  modeList.enabledExecuteNodeTest = true;  break;
      }
    }
  }
  startGrab(modeList);
  
  return EXIT_SUCCESS;
}