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
#include "../include/stmanager.hpp"
using namespace std::chrono_literals;

namespace stcamera
{
    CStManager::CStManager(const std::string &node_namespace, const std::string &node_name) : 
        CStManagerBase(node_namespace, node_name)
        {

        }
    CStManager::~CStManager()
    {

    }

    // Get one connected device's namespace
    std::string CStManager::getConnectedDeviceNameSpace(std::string &device_tltype)
    {
        rclcpp::WallRate loop(1s);
        while (rclcpp::ok())
        {
        std::vector<stcamera_msgs::msg::DeviceConnection> device_connection_list;
        EOSError_t ret = getDeviceList(device_connection_list);
        if(ret == OSError_Success)
        {
            for (size_t i = 0; i < device_connection_list.size(); ++i)
            {
            if (device_connection_list[i].connected)
            {
                device_tltype = device_connection_list[i].device_tltype;
                const std::string ns = pub_name_space_ + "/" + device_connection_list[i].device_namespace;
                return(ns);
            }
            }
        }
        loop.sleep();
        if(!rclcpp::ok()) break;
        }
        return("");
    }

    void CStManager::displayDeviceList()
    {
        std::cout << "displayDeviceList" << std::endl;
        std::vector<stcamera_msgs::msg::DeviceConnection> device_connection_list;
        const EOSError_t eError = getDeviceList(device_connection_list);
        if(eError == OSError_Success)
        {
        std::cout << "Device list: " << device_connection_list.size() << std::endl;
        for (size_t i = 0; i < device_connection_list.size(); ++i)
        {
            std::cout << "\tDevice[" << i << "]" << std::endl;
            std::cout << "\t\tDevice ID: " << device_connection_list[i].device_id << std::endl;
            std::cout << "\t\tDevice Model: " << device_connection_list[i].device_model << std::endl;
            std::cout << "\t\tDevice Serial: " << device_connection_list[i].device_serial << std::endl;
            std::cout << "\t\tDevice TL Type: " << device_connection_list[i].device_tltype << std::endl;
            std::cout << "\t\tDevice Namespace: " << device_connection_list[i].device_namespace << std::endl;
            std::cout << "\t\tDevice Connected: " << device_connection_list[i].connected << std::endl;
            std::cout << "\t\tTimestamp[s]: " << device_connection_list[i].timestamp.sec << "." << std::setfill('0') << std::setw(9) << device_connection_list[i].timestamp.nanosec << std::endl;
            for(size_t j = 0; j < device_connection_list[i].device_tl_specific_field.size(); ++j)
            {
            std::cout << "\t\t" << device_connection_list[i].device_tl_specific_field[j] << ": " << device_connection_list[i].device_tl_specific_value[j] << std::endl;
            }
            displayGenTLInfo(device_connection_list[i].device_gentl_info, "\t\t");
        }
        }
        else
        {
        std::cerr << "Error:" << eError << std::endl;
        }
    }
    void CStManager::displayModuleList()
    {
        std::cout << "displayModuleList" << std::endl;
        std::vector<std::string> genicam_module_list;
        const EOSError_t eError = getModuleList(genicam_module_list);
        if(eError == OSError_Success)
        {
        std::cout << "Module list: " << genicam_module_list.size() << std::endl;
        for (size_t i = 0; i < genicam_module_list.size(); i++)
        {
            std::cout << "\tmodule[" << i << "]: " << genicam_module_list[i] << std::endl;
        }
        }
        else
        {
        std::cerr << "Error:" << eError << std::endl;
        }
    }
    void CStManager::displayGenTLInfo(stcamera_msgs::msg::GenTLInfo &genTLInfo, const std::string &prefix)
    {
        std::cout << prefix << "GenTL vendor: " << genTLInfo.vendor << std::endl;
        std::cout << prefix << "\tversion: " << genTLInfo.version << std::endl;
        std::cout << prefix << "\tproducer: " << genTLInfo.producer_version << std::endl;
        std::cout << prefix << "\tfullpath: " << genTLInfo.full_path << std::endl;
        std::cout << prefix << "\tTL type: " << genTLInfo.tltype << std::endl;
    }
    // Get SentechSDK and GenTL information
    void CStManager::displaySDKInfo()
    {
        std::cout << "displaySDKInfo" << std::endl;
        std::string sdk_version;
        std::vector<stcamera_msgs::msg::GenTLInfo> gentl_info_list;
        const EOSError_t eError = getSDKInfo(sdk_version, gentl_info_list);

        if(eError == OSError_Success)
        {
        std::cout << "SentechSDK ver.: " << sdk_version << std::endl;
        for (size_t i = 0; i < gentl_info_list.size(); i++)
        {
            displayGenTLInfo(gentl_info_list[i]);
        }
        }
        else
        {
        std::cerr << "Error:" << eError << std::endl;
        }
    }
}