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

#ifndef STCAMERA_HPP
#define STCAMERA_HPP

#include "stcamera_base.hpp"

namespace stcamera
{
    class CStCamera : public CStCameraBase
    {
    public:
        CStCamera(const std::string & camera_name_space);
        ~CStCamera();
        void softtrigger_timer_callback();
        void enabledAllChunks();
        void enabledAllEvents();
        void executeNodeTest();
        void displayEventNodeStatusList(const std::string &genicam_module, const std::string &prefix = "");
        void displayEventAcquisitionStatusList(const std::string &prefix = "");
        void ControlNodeFloatTest(const std::string &genicam_node, const std::string &prefix = "");
        void ControlNodeIntTest(const std::string &genicam_node, const std::string &prefix = "");
        void ControlNodeEnumTest(const std::string &genicam_node, const std::string &prefix = "");
        void ControlNodeBoolTest(const std::string &genicam_node, const std::string &prefix = "");
        void ControlNodeStringTest(const std::string &genicam_node, const std::string &prefix = "");
        void printBuffer(const std::vector<uint8_t> &buffer, size_t max_print_len, const std::string &prefix = "");
        void ControlNodeRegisterAndPortTest(const std::string &genicam_node, const std::string &prefix = "");
        void displayGenICamNodeInfo(const std::string &genicam_module, const std::string &genicam_node, const std::string &prefix = "");
        void displayBootstrap(const std::string &device_tl_type);
        void displayGenICamNodeTree(const std::string &genicam_module, const std::string &genicam_node, const std::string &prefix = "");
        void displayTriggerList(const std::string &prefix = "");
        void start_free_run_mode();
        void start_trigger_mode();
        void stop_trigger_mode();
    protected:
        int grabbed_images_;
        std::shared_ptr<rclcpp::Node> node_for_trigger_;
    protected:
        void onRcvImage(sensor_msgs::msg::Image::SharedPtr msg) override;
        void onRcvChunk(stcamera_msgs::msg::Chunk::SharedPtr msg) override;
        void onRcvEvent(stcamera_msgs::msg::Event::SharedPtr msg) override;
    };
}

#endif //STCAMERA_HPP