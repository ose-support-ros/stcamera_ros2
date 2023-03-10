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

/** \file stcamera_node_impl.hpp
 *  \brief Class implementation for stcamera
 *
 *  This class implements the stcamera.
 */

#ifndef STCAMERA_STCAMERA_NODE_IMPL_H
#define STCAMERA_STCAMERA_NODE_IMPL_H

#include "../stcamera_node.hpp"
#include <StApi_TL.h>

namespace stcamera
{
  /** \class StCameraNodeImpl
   *  \brief Class implementation for stcamera
   *
   *  This class implements the stcamera.
   */ 
  class StCameraNodeImpl
  {
    public:
        explicit StCameraNodeImpl(StCameraNode *p_stcamera_node);
        ~StCameraNodeImpl();

      /** Function called inside process loop.
       *  In this function, device availability is checked. Any device lost
       *  will be announced through device_connection topic. Any new found
       *  device will be processed through calling initializeCamera() .
       */
      void DetectingAndOpenningDevices();

      /** Function to initialize a camera.
       * In this function, whether connection to the given camera is allowed
       * will be checked.
       * \param[in] p_iface Pointer to the IStInterface of the camera.
       * \param[in] p_devinfo Pointer to the IStDeviceInfo of the device.
       * \return True if the camera is successfully initialized. 
       *            False otherwise.
       */
      bool initializeCamera(StApi::IStInterface *p_iface,
                            const StApi::IStDeviceInfo *p_devinfo);

      /** ROS service callback for obtaining list of all detected devices
       * including the disallowed camera to connect.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return Always true.
       */
      bool getDeviceListCallback(
          const std::shared_ptr<stcamera_msgs::srv::GetDeviceList_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetDeviceList_Response> res);

      /** ROS service callback for obtaining SentechSDK and GenTL information.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return Always true.
       */
      bool getSDKInfoCallback(
          const std::shared_ptr<stcamera_msgs::srv::GetSDKInfo_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetSDKInfo_Response> res);

      /** ROS service callback for obtaining all module name: System, Interface,
       * LocalDevice, RemoteDevice, DataStream.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return Always true.
       */
      bool getModuleListCallback(
          const std::shared_ptr<stcamera_msgs::srv::GetModuleList_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetModuleList_Response> res);
          
      void initSystemsAndInterfaces();
    protected:
      const std::string &getTextForMsg() const {return(stcamera_node_text_);}
      rclcpp::Logger get_logger()
      {
        return(p_stcamera_node_->get_logger());
      }
    protected:
      /** Clock **/
      rclcpp::Clock clock_;

      /** Store the list of allowed camera or connected camera and the
       * corresponding instances. */
      MapCameraInterface map_camera_;

      /** Store the list of all detected camera (including disallowed to
       * connect) and the camera information. */
      MapDeviceConnection map_connection_;

      /** Variable for auto initialization of SentechSDK */
      StApi::CStApiAutoInit stapi_autoinit_;

      /** List of GenTL instances used by SentechSDK */
      StApi::CIStSystemPtrArray stapi_systems_;

      StCameraNode *p_stcamera_node_;
      
      const std::string stcamera_node_text_;

    private:

      /** Guard to prevent race when looking for new devices. */
      std::mutex mtx_update_device_list_;
      
      /** Guard to prevent race when accessing map_camera_. */
      std::mutex mtx_map_camera_;
      
  };
}
#endif //STCAMERA_STCAMERA_NODE_IMPL_H
