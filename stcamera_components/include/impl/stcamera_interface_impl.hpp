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

/** \file stcamera_interface_impl.hpp
 *  \brief Base class to control a connected camera
 *
 *  This is a base class to control a connected camera, including callback
 *  for services. 
 */

#ifndef STCAMERA_STCAMERA_INTERFACE_IMPL_H
#define STCAMERA_STCAMERA_INTERFACE_IMPL_H

#include <map>
#include <sstream>
#include "../stcamera_interface.hpp"

#include <StApi_TL.h>
#include <StApi_IP.h>

#define ENABLED_STAPI_ZERO_COPY 1

namespace stcamera
{
/** Macro for common exception. */
#define CATCH_COMMON_ERR_RES(RES) catch(const StApi::CStGenTLErrorException &x) \
  {\
    RETURN_ERR_RES((EOSError_t)x.GetError(), x.GetDescription(), RES);\
  }\
  catch(GenICam::GenericException &x)\
  {\
    RETURN_ERR_RES(OSError_GenICamError, x.GetDescription(), RES);\
  }\
  catch(...)\
  {\
    RETURN_ERR_RES(OSError_UnknownError, OSERROR_STR_UNKNOWN_ERROR, RES);\
  }

  /** A structure containing a pair of topic name for publishing data from
   * a callback function and the pointer to the instance of the callback. */
  typedef struct StCallback_t
  {
    /** Name of the topic. */
    std::string topic_name_;
    /** Name of the event. */
    std::string event_name_;
    /** Pointer to the callback. */
    StApi::IStRegisteredCallbackReleasable *cb_;
  }StCallback_t;
  
  /** Map key: callback node name; value: StCallback_t */
  typedef std::map<std::string, StCallback_t> MapCallback;

  /** Map key: chunk node name; value: pointer to the chunk node */
  typedef std::map<std::string, GenApi::INode*> MapChunk;

#if ENABLED_STAPI_ZERO_COPY
    // Custom memory allocator class implementation.
    class CImageAllocator : public StApi::IStAllocator
    {
    public:
        explicit CImageAllocator(const std::string &camera_namespace) : camera_namespace_(camera_namespace)
        {}

        // Function for memory allocation.
        void Allocate(void **pCreatedBuffer, size_t nSize, void **pContext) throw()
        {
            try
            {
                sensor_msgs::msg::Image *pimage = new sensor_msgs::msg::Image();

                pimage->header.frame_id = camera_namespace_;

                // Memory allocation of the requested size.
                pimage->data.resize(nSize);
                *pCreatedBuffer = &pimage->data[0];

                // Assigning the number of allocation to pContext. 
                // Value assigned to pContext is passed as an argument when the memory is deallocated.
                *(sensor_msgs::msg::Image**)pContext = pimage;

                map_buffer_image_.insert(std::make_pair(*pCreatedBuffer, pimage));

                // Display the requested memory size.
                //std::cout << "Allocate Size = " << nSize << ", Image=" << pimage << ", Buffer=" << *pCreatedBuffer << std::endl;
            }
            catch (...)
            {
                *pCreatedBuffer = NULL;
            }

        }

        // Function for memory deallocation.
        void Deallocate(void *ptr, size_t /*nSize*/, void *pContext) throw()
        {
            // Display the size of the memory to be freed.
            //std::cout << "Deallocate Size = " << nSize << ", Buffer=" << ptr << std::endl;

            sensor_msgs::msg::Image *pimage = (sensor_msgs::msg::Image *)pContext;

            map_buffer_image_.erase(ptr);

            // Free the allocated memory.
            pimage->data.resize(0);
            delete pimage;
        }

        // Function called when registered allocator is no longer needed.
        void OnDeregister() throw()
        {
        }

        sensor_msgs::msg::Image* get_image(void* ptr)
        {
            auto it = map_buffer_image_.find(ptr);
            if(it != map_buffer_image_.end())
            {
                //std::cout << "CImageAllocator::get_image Buffer=" << ptr <<", Image=" << it->second <<  std::endl;
                return(it->second);
            }
            //std::cout << "CImageAllocator::get_image Buffer=" << ptr <<", Image=nullptr" <<  std::endl;
            return(nullptr);
        }
    protected:
        std::map<void *, sensor_msgs::msg::Image *> map_buffer_image_;
        const std::string camera_namespace_;
    };
#endif //ENABLED_STAPI_ZERO_COPY

  /** \class StCameraInterfaceImpl
   *  \brief Class to control a connected camera
   *
   *  This class is specific for camera.
   *  If there is a more specific camera to control, derive this 
   *  class.
   */
  class StCameraInterfaceImpl: public StCameraInterface
  {
    public:

      /** Constructor.
       *
       * \param[in] dev Pointer to the IStDeviceReleasable of the device.
       * \param[in] nh_parent The main ROS node handle.
       * \param[in] camera_namespace The namespace for the device.
       * \param[in] param Pointer to the StParameter class instance.
       * \param[in] queue_size Used for initializing publisher (Maximum number 
       *            of outgoing messages to be queued for delivery to 
       *            subscribers). Default is set to #STCAMERA_QUEUE_SIZE 
       */
      StCameraInterfaceImpl(StApi::IStDeviceReleasable *dev,
                        rclcpp::Node *nh_parent, 
                        const std::string &camera_namespace, 
                        StParameter *param,
                        rclcpp::Clock &clock,
                        uint32_t queue_size = STCAMERA_QUEUE_SIZE);

      /** Destructor.
       *
       * All event node callbacks are deregistered and the callbacks are
       * released here. 
       */
      virtual ~StCameraInterfaceImpl();

      /** Check if the device is disconnected.
       * \return True if device is already disconnected. False otherwise.
       */
      bool deviceIsLost() override;
    protected:
      void initDeviceAndDataStream();
      void registerDeviceLostEvent();
      void activateChunks();
    protected:

      /** GenTL System event callback function.
       * 
       * This function is called automatically when any GenTL System event 
       * occured.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventSystemCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** GenTL Interface event callback function.
       * 
       * This function is called automatically when any GenTL Interface event 
       * occured. The event data is published through the default event topic.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventInterfaceCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** GenTL Device event callback function.
       * 
       * This function is called automatically when any GenTL Device event 
       * occured. The event data is published through the default event topic.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventDeviceCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** GenICam node event callback function.
       * 
       * This function is called automatically if a GenICam node event is
       * registered. The event data is published through either the custom
       * event topic provided when enabling the event or the default event
       * topic.
       * \param[in] p Pointer to the GenICam node callback.
       * \param[in] pvContext user's pointer (pointer to the MapCallback
       *            of which the event belongs to)
       */
      void eventGenApiNodeCB(GenApi::INode *p, void *pvContext);

      /** GenTL DataStream event callback function.
       * 
       * This function is called automatically when any GenTL DataStream event 
       * occured. Except for GenTL EVENT_NEW_BUFFER of which data is published 
       * throught image_transport mechanism or chunk topic, other event data is 
       * published through the default event topic.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventDataStreamCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** ROS service callback for obtaining GenICam node value regardless
       * the node value's interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */

      bool readNodeCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNode_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNode_Response> res);
      /** ROS service callback for obtaining GenICam node value with boolean
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeBoolCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodeBool_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodeBool_Response> res);

      /** ROS service callback for obtaining GenICam node value with enumeration
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeEnumCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodeEnum_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodeEnum_Response> res);

      /** ROS service callback for obtaining GenICam node value with integer
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeIntCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodeInt_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodeInt_Response> res);

      /** ROS service callback for obtaining GenICam node value with float
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeFloatCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodeFloat_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodeFloat_Response> res);

      /** ROS service callback for obtaining GenICam node value with port
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodePortCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodePort_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodePort_Response> res);

      /** ROS service callback for obtaining GenICam node value with register
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeRegisterCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodeRegister_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodeRegister_Response> res);

      /** ROS service callback for obtaining GenICam node info with register
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeRegisterInfoCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodeRegisterInfo_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodeRegisterInfo_Response> res);

      /** ROS service callback for obtaining GenICam node value with string
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeStringCallback(
          const std::shared_ptr<stcamera_msgs::srv::ReadNodeString_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ReadNodeString_Response> res);

      /** ROS service callback for writing GenICam node value regardless 
       * the node value's interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNode_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNode_Response> res);

      /** ROS service callback for writing GenICam node value with boolean
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeBoolCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodeBool_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodeBool_Response> res);

      /** ROS service callback for writing GenICam node value with enumeration
       * interface type, where the input is the integer value of the enumeration
       * entry.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeEnumIntCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumInt_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumInt_Response> res);

      /** ROS service callback for writing GenICam node value with enumeration
       * interface type, where the input is the symbolic name of the enumeration
       * entry.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeEnumStrCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumStr_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodeEnumStr_Response> res);

      /** ROS service callback for writing GenICam node value with integer
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeIntCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodeInt_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodeInt_Response> res);

      /** ROS service callback for writing GenICam node value with float
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeFloatCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodeFloat_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodeFloat_Response> res);

      /** ROS service callback for writing GenICam node value with port
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodePortCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodePort_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodePort_Response> res);

      /** ROS service callback for writing GenICam node value with register
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeRegisterCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodeRegister_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodeRegister_Response> res);

      /** ROS service callback for writing GenICam node value with string
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeStringCallback(
          const std::shared_ptr<stcamera_msgs::srv::WriteNodeString_Request> req,
          std::shared_ptr<stcamera_msgs::srv::WriteNodeString_Response> res);

      /** ROS service callback for executing GenICam node that has command
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool executeNodeCallback(
          const std::shared_ptr<stcamera_msgs::srv::ExecuteNode_Request> req,
          std::shared_ptr<stcamera_msgs::srv::ExecuteNode_Response> res);

       /** ROS service callback for enabling or disabling chunk.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableChunkCallback(
          const std::shared_ptr<stcamera_msgs::srv::EnableChunk_Request> req,
          std::shared_ptr<stcamera_msgs::srv::EnableChunk_Response> res);

      /** ROS service callback for enabling or disabling trigger.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableTriggerCallback( 
          const std::shared_ptr<stcamera_msgs::srv::EnableTrigger_Request> req,
          std::shared_ptr<stcamera_msgs::srv::EnableTrigger_Response> res);

      /** ROS service callback for enabling or disabling GenICam event.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableEventNodeCallback( 
          const std::shared_ptr<stcamera_msgs::srv::EnableEventNode_Request> req,
          std::shared_ptr<stcamera_msgs::srv::EnableEventNode_Response> res);

      /** ROS service callback for enabling or disabling image acquisition.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableImageAcquisitionCallback( 
          const std::shared_ptr<stcamera_msgs::srv::EnableImageAcquisition_Request> req,
          std::shared_ptr<stcamera_msgs::srv::EnableImageAcquisition_Response> res);

      /** ROS service callback for enabling or disabling event acquisition. 
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableEventAcquisitionCallback( 
          const std::shared_ptr<stcamera_msgs::srv::EnableEventAcquisition_Request> req,
          std::shared_ptr<stcamera_msgs::srv::EnableEventAcquisition_Response> res);

      /** ROS service callback for obtaining image acquisition status.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getImageAcquisitionStatusCallback( 
          const std::shared_ptr<stcamera_msgs::srv::GetImageAcquisitionStatus_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetImageAcquisitionStatus_Response> res);

      /** ROS service callback for obtaining event acquisition status.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getEventAcquisitionStatusListCallback( 
          const std::shared_ptr<stcamera_msgs::srv::GetEventAcquisitionStatusList_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetEventAcquisitionStatusList_Response> res);

      /** ROS service callback for obtaining GenICam node events status.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getEventNodeStatusListCallback( 
          const std::shared_ptr<stcamera_msgs::srv::GetEventNodeStatusList_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetEventNodeStatusList_Response> res);

      /** ROS service callback for obtaining chunks provided by the camera.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getChunkListCallback(
          const std::shared_ptr<stcamera_msgs::srv::GetChunkList_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetChunkList_Response> res);

      /** ROS service callback for obtaining triggers provided by the camera.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getTriggerListCallback(
          const std::shared_ptr<stcamera_msgs::srv::GetTriggerList_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetTriggerList_Response> res);

      /** ROS service callback for obtaining enumeration entry of a given
       * GenICam node with interface type of enumeration.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getEnumListCallback(
          const std::shared_ptr<stcamera_msgs::srv::GetEnumList_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetEnumList_Response> res);

      /** ROS service callback for obtaining information of a given GenICam 
       * node.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getGenICamNodeInfoCallback(
          const std::shared_ptr<stcamera_msgs::srv::GetGenICamNodeInfo_Request> req,
          std::shared_ptr<stcamera_msgs::srv::GetGenICamNodeInfo_Response> res);

      /** ROS service callback for sending software trigger execution command
       * to the camera.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool sendSoftTriggerCallback(
          const std::shared_ptr<stcamera_msgs::srv::SendSoftTrigger_Request> req,
          std::shared_ptr<stcamera_msgs::srv::SendSoftTrigger_Response> res);

      /** Initialize information of the camera. This information is delivered 
       * to image subscribers upon publishing acquired images. 
       */  
      void initializeCameraInfo();

      /** A helper function to obtain the GenICam node for a given module name.
       * \param[in] genicam_module Name of the module: System, Interface,
       *             LocalDevice, RemoteDevice, DataStream.
       * \return nullptr if module name is invalid. Otherwise GenICam node of
       *         the module.
       */
      GenApi::INodeMap *getNodeMap(const std::string &genicam_module);


      /** A helper function to obtain one of event callback mapping for a given
       * module name.
       * \param[in] genicam_module Name of the module: System, Interface,
       *             LocalDevice, RemoteDevice, DataStream.
       * \return nullptr if module name is invalid. Otherwise the callback map
       *         that corresponds to the module.
       */
      MapCallback *getCallbackMap(const std::string &genicam_module);
      
      /** A helper function to publish an event through the default event
       * publisher.
       * \param[in] msg The message to deliver.
       */
      void publishEventDefault(stcamera_msgs::msg::Event &msg);

    protected:
      void initPublishers();
      void initServices();

    protected:
      /** Smart pointer to the CIStDevice that holds the device instance. */
      StApi::CIStDevicePtr tl_dev_;

      /** Used for managing the camera info. */
      camera_info_manager::CameraInfoManager camera_info_manager_; 
      /** Queue size for publisher (used when dynamically advertise custom
       * topic name).
       */
      const uint32_t queue_size_;

      /** Flag to indicate if GenTL System event is being enabled or not. */
      bool bool_event_system_;
      /** Flag to indicate if GenTL Interface event is being enabled or not. */
      bool bool_event_interface_;
      /** Flag to indicate if GenTL Device event is being enabled or not. */
      bool bool_event_device_;
      /** Flag to indicate if GenTL DataStream event is being enabled or not. */
      bool bool_event_datastream_;
      
      /** destroyed */
      bool destroyed_;

      StApi::CIStRegisteredCallbackPtr ist_registered_callback_system_;
      StApi::CIStRegisteredCallbackPtr ist_registered_callback_interface_;
      StApi::CIStRegisteredCallbackPtr ist_registered_callback_device_;
      StApi::CIStRegisteredCallbackPtr ist_registered_callback_datastream_;

      /** Publisher for image data. */
      image_transport::CameraPublisher it_campub_;

      /** Smart pointer to the CIStDataStream that holds the device's datastream. */
      StApi::CIStDataStreamPtr tl_ds_;

      rclcpp::Publisher<stcamera_msgs::msg::Event>::SharedPtr def_event_;

      /** Publisher for chunk data. */
      rclcpp::Publisher<stcamera_msgs::msg::Chunk>::SharedPtr msg_chunk_;

      /** Guard to prevent race when enabling or disabling image acquisition. */
      std::mutex mtx_acquisition_;
      /** Flag to indicate if image acquisition is being enabled or not. */
      bool bool_acquisition_is_started_;

      /** Guard to prevent race when enabling or disabling event acquisition. */
      std::mutex mtx_event_;

      /** Map of event publisher in pair of (topic name, publisher). */
      MapPublisher  map_msg_event_;

      /** Map the GenICam event node callback name (System module) with the 
       *  StCallback_t (pair of topic/event name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_system_;
      /** Map the GenICam event node callback name (Interface module) with the 
       *  StCallback_t (pair of topic/event name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_interface_;
      /** Map the GenICam event node callback name (LocalDevice module) with 
       *  the StCallback_t (pair of topic name/event and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_localdevice_;
      /** Map the GenICam event node callback name (RemoteDevice module) with 
       *  the StCallback_t (pair of topic/event name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_remotedevice_;
      /** Map the GenICam event node callback name (DataStream module) with the 
       *  StCallback_t (pair of topic/event name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_datastream_;
      
      /** Guard to prevent race when enabling or disabling chunk. */
      std::mutex mtx_chunk_;
      
      /** Map the chunk name with the chunk's GenICam node */
      MapChunk  map_chunk_;

#if ENABLED_STAPI_ZERO_COPY
      CImageAllocator image_allocator_;
      StApi::CIStImageBufferPtr p_stimage_buffer_;
      StApi::CIStPixelFormatConverterPtr p_stpixel_format_converter_;
#else
      sensor_msgs::msg::Image image_;
#endif //ENABLED_STAPI_ZERO_COPY
      sensor_msgs::msg::CameraInfo camera_info_;

    protected:
      /** Server for read_node */
      rclcpp::Service<stcamera_msgs::srv::ReadNode>::SharedPtr  srv_read_node_;
      /** Server for read_node_bool */
      rclcpp::Service<stcamera_msgs::srv::ReadNodeBool>::SharedPtr  srv_read_node_bool_;
      /** Server for read_node_enum */
      rclcpp::Service<stcamera_msgs::srv::ReadNodeEnum>::SharedPtr  srv_read_node_enum_;
      /** Server for read_node_int */
      rclcpp::Service<stcamera_msgs::srv::ReadNodeInt>::SharedPtr  srv_read_node_int_;
      /** Server for read_node_float */
      rclcpp::Service<stcamera_msgs::srv::ReadNodeFloat>::SharedPtr  srv_read_node_float_;
      /** Server for read_node_port */
      rclcpp::Service<stcamera_msgs::srv::ReadNodePort>::SharedPtr  srv_read_node_port_;
      /** Server for read_node_register*/
      rclcpp::Service<stcamera_msgs::srv::ReadNodeRegister>::SharedPtr  srv_read_node_register_;
      /** Server for read_node_register_info*/
      rclcpp::Service<stcamera_msgs::srv::ReadNodeRegisterInfo>::SharedPtr  srv_read_node_register_info_;
      /** Server for read_node_string */
      rclcpp::Service<stcamera_msgs::srv::ReadNodeString>::SharedPtr  srv_read_node_string_;

      /** Server for write_node */
      rclcpp::Service<stcamera_msgs::srv::WriteNode>::SharedPtr  srv_write_node_;
      /** Server for write_node_bool */
      rclcpp::Service<stcamera_msgs::srv::WriteNodeBool>::SharedPtr  srv_write_node_bool_;
      /** Server for write_node_enum_int */
      rclcpp::Service<stcamera_msgs::srv::WriteNodeEnumInt>::SharedPtr  srv_write_node_enum_int_;
      /** Server for write_node_enum_str */
      rclcpp::Service<stcamera_msgs::srv::WriteNodeEnumStr>::SharedPtr  srv_write_node_enum_str_;
      /** Server for write_node_int */
      rclcpp::Service<stcamera_msgs::srv::WriteNodeInt>::SharedPtr  srv_write_node_int_;
      /** Server for write_node_float */
      rclcpp::Service<stcamera_msgs::srv::WriteNodeFloat>::SharedPtr  srv_write_node_float_;
      /** Server for write_node_port */
      rclcpp::Service<stcamera_msgs::srv::WriteNodePort>::SharedPtr  srv_write_node_port_;
      /** Server for write_node_register */
      rclcpp::Service<stcamera_msgs::srv::WriteNodeRegister>::SharedPtr  srv_write_node_register_;
      /** Server for write_node_string */
      rclcpp::Service<stcamera_msgs::srv::WriteNodeString>::SharedPtr  srv_write_node_string_;

      /** Server for execute_node */
      rclcpp::Service<stcamera_msgs::srv::ExecuteNode>::SharedPtr  srv_execute_node_;
      
      /** Server for enable_chunk */
      rclcpp::Service<stcamera_msgs::srv::EnableChunk>::SharedPtr  srv_enable_chunk_;    
      /** Server for enable_trigger */
      rclcpp::Service<stcamera_msgs::srv::EnableTrigger>::SharedPtr  srv_enable_trigger_;  
      /** Server for enable_event_node */
      rclcpp::Service<stcamera_msgs::srv::EnableEventNode>::SharedPtr  srv_enable_event_node_;
      /** Server for enable_image_acquisition */
      rclcpp::Service<stcamera_msgs::srv::EnableImageAcquisition>::SharedPtr  srv_enable_image_acquisition_;
      /** Server for enable_event_acquisition */
      rclcpp::Service<stcamera_msgs::srv::EnableEventAcquisition>::SharedPtr  srv_enable_event_acquisition_;
      
      /** Server for get_image_acquisition_status */
      rclcpp::Service<stcamera_msgs::srv::GetImageAcquisitionStatus>::SharedPtr  srv_get_image_acquisition_status_;
      /** Server for get_event_acquisition_status_list */
      rclcpp::Service<stcamera_msgs::srv::GetEventAcquisitionStatusList>::SharedPtr  srv_get_event_acquisition_status_list_;
      /** Server for get_event_node_status_list */
      rclcpp::Service<stcamera_msgs::srv::GetEventNodeStatusList>::SharedPtr  srv_get_event_node_status_list_;
      /** Server for get_chunk_list */
      rclcpp::Service<stcamera_msgs::srv::GetChunkList>::SharedPtr  srv_get_chunk_list_;
      /** Server for get_trigger_list */
      rclcpp::Service<stcamera_msgs::srv::GetTriggerList>::SharedPtr  srv_get_trigger_list_;
      /** Server for get_enum_list */
      rclcpp::Service<stcamera_msgs::srv::GetEnumList>::SharedPtr  srv_get_enum_list_;
      /** Server for get_genicam_node_info */
      rclcpp::Service<stcamera_msgs::srv::GetGenICamNodeInfo>::SharedPtr  srv_get_genicam_node_info_;

      /** Server for send_soft_trigger */
      rclcpp::Service<stcamera_msgs::srv::SendSoftTrigger>::SharedPtr srv_send_soft_trigger_;  
  };
}
#endif //STCAMERA_STCAMERA_INTERFACE_IMPL_H