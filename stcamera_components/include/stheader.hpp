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

/** \file stheader.hpp
 *  \brief Contains definitions and macros
 *
 * Contains definitions for error code, name of topics and services, and macro
 * to deal with common exception.
 */

#ifndef STCAMERA_STHEADER_H
#define STCAMERA_STHEADER_H

/**  Max number of topic's item to be queued for delivery to subscribers. */
#define STCAMERA_QUEUE_SIZE         5

#define TIME_SOURCE RCL_ROS_TIME

/** 
 * \defgroup ERROR_CODE_GROUP List of returned error code other than from GenTL
 * \{
 */
typedef enum EOSError_t
{
    OSError_Success = 0,    // Success
    OSError_GenTL_Error = -1001,
    OSError_GenTL_NotInitializedError = -1002,
    OSError_GenTL_NotImplementedError = -1003,
    OSError_GenTL_ResourceInUseError = -1004,
    OSError_GenTL_AccessDeniedError = -1005,
    OSError_GenTL_InvalidHandleError = -1006,
    OSError_GenTL_InvalidIDError = -1007,
    OSError_GenTL_NoDataError = -1008,
    OSError_GenTL_InvalidParameterError = -1009,
    OSError_GenTL_IOError = -1010,
    OSError_GenTL_TimeoutError = -1011,
    OSError_GenTL_AbortError = -1012,
    OSError_GenTL_InvalidBufferError = -1013,
    OSError_GenTL_NotAvailableError = -1014,
    OSError_GenTL_InvalidAddressError = -1015,
    OSError_GenTL_BufferTooSmallError = -1016,
    OSError_GenTL_InvalidIndexError = -1017,
    OSError_GenTL_ParsingChunkDataError = -1018,
    OSError_GenTL_InvalidValueError = -1019,
    OSError_GenTL_ResouceExhaustedError = -1020,
    OSError_GenTL_OutOfMemoryError = -1021,
    OSError_GenTL_BusyError = -1022,
    OSError_GenTL_AmbiguousError = -1023,

    /** Error code returned when unknown exception happened. */
    OSError_UnknownError = 29999,

    /** Error code returned when any GenICam/GenApi library throws exception. */
    OSError_GenICamError = 30000,

    /** Error code returned when module name is invalid. Possible module name are
     * as follows: System, Interface, LocalDevice, RemoteDevice, DataStream. */
    OSError_ModuleError = 30001,

    /** Error code returned when either node name is invalid or inaccessible. */
    OSError_NodeError = 30002,

    /** Error code returned when enabling an-already-enabled event. */
    OSError_EventAlreadyOnError = 30003,

    /** Error code returned when disabling an-already-disabled event. */
    OSError_EventAlreadyOffError = 30004,

    /** Error code returned when enabling an-already-enabled image acquisition. */
    OSError_ImageAcquisitionAlreadyOnError = 30005,

    /** Error code returned when disabling an-already-disabled image acquisition. */
    OSError_ImageAcquisitionAlreadyOffError = 30006,

    /** Error code returned when accessing chunk while chunk is not supported. */
    OSError_ChunkNotSupportedError = 30007,

    /** Error code returned when enabling non-existance/inaccessible chunk. */
    OSError_ChunkInaccessibleError = 30008,

    /** Error code returned when trigger is not fully supported. */
    OSError_TriggerNotSupportedError = 30009,

    /** Error code returned when enabling non-existance/inaccessible trigger 
     * selector or source. */
    OSError_TriggerInaccessibleError = 30010,

    /** Error code returned when event is not fully supported. */
    OSError_EventNotSupportedError = 30011,

    /** Error code returned when enabling non-existance/inaccessible event or 
     * event node callback. */
    OSError_EventInaccessibleError = 30012,


    OSError_GivenIdCameraNotFound = 30013,
} EOSError_t, *PEOSError_t;

#define OSERROR_STR_GENTL_INVALID_ID_ERROR "ID could not be connected to a resource"
#define OSERROR_STR_UNKNOWN_ERROR "Unknown exception error"
#define OSERROR_STR_MODULE_ERROR "Invalid module name"
#define OSERROR_STR_REMOTE_DEVUCE_EVENT_ALWAYS_ENABLED_ERROR "EventAcquisition for RemoteDevice is always enabled while the device is connected."
#define OSERROR_STR_NODE_ERROR "Invalid GenICam node"
#define OSERROR_STR_EVENT_ALREADY_ON_ERROR "Event already ON"
#define OSERROR_STR_EVENT_ALREADY_OFF_ERROR "Event already OFF"
#define OSERROR_STR_IMAGE_ACQ_ALREADY_ON_ERROR "Image acquisition already ON"
#define OSERROR_STR_IMAGE_ACQ_ALREADY_OFF_ERROR "Image acquisition already OFF"
#define OSERROR_STR_CHUNK_NOT_SUPPORTED_ERROR "Chunk not supported"
#define OSERROR_STR_CHUNK_INACCESSIBLE_ERROR "Chunk inaccessible"
#define OSERROR_STR_TRIGGER_NOT_SUPPORTED_ERROR "Trigger not supported"
#define OSERROR_STR_TRIGGER_INACCESSIBLE_ERROR "Trigger inaccessible" 
#define OSERROR_STR_EVENT_NOT_SUPPORTED_ERROR "Event not supported"
#define OSERROR_STR_EVENT_INACCESSIBLE_ERROR_STR "Event inaccessible"
#define OSERROR_STR_GIVEN_ID_CAMERA_NOT_FOUND_STR "The Camera with the given ID could not be found."
/** \} */


/**
 * \defgroup TOPIC_NAME_GROUP List of topic name
 * \{
 */

/** Topic name for device_connection. */
#define STMSG_device_connection     "device_connection"

/** Topic name for image. */
#define STMSG_image                 "image_raw"

/** Topic name for chunk. */
#define STMSG_chunk                 "chunk"

/** Topic name for default event. */
#define STMSG_event                 "event"

/** \} */

#define STCAMERA_NODE_NAME           "stcameras"

/**
 * \defgroup SERVICE_NAME_GROUP List of service name
 * \{
 */

/** Service name for obtaining the list of detected devices. */
#define STSRV_G_device_list         "get_device_list"

/** Service name for obtaining the description of a given error code. */
#define STSRV_G_error_code_info     "get_error_code_info"

/** Service name for obtaining the list of module name. */
#define STSRV_G_module_list         "get_module_list"

/** Service name for obtaining the SentechSDK version and GenTL information. */
#define STSRV_G_sdk_info            "get_sdk_info"

/** Service name for obtaining GenICam node value regardless the interface type. 
 */
#define STSRV_R_node                "read_node"

/** Service name for obtaining GenICam node value with boolean interface type.
 */
#define STSRV_R_node_bool           "read_node_bool"

/** Service name for obtaining GenICam node value with enumeration interface 
 * type. */
#define STSRV_R_node_enum           "read_node_enum"

/** Service name for obtaining GenICam node value with integer interface type. 
 */
#define STSRV_R_node_int            "read_node_int"

/** Service name for obtaining GenICam node value with float interface type. */
#define STSRV_R_node_float          "read_node_float"

/** Service name for obtaining GenICam node value with port interface type. */
#define STSRV_R_node_port         "read_node_port"

/** Service name for obtaining GenICam node value with register interface type. */
#define STSRV_R_node_register        "read_node_register"

/** Service name for obtaining GenICam node info with register interface type. */
#define STSRV_R_node_register_info        "read_node_register_info"

/** Service name for obtaining GenICam node value with string interface type. */
#define STSRV_R_node_string         "read_node_string"

/** Service name for writing GenICam node value regardless the interface type. 
 */
#define STSRV_W_node                "write_node"

/** Service name for writing GenICam node value with boolean interface type. */
#define STSRV_W_node_bool           "write_node_bool"

/** Service name for writing GenICam node value with Enumeration interface type
 * using the integer value of the enumeration entry. */
#define STSRV_W_node_enum_int       "write_node_enum_int"

/** Service name for writing GenICam node value with Enumeration interface type
 * using the symbolic name of the enumeration entry. */
#define STSRV_W_node_enum_str       "write_node_enum_str"

/** Service name for writing GenICam node value with integer interface type. */
#define STSRV_W_node_int            "write_node_int"

/** Service name for writing GenICam node value with float interface type. */
#define STSRV_W_node_float          "write_node_float"

/** Service name for writing GenICam node value with port interface type. */
#define STSRV_W_node_port         "write_node_port"

/** Service name for writing GenICam node value with register interface type. */
#define STSRV_W_node_register         "write_node_register"

/** Service name for writing GenICam node value with string interface type. */
#define STSRV_W_node_string         "write_node_string"

/** Service name for executing GenICam node command. */
#define STSRV_W_node_command        "execute_node"

/** Service name for enabling or disabling a given chunk. */
#define STSRV_E_chunk               "enable_chunk"

/** Service name for enabling or disabling a given trigger. */
#define STSRV_E_trigger             "enable_trigger"

/** Service name for enabling or disabling a given GenICam node event. */
#define STSRV_E_event_node          "enable_event_node"

/** Service name for enabling or disabling image acquisition. */
#define STSRV_E_image_acquisition   "enable_image_acquisition"

/** Service name for enabling or disabling event acquisition. */
#define STSRV_E_event_acquisition   "enable_event_acquisition"

/** Service name for obtaining image acquisition status. */ 
#define STSRV_G_image_acquisition_status  "get_image_acquisition_status"

/** Service name for obtaining all event acquisition status. */ 
#define STSRV_G_event_acquisition_status_list "get_event_acquisition_status_list"

/** Service name for obtaining GenICam node event status. */ 
#define STSRV_G_event_node_status_list    "get_event_node_status_list"

/** Service name for obtaining the list of chunks provided by the device. */
#define STSRV_G_chunk_list                "get_chunk_list"

/** Service name for obtaining the list of triggers provided by the device. */
#define STSRV_G_trigger_list              "get_trigger_list"

/** Service name for obtaining the list of enumeration entries for a 
 * given GenICam Enumeration node. */
#define STSRV_G_enum_list                 "get_enum_list"

/** Service name for obtaining the information of a given GenICam node. */
#define STSRV_G_genicam_node_info         "get_genicam_node_info"

/** Service name for sending software trigger execution command. */
#define STSRV_send_soft_trigger           "send_soft_trigger"

/** Service name for obtaining the last error information. */
#define STSRV_G_last_error                "get_last_error"

/** \} */

#endif
