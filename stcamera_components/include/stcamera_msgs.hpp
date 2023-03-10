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

#ifndef STCAMERA_MSGS_H
#define STCAMERA_MSGS_H

#include "stcamera_msgs/msg/chunk.hpp"
#include "stcamera_msgs/msg/device_connection.hpp"

#include "stcamera_msgs/msg/element_bool.hpp"
#include "stcamera_msgs/msg/element_float64.hpp"
#include "stcamera_msgs/msg/element_int64.hpp"
#include "stcamera_msgs/msg/element_string.hpp"

#include "stcamera_msgs/msg/error_info.hpp"
#include "stcamera_msgs/msg/event.hpp"
#include "stcamera_msgs/msg/gen_i_cam_event.hpp"
#include "stcamera_msgs/msg/gen_tl_info.hpp"



#include "stcamera_msgs/srv/enable_chunk.hpp"
#include "stcamera_msgs/srv/enable_event_acquisition.hpp"
#include "stcamera_msgs/srv/enable_event_node.hpp"
#include "stcamera_msgs/srv/enable_image_acquisition.hpp"
#include "stcamera_msgs/srv/enable_trigger.hpp"

#include "stcamera_msgs/srv/execute_node.hpp"

#include "stcamera_msgs/srv/get_chunk_list.hpp"
#include "stcamera_msgs/srv/get_device_list.hpp"
#include "stcamera_msgs/srv/get_enum_list.hpp"
#include "stcamera_msgs/srv/get_event_acquisition_status_list.hpp"
#include "stcamera_msgs/srv/get_event_node_status_list.hpp"
#include "stcamera_msgs/srv/get_gen_i_cam_node_info.hpp"
#include "stcamera_msgs/srv/get_image_acquisition_status.hpp"
#include "stcamera_msgs/srv/get_module_list.hpp"
#include "stcamera_msgs/srv/get_sdk_info.hpp"
#include "stcamera_msgs/srv/get_trigger_list.hpp"

#include "stcamera_msgs/srv/read_node.hpp"
#include "stcamera_msgs/srv/read_node_bool.hpp"
#include "stcamera_msgs/srv/read_node_enum.hpp"
#include "stcamera_msgs/srv/read_node_float.hpp"
#include "stcamera_msgs/srv/read_node_int.hpp"
#include "stcamera_msgs/srv/read_node_port.hpp"
#include "stcamera_msgs/srv/read_node_register.hpp"
#include "stcamera_msgs/srv/read_node_register_info.hpp"
#include "stcamera_msgs/srv/read_node_string.hpp"

#include "stcamera_msgs/srv/send_soft_trigger.hpp"

#include "stcamera_msgs/srv/write_node.hpp"
#include "stcamera_msgs/srv/write_node_bool.hpp"
#include "stcamera_msgs/srv/write_node_enum_int.hpp"
#include "stcamera_msgs/srv/write_node_enum_str.hpp"
#include "stcamera_msgs/srv/write_node_float.hpp"
#include "stcamera_msgs/srv/write_node_int.hpp"
#include "stcamera_msgs/srv/write_node_port.hpp"
#include "stcamera_msgs/srv/write_node_register.hpp"
#include "stcamera_msgs/srv/write_node_string.hpp"

#endif //STCAMERA_MSGS_H
