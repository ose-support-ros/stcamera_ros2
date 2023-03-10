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

/** \file stparameter.hpp
 *  \brief Class to handle ROS parameter
 *
 *  This class reads camera list parameter.
 */

#ifndef STCAMERA_STPARAMETER_H
#define STCAMERA_STPARAMETER_H

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>
#include <string>

namespace stcamera
{
  /** \class StParameter
   *  \brief Class to handle ROS parameter
   *
   *  This class reads camera list parameter.
   */
 class StParameter
  {
    public:

      /** Constructor.
       */
      StParameter();

      /** Destructor.
       */
      virtual ~StParameter();

      /** Load the camera list.
       *
       * This function retrieves data from parameter camera_to_connect 
       * as a list of allowed camera to connect. The data is kept in variable
       * vec_camera_to_connect_. If there is a special keyword 'all' among the
       * data, the node will attempt to connect to any detected camera.
       * If the parameter does not exist or there is no data, the node will
       * only connect to the first found camera.
       * \param[in] nh Node handle of the main node.
       * \param[out] camera_to_connect Store the retrieved data excluding the
       *              special keyword 'all'.
       */
      void loadCameraList(const rclcpp::Node *nh, 
                         std::vector<std::string> &camera_to_connect);

      /** Check if the node shall connect to only the first found camera.
       *
       * This function checks if the node shall connect to only the first found 
       * camera.
       * \return True if connect to only the first found camera. False 
       *          otherwise.
       */
      bool connectFirstCameraOnly() const;

      /** Check if the node shall connect to all detected camera.
       *
       * This function checks if the node shall connect all detected camera.
       * \return True if connect to all detected camera. False otherwise.
       */
       bool connectAllCamera() const;

      /** Obtain the namespace of the given camera identifier.
       *
       * This function replaces any puctuation in the given camera identifier,
       * which can be in the form of camera ID or MODEL(SERIAL), with under-
       * score '_', so that it complies with ROS namespace.
       * \param[in] device_id The camera identifier.
       * \return  The namespace of the camera.
       */
      std::string getNamespace(std::string device_id) const;

      /** Get logger level parameter.
       *
       * This function will search for the logger_level parameter for the
       * given camera namespace. If such parameter does not exist, 
       * rclcpp::Logger::Level::Info is returned.
       *
       * \param[in] nh Node handle of the main node.
       * \return Logger level.
       */
      rclcpp::Logger::Level getLoggerLevel(const rclcpp::Node *nh) const;

    protected:

      /** Store the list of allowed camera to connect, including special key-
       * word 'all' if specified.
       */
      std::vector<std::string> vec_camera_to_connect_;
  };
}
#endif
