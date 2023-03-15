# ROS2-based Node for OMRON SENTECH Cameras

## 1. Overview
This stcamera_ros2 package is provided by OMRON SENTECH, CO.,LTD.
Any cameras supported by SentechSDK are also supported by this package on ROS2.
You can use ROS2 topics and services to retrieve images from the camera, configure the camera, generate triggers, and more.
*Note that GenICam GenApi * nodes are completely different from the ones in ROS. Each GenICamGenApi node is used to get information and configure the camera using a generic interface. The detail of the GenICam GenApi can be found in the [GenICam GenApi documentation].

## 2. Installation
This package has been tested on ROS2-Humble Hawksbill (Ubuntu 22.04 64bit).
This package uses SentechSDK (v 1.2.1 or later), so please install SentechSDK before installing this package. If you are new to the SetenchSDK, you can check out the documentation that comes with the SentechSDK and make sure that the SentechSDK's Viewer (StViewer) can capture images from your camera to make the rest of the process go smoothly. Also, if you use StViewer to change a camera's settings in advance and then use UserSetSave/UserSetDefault to save those settings to the camera, you may not need to set the camera in ROS2. You can download the SentechSDK from the following URL:

[SentechSDK download site](https://sentech.co.jp/en/data/)

For more information about how to install and use the SentechSDK, see the documentation that accompanies the SentechSDK.
 If the environment variables for using the SentechSDK are not set, run the following command (if you installed the SentechSDK in/opt/sentech):

```
source /opt/sentech/.stprofile
```

 If the ROS2 environment has not been set, execute the following command:

```
source /opt/ros/humble/setup.bash
```

 Create a workspace (for example, ~/dev_ws) and move it to the current directory.

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

 Clone the stcamera_ros2 project.

```
git clone https://github.com/ose-support-ros/stcamera_ros2.git -b humble
```

 Check dependencies and install required packages.

```
cd ~/dev_ws
rosdep install -i --from-path src --rosdistro humble -y
```

 Build.

```
colcon build
```

## 3. StCameraNode
Generating a StCameraNode in the package will start getting images from the specified camera according to the parameter **camera_to_connect**.
A StCameraNode can be generated with one of the following commands:

```
source install/setup.bash
ros2 launch stcamera_launch stcamera_launch.py
```

or

```
source install/setup.bash
ros2 component standalone --node-name 'stcameras' --node-namespace '/stcamera_launch' stcamera_components stcamera::StCameraNode
```

or

```
source install/setup.bash
ros2 run rclcpp_components component_container
```

```
source install/setup.bash
ros2 component load /ComponentManager  -n 'stcameras' --node-namespace '/stcamera_launch' stcamera_components stcamera::StCameraNode``
```

The node names and name spaces listed in the second and third methods are optional, but you can make the sample program work by specifying them as above.
If you start StCameraNode with the stcamera_launch file script, you can easily set camera connection parameters in default.yaml(./install/stcamera_launch/share/stcamera_launch/config/). See the next chapter for details on setting parameters.

### 3.1. Node Parameters
Parameter values are case sensitive. The parameters that can be set are as follows.

* **camera_to_connect** : Specify the camera to use with ROS2. This parameter must be set before StCameraNode is executed. If you change this parameter during a run, the change is not reflected. If the setting is empty, the first camera detected is used. If "all" is specified, all detected cameras are used. If CAMERA_ID or CAMERA_MODEL(SERIAL) is specified, only the specified camera will be used.

Examples:

* **camera_to_connect: []**: The first camera detected is used.
* **camera_to_connect: ["all"]**: All detected cameras are used.
* **camera_to_connect: ["00:11:1c:f6:yy:xx","STC-MCS510U3V(00XXYY0)"]**: Only GigEVision camera with MAC address "00:11:1c:f6:yy:xx" and USB3 Vision camera of which model is "STC-MCS510U3V", with serial number "00XXYY0" will be used.
* **camera_to_connect: ["14210003XXYY"]**: A camera with ID "14210003 XXYY" will be used.

### 3.2. Camera Namespace
StCameraNode declares a topic and service for each camera. Each camera's namespace used to access individual camera topics and services is automatically generated using the following rules:

* If **camera_to_connect** is empty or "all," the format of the namespace is dev\_{CAMERA\_ID}. {CAMERA\_ID} is the ID of the camera. If the connected camera ID is "14210003 xxYY", the namespace will be "dev_14210003xxYY". Non-alphanumeric characters in the camera ID, such as GigEVision's MAC address, will be replaced with underscores.

* If **camera_to_connect** is CAMERA\_MODEL(SERIAL) or CAMERA\_ID, the namespace will be of the form dev\_{CAMERA\_MODEL\_SERIAL\_} or dev\_{CAMERA\_ID}. {CAMERA\_MODEL\_SERIAL\_} is a rewritten string from CAMERA_MODEL(SERIAL). {CAMERA\_ID} is the camera ID. Non-alphanumeric characters are replaced with underscores.

### 3.3. Published Topics
Topics published by StCameraNode include:

Topics                             | Description
---------------------------------- | ----------------------------------------------
**device_connection**              | Issued when you connect or disconnect.
*{dev\_CAMERA-NS}*/**event**       | Register a camera event and it will be published when the registered event occurs.
*{dev\_CAMERA-NS}*/**chunk**       | Issued when chunk data is retrieved.
*{dev\_CAMERA-NS}*/**image_raw\*** | Published when image data is retrieved (topic based on image_transport in ROS).
*{dev\_CAMERA-NS}*/**camera_info** | A topic based on image_transport in ROS.

### 3.4. Services
StCameraNode offers the following services:

(The GenICam nodes listed below are completely separate from the ROS nodes.)

Services                           | Description
---------------------------------- | ----------------------------------------------
**get_sdk_info**                   | Get SentechSDK version and GenTL producer information.
**get_module_list**                | Get a list of GenTL module names (*System*, *Interface*, *LocalDevice*, *RemoteDevice*, *DataStream*). The module name must be passed as an argument when accessing a particular GenICam node. For example, to access a camera's * Gain * via the read_node or write_node service, you must specify RemoteDevice as the module name and Gain as the GenICam node name.
**get_device_list**                | Get a list of discovered devices.
*{dev\_CAMERA-NS}*/**get_image_acquisition_status** | Get the status of image acquisition.
*{dev\_CAMERA-NS}*/**enable_image_acquisition** | Start (enable) or stop (disable) image acquisition (Automatically enabled when StCameraNode is generated or when a camera is connected).
*{dev\_CAMERA-NS}*/**get_event_acquisition_status_list** | Get the status of event retrievals for all GenTL modules.
*{dev\_CAMERA-NS}*/**enable_event_acquisition** | Starts (enables) or stops (disables) event retrieval for the specified GenTL module.
*{dev\_CAMERA-NS}*/**get_event_node_status_list** | Get the status of the GenICam node event retrieval for the specified GenTL module.
*{dev\_CAMERA-NS}*/**enable_event_node** | Enable or disable event callbacks for the GenICam node of the specified GenTL module.
*{dev\_CAMERA-NS}*/**get_chunk_list** | Get the list and status of the chunks the camera supports. You need to stop image capture before using it.
*{dev\_CAMERA-NS}*/**enable_chunk** | Enable or disable the camera's chunked output.
*{dev\_CAMERA-NS}*/**get_trigger_list** | Get the list of triggers the camera supports.
*{dev\_CAMERA-NS}*/**enable_trigger** | Set the specified trigger on/off, trigger source, etc.
*{dev\_CAMERA-NS}*/**send_soft_trigger** | Sends software triggers to the camera.
*{dev\_CAMERA-NS}*/**execute_node** | Runs the specified GenICam node (ICommand interface type).
*{dev\_CAMERA-NS}*/**get_genicam_node_info** | Gets information about the specified GenICam node.
*{dev\_CAMERA-NS}*/**read_node** | Reads the value of the specified GenICam node as a string. Available only when the GenICam node interface type is IInteger, IFloat, IBoolean, IString, or IEnumeration.
*{dev\_CAMERA-NS}*/**write_node** | Updates the value of the specified GenICam node with the specified string. Available only when the GenICam node interface type is IInteger, IFloat, IBoolean, IString, or IEnumeration.
*{dev\_CAMERA-NS}*/**read_node_bool** | Reads the value of the specified GenICam node (IBoolean interface type).
*{dev\_CAMERA-NS}*/**write_node_bool** | Writes a value to the specified GenICam node (IBoolean interface type).
*{dev\_CAMERA-NS}*/**get_enum_list** | Gets the list of enumerators for the specified GenICam node (IEnumeration interface type).
*{dev\_CAMERA-NS}*/**read_node_enum** | Reads the value of the specified GenICam node (IEnumeration interface type).
*{dev\_CAMERA-NS}*/**write_node_enum_int** | Writes the integer value of the enumerator to the specified GenICam node (IEnumeration interface type).
*{dev\_CAMERA-NS}*/**write_node_enum_str** | Writes an enumerator string to the specified GenICam node (IEnumeration interface type).
*{dev\_CAMERA-NS}*/**read_node_int** | Reads the value of the specified GenICam node (IInteger interface type).
*{dev\_CAMERA-NS}*/**write_node_int** | Writes a value to the specified GenICam node (IInteger interface type).
*{dev\_CAMERA-NS}*/**read_node_float** | Reads the value of the specified GenICam node (IFloat interface type).
*{dev\_CAMERA-NS}*/**write_node_float** | Writes a value to the specified GenICam node (IFloat interface type).
*{dev\_CAMERA-NS}*/**read_node_string** | Reads the value of the specified GenICam node (IString interface type).
*{dev\_CAMERA-NS}*/**write_node_string** | Writes a value to the specified GenICam node (IString interface type).
*{dev\_CAMERA-NS}*/**read_node_port** | Reads the value of the specified address using the specified GenICam node (IPort interface type).
*{dev\_CAMERA-NS}*/**write_node_port** | Writes a value to the specified address using the specified GenICam node (IPort interface type).
*{dev\_CAMERA-NS}*/**read_node_register_info** | Reads information from the specified GenICam node (IRegister interface type).
*{dev\_CAMERA-NS}*/**read_node_register** | Reads the value of the specified GenICam node (IRegister interface type).
*{dev\_CAMERA-NS}*/**write_node_register** | Writes a value to the specified GenICam node (IRegister interface type).
*{dev\_CAMERA-NS}*/**set_camera_info** | A service based on image_transport in ROS.

Attention:

* Some GenICam nodes are not accessible during image acquisition (error occurred). In that case, please stop the image acquisition and access again. You can stop image acquisition with the service call **enable_image_acquisition**. For example:<br />

```
ros2 service call /stcamera_launch/dev_CAMERA-NS/enable_image_acquisition stcamera_msgs/srv/EnableImageAcquisition "{value: false}"
```

<pre>
waiting for service to become available...
requester: making request: stcamera_msgs.srv.EnableImageAcquisition_Request(value=False)

response:
stcamera_msgs.srv.EnableImageAcquisition_Response()
</pre>

* Integer-type GenICam nodes may have limited values that can be set. For the settings below, increment is set to 16, so if the value to be increased or decreased is not a multiple of that, an error occurs.<br />

```
ros2 service call /stcamera_launch/dev_CAMERA-NS/get_genicam_node_info stcamera_msgs/srv/GetGenICamNodeInfo "{genicam_module: 'RemoteDevice', genicam_node: 'Width'}"
```

<pre>
requester: making request: stcamera_msgs.srv.GetGenICamNodeInfo_Request(genicam_module='RemoteDevice', genicam_node='Width')

response:
stcamera_msgs.srv.GetGenICamNodeInfo_Response(name='Width', description='Width of the image provided by the device (in pixels).', name_space='Standard', interface_type='IInteger', access_mode='Read Only', is_cachable='Yes', visibility='Beginner', caching_mode='Write to register, write to cache on read', is_streamable=True, enum_value_str_list=[], enum_value_int_list=[], current_value='2448', min_value='64', max_value='2448', increment='16', unit='', child_node_list=[])
</pre>

* For camera settings that do not need to be changed during use, you can save the trouble of setting them every time by setting them in StViewer that comes with the Sentech SDK beforehand, saving them to the camera (UserSetSave), and setting them to be reflected when the camera is turned on (UserSetDefault).


### 3.5. Error codes
 Codes | Description
 ---: | ---
 < 0 | GenTL error codes. Please refer to [GenICam GenTL documentation].
 0 | No error.
 30000 | GenICam GenericException occurred.
 30001 | Module name is invalid.
 30002 | (GenICam) Node is either invalid or inaccessible.
 30003 | Event already ON.
 30004 | Event already OFF.
 30005 | Image acquisition already ON.
 30006 | Image acquisition already OFF.
 30007 | Chunk is not supported.
 30008 | Chunk name is not valid.
 30009 | Trigger is not supported.
 30010 | Trigger name is not valid.
 30011 | Event is not supported.
 30012 | Event name is not valid.

## 4. Usage
### 4.1. Displaying acquired images withrqt_image_view and Changing Camera Gain:

* Generates a StCameraNode.

```
cd ~/dev_ws
source install/setup.bash
ros2 launch stcamera_launch stcamera_launch.py
```

* Check the name of the current published topic.
```
ros2 topic list
```

* Run rqt_image_view and select/xxxx/xxxx/image_raw to view the images.<br />

```
ros2 run rqt_image_view rqt_image_view
```

**If the camera is in trigger mode, no images can be captured until a trigger is generated. Discard the node, switch to free run mode with StViewer, etc., and check again.**

* You can check the currently issued service and its type with a command like this:

```
ros2 service list -t
```

* You can check the type details with a command like this:

```
ros2 interface show stcamera_msgs/srv/GetGenICamNodeInfo
```

* Get information for GenICam node "Gain".

```
ros2 service call /stcamera_launch/dev_142100000000/get_genicam_node_info stcamera_msgs/srv/GetGenICamNodeInfo '{genicam_module: "RemoteDevice", genicam_node: "Gain"}'
```

<pre>
waiting for service to become available...
requester: making request: stcamera_msgs.srv.GetGenICamNodeInfo_Request(genicam_module='RemoteDevice', genicam_node='Gain')

response:
stcamera_msgs.srv.GetGenICamNodeInfo_Response(error_info=stcamera_msgs.msg.ErrorInfo(error_code=0, description=''), name='Gain', description='Controls the selected gain as an absolute physical value. This is an amplification factor applied to the video signal.', tool_tip='Controls the selected gain as an absolute physical value.', display_name='Gain', name_space='Standard', interface_type='IFloat', access_mode='Read and Write', is_cachable='Yes', visibility='Beginner', caching_mode='Does not use cache', polling_time=-1, is_streamable=True, is_implemented=True, is_available=True, is_readable=True, is_writable=True, is_feature=True, enum_value_str_list=[], enum_value_int_list=[], current_value='0', min_value='0.000000', max_value='192.000000', increment='', unit='', child_node_list=[])
</pre>

* Reads the value of the GenICam node "Gain" (of type IFloat).<br />

```
ros2 service call /stcamera_launch/dev_142100000000/read_node_float stcamera_msgs/srv/ReadNodeFloat '{genicam_module: "RemoteDevice", genicam_node: "Gain"}'
```

<pre>
waiting for service to become available...
requester: making request: stcamera_msgs.srv.ReadNodeFloat_Request(genicam_module='RemoteDevice', genicam_node='Gain')

response:
stcamera_msgs.srv.ReadNodeFloat_Response(error_info=stcamera_msgs.msg.ErrorInfo(error_code=0, description=''), value=0.0)
</pre>

* Set the GenICam node "Gain" (type IFloat) to 100.<br />

```
ros2 service call /stcamera_launch/dev_142100000000/write_node_float stcamera_msgs/srv/WriteNodeFloat '{genicam_module: "RemoteDevice", genicam_node: "Gain", value: 100}'
```

<pre>
requester: making request: stcamera_msgs.srv.WriteNodeFloat_Request(genicam_module='RemoteDevice', genicam_node='Gain', value=100.0)

response:
stcamera_msgs.srv.WriteNodeFloat_Response(error_info=stcamera_msgs.msg.ErrorInfo(error_code=0, description=''))
</pre>

### 4.2. Get images in free run or trigger mode
The node "grabber" in the stcamera_demos folder uses the StCameraNode to retrieve the image data. There are C++ and Python versions, and the C++ version is code that doubles as a test of how services and topics work, covering a variety of services and topics.

## 5. About Support
OMRON SENTECH does not provide direct support for this package. For questions, glitches, etc., please utilize the issuues on GitHub.

[GenICam GenApi documentation]:http://www.emva.org/wp-content/uploads/GenICam_Standard_v2_0.pdf
[GenICam GenTL documentation]:http://www.emva.org/wp-content/uploads/GenICam_GenTL_1_5.pdf
