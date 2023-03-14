import rclpy
from rclpy.node import Node
import time
import os
from stcamera_msgs.msg import *
from stcamera_msgs.srv import *
from sensor_msgs.msg import Image

NODE_NAMESPACE_NAME="/stcamera_launch"
NODE_NAME = "/stcameras"
TRIGGER_SELECTOR = "FrameStart"
TRIGGER_SOURCE = "Software"

class Grabber(Node):
    def __init__(self):
        super().__init__('grabber')

    # Get SentechSDK and GenTL information
    def displaySDKInfo(self):
        global NODE_NAMESPACE_NAME
        global NODE_NAME
        try:
            name = NODE_NAMESPACE_NAME + NODE_NAME + "/get_sdk_info"
            client = self.create_client(GetSDKInfo, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting ...")
            req = GetSDKInfo.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            print(future.result())
        except:
            self.get_logger().error("Grabber::displaySDKInfo got exception")

    # Get one connected device's namespace
    def getConnectedDeviceNamespace(self):
        global NODE_NAMESPACE_NAME
        global NODE_NAME
        try:
            name = NODE_NAMESPACE_NAME + NODE_NAME + "/get_device_list"
            client = self.create_client(GetDeviceList, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting ...")
            req = GetDeviceList.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            output = (x.device_namespace for x in res.device_list if x.connected)
            for dev_ns in output:
                return dev_ns
        except:
            self.get_logger().error("Grabber::getConnectedDeviceNamespace got exception")
        return ""

class StCamera(Node):
    def __init__(self, dev_ns_in):
        super().__init__('stcamera')
        self.dev_ns = dev_ns_in
        self.grabbed_images = 0
        self.sent_triggers = 0

        # Subscribe to the connected device
        video_qos = rclpy.qos.QoSProfile(depth=10)
        # video_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        video_qos.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(Image, NODE_NAMESPACE_NAME + "/" + dev_ns_in + "/image_raw", self.imageCallback, video_qos)

    # Check error
    def checkError(self, error_info):
        if (error_info.error_code != 0):
            self.get_logger().error(error_info.description)
        return error_info.error_code

    # Set Gain
    def setGain(self, value):
        global NODE_NAMESPACE_NAME
        try:
            name = NODE_NAMESPACE_NAME + "/" + self.dev_ns + "/write_node_float"
            client = self.create_client(WriteNodeFloat, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting ...")
            req = WriteNodeFloat.Request()
            req.genicam_module = "RemoteDevice"
            req.genicam_node = "Gain"
            req.value = value
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            return self.checkError(res.error_info)
        except:
            self.get_logger().error("StCamera::setGain got exception")
        return -1

    # Get Gain
    def getGain(self):
        global NODE_NAMESPACE_NAME
        try:
            name = NODE_NAMESPACE_NAME + "/" + self.dev_ns + "/get_genicam_node_info"
            client = self.create_client(GetGenICamNodeInfo, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting ...")
            req = GetGenICamNodeInfo.Request()
            req.genicam_module = "RemoteDevice"
            req.genicam_node = "Gain"
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            return {"value": float(res.current_value), 
                    "min_value":float(res.min_value),
                    "max_value":float(res.max_value)}
        except:
            self.get_logger().error("StCamera::getGain got exception")
        return -1

    # Image callback
    def imageCallback(self, msg):
        self.grabbed_images = self.grabbed_images + 1
        print("Grabbed %d images. WxH: %d x %d Encoding %s" %(
                self.grabbed_images, msg.width, msg.height, msg.encoding))

    # Get the status of image acquisition
    def isImageAcquisitionEnabled(self):
        global NODE_NAMESPACE_NAME
        try:
            name = NODE_NAMESPACE_NAME + "/" + self.dev_ns + "/get_image_acquisition_status"
            client = self.create_client(GetImageAcquisitionStatus, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting ...")
            req = GetImageAcquisitionStatus.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            print('isImageAcquisitionEnabled')
            return res.value
        except:
            self.get_logger().error("StCamera::isImageAcquisitionEnabled got exception")
        return -1

    # Set the status of image acquisition
    def setImageAcquisition(self, enabled):
        global NODE_NAMESPACE_NAME
        try:
            name = NODE_NAMESPACE_NAME + "/" + self.dev_ns + "/enable_image_acquisition"
            client = self.create_client(EnableImageAcquisition, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting ...")
            req = EnableImageAcquisition.Request()
            req.value = enabled
            if(enabled):
                self.grabbed_images = 0
                self.sent_triggers = 0
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            return self.checkError(res.error_info)
        except:
            self.get_logger().error("StCamera::setImageAcquisition got exception")
        return -1

    # Set trigger
    def setTrigger(self, enabled):
        global NODE_NAMESPACE_NAME
        global TRIGGER_SELECTOR
        global TRIGGER_SOURCE
        try:
            name = NODE_NAMESPACE_NAME + "/" + self.dev_ns + "/enable_trigger"
            client = self.create_client(EnableTrigger, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting ...")
            req = EnableTrigger.Request()
            req.trigger_selector = TRIGGER_SELECTOR
            req.value = enabled
            req.trigger_source= TRIGGER_SOURCE
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            return self.checkError(res.error_info)
        except:
            self.get_logger().error("StCamera::setTrigger got exception")
        return -1

    # Send trigger
    def sendTrigger(self):
        global NODE_NAMESPACE_NAME
        global TRIGGER_SELECTOR
        try:
            name = NODE_NAMESPACE_NAME + "/" + self.dev_ns + "/send_soft_trigger"
            client = self.nodeForTrigger.create_client(SendSoftTrigger, name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.nodeForTrigger.get_logger().info("service not available, waiting ...")
            req = SendSoftTrigger.Request()
            req.trigger_selector = TRIGGER_SELECTOR
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.nodeForTrigger, future)
            self.sent_triggers = self.sent_triggers + 1
            print("Sent %d triggers. " % self.sent_triggers)
            res = future.result()
            return self.checkError(res.error_info)
        except:
            self.get_logger().error("StCamera::sendTrigger got exception")
        return -1

    def controlGainTest(self):
        # Read gain value
        cur_gain = self.getGain()
        gain_value = cur_gain["value"]
        new_gain = round((cur_gain["min_value"] + cur_gain["max_value"])/2,0)
        print("Min gain: %.2f" %(cur_gain["min_value"]))
        print("Max gain: %.2f" %(cur_gain["max_value"]))
        print("Current gain: %.2f" %(gain_value))

        # Change gain value 
        if (self.setGain(new_gain) == 0):
            print("Update gain to %.2f was successful" %(new_gain))

        # Set the original gain value
        self.setGain(gain_value)

    def start_free_run_mode(self):
        self.setImageAcquisition(False)
        self.setTrigger(False)
        if (not self.isImageAcquisitionEnabled()):
            self.setImageAcquisition(True)
    def start_trigger_mode(self):
        self.nodeForTrigger = Node('trigger')
        print("Enable trigger selector: %s, source: %s" %(TRIGGER_SELECTOR, TRIGGER_SOURCE))
        try:
            self.setImageAcquisition(False)
            self.setTrigger(True)
            self.setImageAcquisition(True)
            self.create_timer(0.1, self.sendTrigger)
        except:
            pass

def startGrab(isFreeRun):
    # Display SDK info
    grabber = Grabber()
    grabber.displaySDKInfo()

    # Check connected devices
    dev_ns = ""
    while(True):
        dev_ns = grabber.getConnectedDeviceNamespace()
        if (dev_ns != ""):
            break
        time.sleep(1)

    if (dev_ns != ""):
        print("Device: %s" %dev_ns)
        
        stcamera = StCamera(dev_ns)

        stcamera.controlGainTest()

        if(isFreeRun):
            #Free run mode
            stcamera.start_free_run_mode()

        else:
            # Trigger mode
            stcamera.start_trigger_mode()

        rclpy.spin(stcamera) 

#        stcamera.destroy_node()
    
#    grabber.destroy_node()



# Main 
def main(args=None):
    rclpy.init(args=args)

    isFreeRun = True
    startGrab(isFreeRun)

#    rclpy.shutdown()


if __name__ == '__main__':
    main()
