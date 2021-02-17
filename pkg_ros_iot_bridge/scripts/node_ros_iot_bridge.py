#!/usr/bin/env python

# ROS Node - ROS IoT Bridge

import rospy 
from pkg_ros_iot_bridge.msg import msgIncOrder           # Message Class for MQTT Subscription Messages

from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks

class RosIotBridge:

    # Constructor
    def __init__(self):

        rospy.init_node('node_ros_iot_bridge')  

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print(param_config_iot)

        self.incoming_order_pub = rospy.Publisher('incoming_order',msgIncOrder,queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                self._config_mqtt_server_url, 
                                                self._config_mqtt_server_port, 
                                                self._config_mqtt_sub_topic, 
                                                self._config_mqtt_qos   
                                            )
        
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")
        
        rospy.loginfo("Started ROS-IoT Bridge Node.")

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):

            incoming_order_message = eval(message.payload.decode("utf-8"))
        
            print("[MQTT SUB CB] Message: ", incoming_order_message)
            print("[MQTT SUB CB] Topic: ", message.topic)
            rospy.loginfo('MQTT Message recieved')

            order_message = msgIncOrder()
            order_message.order_id = incoming_order_message["order_id"]
            order_message.item_type = incoming_order_message["item"]  
            self.incoming_order_pub.publish(order_message)
            
 
# Main
def main():    

    RosIotBridge()
    rospy.spin()

if __name__ == '__main__':
    main()
