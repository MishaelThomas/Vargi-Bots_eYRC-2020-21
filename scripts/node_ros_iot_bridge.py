#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgOrder     # Message Class for MQTT Subscription Messages
from pyiot import iot                                 # Custom Python Module to perfrom MQTT Tasks




class IotRosBridge:

    # Constructor
    def __init__(self):
        # Initialize the ROS node
        
        rospy.init_node("node_ros_iot_bridge")
        
        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print(param_config_iot)


        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # spreadsheet_write_node can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self.incoming_order_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub)

        #node_ur51_pick can subscribe to this ROS Topic("/order_to_ur5_1") to get order_Id and item_type of recieved order id
        self.order_to_ur51_pub=rospy.Publisher("/order_to_ur5_1",msgOrder)

        # Subscribe to MQTT Topic (/eyrc/vb/PaThJaPa/orders) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        self._config_mqtt_sub_topic, 
                                                        self._config_mqtt_qos   )
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        #rosnode subscibe to topic turtle_final_pos  on which final position of turtlebot get published
        #self.sub_finalpos=rospy.Subscriber("turtle_final_pos",msgTurtleResult,self.write_to_sheet)
        # Start the Action Server
        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    
    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        print("-----publishing_data-----")
        payload = str(message.payload.decode("utf-8"))
        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)
        rospy.loginfo('recieved')

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload
        payload_dict=eval(payload)
        self.incoming_order_pub.publish(msg_mqtt_sub,queue_size=10)
        self.order_to_ur51_pub.publish(Order_Id=payload_dict["order_id"],Item_type=payload_dict["item"],queue_size=10)

# Main
def main():
    IotRosBridge()
    rospy.spin()



if __name__ == '__main__':
    main()
