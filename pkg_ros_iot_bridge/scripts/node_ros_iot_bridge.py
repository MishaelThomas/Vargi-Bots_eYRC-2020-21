'''
    This python file is reponsible for subscribing to the MQTT topic and publish the incoming
    orders to a ROS Topic.

    node_ros_iot_bridge.py initializes the ROS node: node_ros_iot_bridge. This node subscribes to
    the MQTT Topic stored in the parameter server and publishes the incoming order details in the
    ROS Topic: "order_to_ur5_1".
'''

#!/usr/bin/env python

import rospy
# msgMqttSub: incoming orders are published to this ROS Topic: /ros_iot_bridge/mqtt/sub
# msgIncOrder: ROS node: node_ros_iot_bridge publishes the incoming orders to ROS Topic:
# "order_to_ur5_1" for node_control_ur5_1
from pkg_ros_iot_bridge.msg import msgMqttSub, msgIncOrder
from pyiot import iot   # Custom Python Module to perfrom MQTT Tasks

class RosIotBridge:
    '''
        RosIotBridge class contains methods to pass on incoming orders to ROS Topics:
        "order_to_ur5_1".

        Attributes:
                    param_config_iot
                    _config_mqtt_server_url
                    _config_mqtt_server_port
                    _config_mqtt_sub_topic
                    _config_mqtt_qos
                    _config_mqtt_sub_cb_ros_topic
                    inc_spreadsheet_pub
                    order_to_ur5_1_pub
                    ret

        Methods:
                __init__()
                mqtt_sub_callback()

        Detailed explanation for attributes and methods are given along with their use.

    '''

    # Constructor
    def __init__(self):
        '''
            Constructor of class RosIotBridge. It does the following:
                1. Sets up the ROS Node: 'node_control_ur5_2_and_belt'
                2. Defines various attributes required by Moveit Motion planning framework
                3. Creates handle for vacuum gripper service, conveyor_belt_service
                4. Creates subscriber for incoming orders ROS message: msgDispatchOrder,
                                                                        LogicalCameraImage
                5. Creates handle for publishing to ROS message: msgDispatchAndShip
                6. Sets up path for accessing saved trajectory files

            Parameters:
                        self: object of class Ur5_2_Moveit

            Return: None
        '''
        # Initialize the ROS node
        rospy.init_node("node_ros_iot_bridge", anonymous=True)

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print param_config_iot

        # Initialize ROS Topic Publication.  Incoming message from MQTT Subscription will be
        # published on a ROS Topic (/ros_iot_bridge/mqtt/sub). node_update_spreadsheets can
        # subscribe to this ROS Topic: /ros_iot_bridge/mqtt/sub to get messages from MQTT
        # Subscription.
        self.inc_spreadsheet_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                                   msgMqttSub, queue_size=10)

        # node_control_ur5_1 can subscribe to this ROS Topic: "order_to_ur5_1"
        # to get order_Id and item_type of recieved order
        self.order_to_ur5_1_pub = rospy.Publisher("order_to_ur5_1", msgIncOrder, queue_size=9)

        # Subscribe to MQTT Topic: /eyrc/vb/PaThJaPa/orders which is defined in
        # 'config_iot_ros.yaml'. self.mqtt_sub_callback() function will be called
        # when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)

        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        rospy.loginfo("Started ROS-IoT Bridge.")


    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, message):
        '''
            mqtt_sub_callback() is the callback function for MQTT Topic: /eyrc/vb/PaThJaPa/orders.
            This method is reponsible for publishing the incoming orders to ROS Topics:
            /ros_iot_bridge/mqtt/sub and "order_to_ur5_1"
        '''

        print "----------- publishing incoming order ------------"
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)
        rospy.loginfo('recieved')

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        # Publishing to ROS Topic: /ros_iot_bridge/mqtt/sub
        self.inc_spreadsheet_pub.publish(msg_mqtt_sub)

        # Converting the incoming message from string to dictionary for publishing to ROS Topic:
        # "order_to_ur5_1"
        payload_dict = eval(payload)
        self.order_to_ur5_1_pub.publish(Order_Id=payload_dict["order_id"],
                                        Item_type=payload_dict["item"])

# Main
def main():
    '''
        Creates an objectof RosIotBrdige class and keeps the node active
    '''

    RosIotBridge()
    rospy.spin()

if __name__ == '__main__':
    main()
