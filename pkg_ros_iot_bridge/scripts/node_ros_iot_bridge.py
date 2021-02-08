#!/usr/bin/env python

# ROS Node - ROS IoT Bridge

import rospy 
from threading import Thread
from datetime import date
from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages

from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks


item_info = { 
              "Red":   {"item_type":"Medicine","Priority":"HP","Cost":"250"},
              "Yellow":{"item_type":"Food",    "Priority":"MP","Cost":"150"},
              "Green": {"item_type":"Clothes", "Priority":"LP","Cost":"100"}
            }

# This dictionary is created to store the color of packages as decoded using QR code. It is updated later in main()
package_data = {'packagen00':rospy.get_param('packagen00'),
                'packagen01':rospy.get_param('packagen01'),
                'packagen02':rospy.get_param('packagen02'),
                'packagen10':rospy.get_param('packagen10'),
                'packagen11':rospy.get_param('packagen11'),
                'packagen12':rospy.get_param('packagen12'),
                'packagen20':rospy.get_param('packagen20'),
                'packagen21':rospy.get_param('packagen21'),
                'packagen22':rospy.get_param('packagen22'),
                'packagen30':rospy.get_param('packagen30'),
                'packagen31':rospy.get_param('packagen31'),
                'packagen32':rospy.get_param('packagen32')}

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
            
            global item_info

            incoming_order_message = eval(message.payload.decode("utf-8"))
        
            print("[MQTT SUB CB] Message: ", incoming_order_message)
            print("[MQTT SUB CB] Topic: ", message.topic)
            rospy.loginfo('MQTT Message recieved')

            Priority_and_Cost = [ [item_info[key]["Priority"],item_info[key]["Cost"]] for key in item_info.keys() 
                                                                               if item_info[key]["item_type"]==incoming_order_message["item"]]
        
            URL_incoming_orders = "https://script.google.com/macros/s/AKfycbxB63R6GHpzV8YqhqyVeU_mPOxpCR7ucoZ4DWKKbJgFt5uM4kDhbFio/exec"
    
            request = iot.spreadsheet_write(URL_incoming_orders,
                                            Id = "IncomingOrders",
                                            Team_Id = "VB#1194",
                                            Unique_Id = "PaThJaPa",
                                            Order_Id = incoming_order_message["order_id"],
                                            Order_Date_and_Time = incoming_order_message["order_time"],
                                            Item = incoming_order_message["item"],
                                            Priority = Priority_and_Cost[0][0],
                                            Order_Quantity = incoming_order_message["qty"],
                                            City = incoming_order_message["city"],
                                            Longitude = incoming_order_message["lon"],
                                            Latitude = incoming_order_message["lat"],
                                            Cost = Priority_and_Cost[0][1]
                                            )

            if(request=="success"):
                print("value get updated in incoming")
            else:
                print("request is unsuccessful for incoming")

def update_inventory_sheet():
    
    global package_data
    global item_info

    URL_inventory="https://script.google.com/macros/s/AKfycbxB63R6GHpzV8YqhqyVeU_mPOxpCR7ucoZ4DWKKbJgFt5uM4kDhbFio/exec"
    
    for key,value in package_data.items():
        
        package_colour = value.capitalize()
        package_location = key
        
        request = iot.spreadsheet_write(URL_inventory,
                                        Id="Inventory",
                                        Team_Id="VB#1194",
                                        Unique_Id="PaThJaPa",
                                        SKU=package_colour[0]+package_location[8]+package_location[9]+str("%02d"%date.today().month)+str(date.today().year)[2]+str(date.today().year)[3],
                                        Item=item_info[package_colour]["item_type"],
                                        Priority=item_info[package_colour]["Priority"],
                                        Storage_Number="R"+package_location[8]+" "+"C"+package_location[9],
                                        Cost=item_info[package_colour]["Cost"],
                                        Quantity="1"
                                        )

        if(request=="success"):
            print("value get updated in inventory")
        else:
            print("request is unsuccessful")   
# Main
def main():    

    #thread for updating inventory_spreadsheet
    Inventory_sheet_thread = Thread(target = update_inventory_sheet())
    Inventory_sheet_thread.start() 

    RosIotBridge()
    
    rospy.spin()



if __name__ == '__main__':
    main()
