#!/usr/bin/env python
import rospy
from collections import OrderedDict as od
from datetime import datetime,timedelta,date
from threading import Thread

from pyiot import iot
from pkg_ros_iot_bridge.msg import msgMqttSub, msgIncOrder
from pkg_task5.msg import msgDispatchAndShip


item_data=rospy.get_param("/item_info/")

URL= "https://script.google.com/macros/s/AKfycbx8MaUzOnAjYPUw2zmVLqMjHoFyk9s6PTeuVijQ7O4kghb1rTD-VF3UPw/exec"

class update_spreadheets:

    def __init__(self):
        """
        A class for updating the Inventory Management Sheets
        """
        rospy.init_node("node_update_spreadsheets") #initiating node
        self.all_orders={} #dictionary for storing all the recieved orders
        
        #subscribing to topic(/ros_iot_bridge/mqtt/sub) to get recieved orders from MQTT subscription
        rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.incoming_orders, queue_size = 5)

        #subscribing to topic(dispatch_ship_info) to get dipatched and shipped package data
        rospy.Subscriber("dispatch_ship_info", msgDispatchAndShip, self.dispatched_and_shipped_sheet, queue_size=5)

    def incoming_orders_sheet(self,order):
        """
        A update_spreadsheets class method to update IncomingOrders sheet
        """
        global item_data
        global URL
        
        Item = order["item"]
        
        payload = {
                    "Id" : "IncomingOrders",
                    "Team Id" : "VB#1194",
                    "Unique Id" : "PaThJaPa",
                    "Order Id" : order["order_id"],
                    "Order Date and Time" : order["order_time"],
                    "Item" : Item,
                    "Priority" : item_data["priority"][Item],
                    "Order Quantity" : order["qty"], 
                    "City" : order["city"],
                    "Longitude" : order["lon"],
                    "Latitude" : order["lat"],
                    "Cost" : item_data["cost"][Item]
                    }
        iot.spreadsheet_write(URL, payload = payload)


    def appendTo_all_orders(self,order):
        """
        a update_spreadsheet class method to insert incoming order to a dictionary containing all orders
        """
        Order_Id = order["order_id"] #storing order_id of incoming order in Order_Id varibale
        self.all_orders[Order_Id] = order #inserting a key value pair in all_orders dictionary where key is order_id and value is data of recieved incoming order
    
    def incoming_orders(self,order):
        """
        a callback function to process all the messages published on the topic "/ros_iot_bridge/mqtt/sub"
        """
        incoming_order = eval(order.message.decode('utf-8')) #converting string to dictionary
        self.appendTo_all_orders(incoming_order) # updating all_orders dictionary
        self.incoming_orders_sheet(incoming_order) # updating IncomingOrders spreadsheet
    
    def dispatched_and_shipped_sheet(self,order):
        

        global item_data
        global URL
        print(self.all_orders)
        Order_Id=order.Order_Id
        order_info=self.all_orders[Order_Id]
        payload1={
                  "Team Id":"VB#1194",
                  "Unique Id":"PaThJaPa",
                  "Order Id":Order_Id,
                  "City":order_info["city"],
                  "Item":order_info["item"],
                  "Priority":item_data["priority"][order_info["item"]],
                  "Cost":item_data["cost"][order_info["item"]]
                  }

        if order.task_done=="Dispatched":
            payload2=  {
                        "Id":"OrdersDispatched",
                        "Dispatch Quantity":"1",
                        "Dispatch Status":"Yes",
                        "Dispatch Date and Time":order.Date_and_Time
                        }
                        
        else:
            payload2={
            "Id":"OrdersShipped",
            "Shipped Quantity":"1",
            "Shipped Status":"Yes",
            "Shipped Date and Time":order.Date_and_Time,
            "Estimated Time of Delivery":self.estimate_time_delivery(item_data["delivery_time"][order_info["item"]],order.Date_and_Time)
            }
        payload1.update(payload2)
        iot.spreadsheet_write(URL,payload=payload1)

    def estimate_time_delivery(self,delivery_time,shipped_time):
        ship_time=datetime.strptime(shipped_time,'%d/%m/%Y %H:%M:%S')
        est_time_of_delivery=ship_time+timedelta(int(delivery_time))
        return datetime.strftime(est_time_of_delivery,"%Y-%m-%d")
        

def update_inventory_sheet():
    global item_data
    global URL
    package_data=rospy.get_param("/pkg_clr/")
    print(package_data)
    sort_data=od(sorted(package_data.items()))
    print(sort_data)
    today=date.today().strftime("%d/%m%Y")
    for key,value in sort_data.items():
        pkg_name=key
        pkg_color=value.capitalize()
        storage_number=pkg_name[8:]
        SKU=pkg_color[0]+storage_number+today[3:]
        storage_pos="R"+storage_number[0]+"C"+storage_number[1]
        item=[k for k in item_data["item_pkg_color"].keys() if item_data["item_pkg_color"][k]==pkg_color][0]
        parameters={
                    "Id":"Inventory",
                    "Team Id":"VB#1194",
                    "Unique Id":"PaThJaPa",
                    "SKU":SKU,
                    "Storage Number":storage_pos,
                    "Cost":item_data["cost"][item],
                    "Qunatity":"1",
                    "Priority":item_data["priority"][item],
                    "Item":item
                    }
        iot.spreadsheet_write(URL,payload=parameters)
        



def main():
    update_spreadheets()
    rospy.spin()
   

if __name__=="__main__":
    invensheet_thread=Thread(target=update_inventory_sheet,args=())
    invensheet_thread.start()
    main()