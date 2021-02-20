#!/usr/bin/env python
import rospy
from collections import OrderedDict as od
from datetime import date
from threading import Thread

from pyiot import iot
from pkg_ros_iot_bridge.msg import msgMqttSub, msgIncOrder
from pkg_task5.msg import msgDispatchAndShip


item_data = { "Medicine" : { "Priority" : "HP", "Cost" : "250", "Delivery Time" : "1"},
                  "Food" : { "Priority" : "MP", "Cost" : "150", "Delivery Time" : "3"},
               "Clothes" : { "Priority" : "LP", "Cost" : "100", "Delivery Time" : "5"}
               }

URL= "https://script.google.com/macros/s/AKfycbx8MaUzOnAjYPUw2zmVLqMjHoFyk9s6PTeuVijQ7O4kghb1rTD-VF3UPw/exec"

class update_spreadheets:

    def __init__(self):
        """
        A class for updating the Inventory Management Sheets
        
        """
        rospy.init_node("node_update_spreadsheets")
        self.all_orders={}
        rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.incoming_orders, queue_size = 5)
        rospy.Subscriber("dispatch_ship_info", msgDispatchAndShip, self.dispatched_and_shipped_sheet, queue_size=5)

    def incoming_orders_sheet(self,order):
        global item_data
        global URL
        
        Item = order["item"]
        
        payload = { "Id" : "IncomingOrders",
                    "Team Id" : "VB#1194",
                    "Unique Id" : "PaThJaPa",
                    "Order Id" : order["order_id"],
                    "Order Date and Time" : order["order_time"],
                    "Item" : Item,
                    "Priority" : item_data[Item]["Priority"],
                    "Order Quantity" : order["qty"], 
                    "City" : order["city"],
                    "Longitude" : order["lon"],
                    "Latitude" : order["lat"],
                    "Cost" : item_data[Item]["Cost"]
                    }
        iot.spreadsheet_write(URL, payload = payload)


    def appendTo_all_orders(self,order):
        Order_Id = order["order_id"]
        self.all_orders[Order_Id] = order
    
    def incoming_orders(self,order):
        incoming_order = eval(order.message.decode('utf-8'))
        self.appendTo_all_orders(incoming_order)
        self.incoming_orders_sheet(incoming_order)
    
    def dispatched_and_shipped_sheet(self,order):
        global item_data
        global URL
        print(self.all_orders)
        Order_Id=order.Order_Id
        order_info=self.all_orders[Order_Id]
        payload1={"Team Id":"VB#1194",
                  "Unique Id":"PaThJaPa",
                  "Order Id":Order_Id,
                  "City":order_info["city"],
                  "Item":order_info["item"],
                  "Priority":item_data[order_info["item"]]["Priority"],
                  "Cost":item_data[order_info["item"]]["Cost"]}

        if order.task_done=="Dispatched":
            payload2=  {"Id":"OrdersDispatched",
                        "Dispatch Quantity":"1",
                        "Dispatch Status":"Yes",
                        "Dispatch Date and Time":order.Date_and_Time}
                        
        else:
            payload2={"Id":"OrdersShipped",
            "Shipped Quantity":"1",
            "Shipped Status":"Yes",
            "Shipped Date and Time":order.Date_and_Time,
            "Estimated Time of Delivery":item_data[order_info["item"]]["Delivery Time"]}
        payload1.update(payload2)
        iot.spreadsheet_write(URL,payload=payload1)

def update_inventory_sheet():
    global item_data
    global URL
    item_color={"Red":"Medicine","Green":"Clothes","Yellow":"Food"}
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
        parameters={"Id":"Inventory",
                    "Team Id":"VB#1194",
                    "Unique Id":"PaThJaPa",
                    "SKU":SKU,
                    "Storage Number":storage_pos,
                    "Cost":item_data[item_color[pkg_color]]["Cost"],
                    "Qunatity":"1",
                    "Priority":item_data[item_color[pkg_color]]["Priority"],
                    "Item":item_color[pkg_color]}
        iot.spreadsheet_write(URL,payload=parameters)
        



def main():
    update_spreadheets()
    rospy.spin()
   

if __name__=="__main__":
    invensheet_thread=Thread(target=update_inventory_sheet,args=())
    invensheet_thread.start()
    main()