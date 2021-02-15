#!/usr/bin/env python
import rospy
from pyiot import iot
from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgOrder
from pkg_task5.msg import dispatch_ship_msg

class update_spreadheets:
    def __init__(self):
        rospy.init_node("node_update_spreadsheets")
        self.item_data={"Medicine":{"Priority":"HP","Cost":"250","Delivery Time":"1"},"Food":{"Priority":"MP","Cost":"150","Delivery Time":"3"},"Clothes":{"Priority":"LP","Cost":"100","Delivery Time":"5"}}
        self.all_orders={}
        self.URL= "https://script.google.com/macros/s/AKfycbx8MaUzOnAjYPUw2zmVLqMjHoFyk9s6PTeuVijQ7O4kghb1rTD-VF3UPw/exec"
        rospy.init_node("node_update_spreadsheets")
        rospy.Subscriber("/ros_iot_bridge/mqtt/sub",msgMqttSub,self.incoming_orders,queue_size=5)
        rospy.Subscriber("/dispatching_shipping_info",dispatch_ship_msg,self.dispatched_and_shipped_sheet,queue_size=5)

    def incoming_orders_sheet(self,order):
        Item=order["item"]
        payload={"Id":"IncomingOrders","Team Id":"VB#1194","Unique Id":"PaThJaPa","Order Id":order["order_id"],"Order Date and Time":order["order_time"],"Item":Item,"Priority":self.item_data[Item]["Priority"],"Order Quantity":order["qty"],"City":order["city"],"Longitude":order["lon"],"Latitude":order["lat"],"Cost":self.item_data[Item]["Cost"]}
        iot.spreadsheet_write(self.URL,payload=payload)


    def appendTo_all_orders(self,order):
        Order_Id=order["order_id"]
        self.all_orders[Order_Id]=order
    
    def incoming_orders(self,order):
        incoming_order=eval(order.message.decode('utf-8'))
        self.appendTo_all_orders(incoming_order)
        self.incoming_orders_sheet(incoming_order)
    
    def dispatched_and_shipped_sheet(self,order):
        print(self.all_orders)
        Order_Id=order.Order_Id
        order_info=self.all_orders[Order_Id]
        payload1={"Team Id":"VB#1194","Unique Id":"PaThJaPa","Order Id":Order_Id,"City":order_info["city"],"Item":order_info["item"],"Priority":self.item_data[order_info["item"]]["Priority"],"Cost":self.item_data[order_info["item"]]["Cost"]}
        if order.task_done=="Dispatched":
            payload2={"Id":"OrdersDispatched","Dispatch Quantity":"1","Dispatch Status":"Yes","Dispatch Date and Time":order.Date_and_Time}
        else:
            payload2={"Id":"OrdersShipped","Shipped Quantity":"1","Shipped Status":"Yes","Shipped Date and Time":order.Date_and_Time,"Estimated Time of Delivery":self.item_data[order_info["item"]]["Delivery Time"]}
        payload1.update(payload2)
        iot.spreadsheet_write(self.URL,payload=payload1)

def main():
    update_spreadheets()
    rospy.spin()
   

if __name__=="__main__":
    main()