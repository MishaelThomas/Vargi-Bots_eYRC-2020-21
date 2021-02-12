import rospy
from pyiot import iot
from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgOrder

class update_spreadheets():
    def __init__():
        item_data={"Medicine":{"Priority":"HP","Cost":"250","Delivery Time":"1"},"Food":{"Priority":"MP","Cost":"150","Delivery Time":"3"},"Clothes":{"Priority":"LP","Cost":"100","Delivery Time":"5"}}
        all_orders={}
        URL= ""
        rospy.init_node("/node_update_spreadsheets")
        rospy.Subscriber("/ros_iot_bridge/mqtt/sub",msgMqttSub,self.incoming_orders,queue_size=5)
        rospy.Subscriber("/dispatching_shipping_info",msgDispatched_Shipped,self.dispatched_and_shipped_sheet,queue_size=5)

    def incoming_orders_sheet(self,order):
         Item=order["item"]
         payload={"Id":"IncomingOrders","Team Id":"VB#1194","Unique Id":"PaThJaPa","Order Id":order["order_id"],"Order Date and Time":order["order_time"],"Item":Item,"Priority":self.item_data[Item]["Priority"],"Order Quantity":order["qty"],"City":order["city"],"Longitude":order["lon"],"Latitude":order["lat"],"Cost":self.item_data[Item]["Cost"]}
         iot.spreadsheet_write(self.URL,payload)

    def appendTo_all_orders(self,order):
        Order_Id=order["order_id"]
        del order["order_id"]
        self.all_orders[Order_Id]=order
    
    def incoming_orders(self,order):
        incoming_order=eval(order.message.decode('utf-8'))
        self.incoming_orders_sheet(incoming_order)
        self.appendTo_all_orders(incoming_order)
    
    def dispatched_and_shipped_sheet(self,order):
        Order_Id=order.order_id
        order_info=self.all_orders[Order_Id]
        payload1={"Team Id":"VB#1194","Unique Id":"PaThJaPa","Order Id"=Order_Id,"City":order_info["city"],"Item":order_info["item"],"Priority":self.item_data[order_info["item"]]["Priority"],"Cost":self.item_data[orders_info["item"]]["Cost"]}
        if task_done=="Dispatched":
            payload2={"Id":"OrdersShipped","Dispatch Quantity":"1","Dispatch Status":"Yes","Dispatch Date and Time":order.date_and_time}
            payload1.update(payload2)
        else:
            payload2={"Id":"OrdersShipped","Shipped Quantity":"1","Shipped Status":"Yes","Shipped Date and Time":order.date_and_time,"Estimated Time of Delivery":self.item_data[order_info["item"]]["Delivery Time"]}
            payload1.update(payload2)
        iot.spreadsheet_write(self.URL,payload1)