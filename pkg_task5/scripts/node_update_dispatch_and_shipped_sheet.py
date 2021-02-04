import rospy
from pkg_task5.msg import dispatch_ship_msg
from pyiot import iot
from pkg_iot_ros_bridge.msg import msgMqttSub  

all_orders=[]
def cb_incoming_order(order_data):
        global all_orders
        incoming_order=eval(order_data.message.decode('utf-8')) #a dict containing whole data of incoming order
        all_orders.append(incoming_order)

def cb_update_sheet(data):
    global all_orders
    item_data={"Red":{"item_type":"Medicine","Priority":"HP","Cost":"250"},"Yellow":{"item_type":"Food","Priority":"MP","Cost":"150"},"Green":{"item_type":"Clothes","Priority":"LP","Cost":"100"}}
    Delivery_time={"HP":"1","MP":"3","LP":"5"}
    URL_order_dispatched=""
    URL_order_shipped=""
    order=[k for k in all_orders if k["order_id"]==data.Order_Id]
    Priority_and_Cost=[ [item_data[key]["Priority"],item_data[key]["Cost"]] for key in item_data.keys() if item_data[key]["item_type"]==order["item"]]
    if (data.Shipped):
        request= iot.spreadsheet_write(URL_order_dispatched,Id="OrdersDispatched",Team_Id="VB#1194",Unique_Id="PaThJaPa",Order_Id=data.Order_Id,City=order[0]["city"],Item=order[0]["item"],Priority=Priority_and_Cost[0][0],Dispatch_Quantity="1",Cost=Priority_and_Cost[0][1],Dispatch_Status="Yes",Dispatch_Date_and_Time=data.date_and_time)
    elif (data.Dispatched):
        request=iot.spreadsheet_write(URL_order_shipped,Id="OrdersShipped",Team_Id="VB#1194",Unique_Id="PaThJaPa",Order_Id=data.Order_Id,City=order[0]["city"],Item=order[0]["item"],Priority=Priority_and_Cost[0][0],Shipped_Quantity="1",Cost=Priority_and_Cost[0][1],Shipped_Status="Yes",Shipped_Date_and_Time=data.date_and_time,Estimated_Time_of_Delivery=str(int(data.date_and_time[0:2])+int(Delivery_time[Priority_and_Cost[0][0]]))+"-"+data.date_and_time[3:5]+"-"+data.date_and_time[6:10])
    if(request=="success"):
        rospy.loginfo("Spreadsheet_write_is_success")
    else:
        rospy.loginfo("Spreadsheet_write_is_unsuccess")

def main():
    rospy.init_node("node_update_dispatch_shipped_sheet")
    rospy.Subscriber('/ros_iot_bridge/mqtt/sub',msgMqttSub,cb_incoming_order)
    rospy.Subscriber("/order_shipped_diapatched_info",dispatch_ship_msg,callback=cb_update_sheet)
    rospy.spin()

if __name__ == '__main__':
    main()