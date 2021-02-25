#! /usr/bin/env python


'''
    This python file is reponsible for updating the spreadsheets which consists of Inventory,
    Incoming order, Orders Dispatched and Orders Shipped .

    Inventory sheet is updated by accessing the parameter 'pkg_clr' for determining the
    color of the packages. As, incoming orders are published onto ROS Topic:
    "/ros_iot_bridge/mqtt/sub", Incoming Orders sheet is also updated. Similarly, when message is
    published onto ROS Topic: "dispatch_ship_info", Dispatched Order and Shipping Order sheets
    are also updated.
'''



from collections import OrderedDict as od
from datetime import datetime,timedelta,date
from threading import Thread
import rospy

# msgMqttSub: incoming orders are obtained by subscribing to ROS topic: "/ros_iot_bridge/mqtt/sub"
# msgDispatchAndShip: Information about packages dispatched and shipped are obtained by
# subscribing to ROS Topic: "dispatch_ship_info"
from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_task5.msg import msgDispatchAndShip

from pyiot import iot


# Obtain data related to a particular item from parameter server like cost, estimated time of
# delivery, etc.
ITEMDATA = rospy.get_param("/item_info/")

# URL of the sheet over which data is to be written
URL = "https://script.google.com/macros/s/"+rospy.get_param("config_pyiot/google_apps/spreadsheet_id")+"/exec"

class UpdateSpreadheets:
    '''
        UpdateSpreadheets class contains methods that update the following sheets: Inventory,
        Incoming Orders, Dispatched orders and Shipping Orders.

        Attributes:
                    all_orders
        Methods:
                __init__()
                incoming_orders_sheet()
                dispatched_and_shipped_sheet()

        Detailed explanation for attributes and methods are given along with their use.
    '''

    def __init__(self):
        '''
        Constructor for the UpdateSpreadheets. It initializes ROS node:
        node_update_spreadsheets, attribute all_orders as an empty dictionary and subscribes to
        ROS Topics: "/ros_iot_bridge/mqtt/sub" and "dispatch_ship_info"

        Parameters:
                    self: object of UpdateSpreadheets class

        Return: None
        '''

        rospy.init_node("node_update_spreadsheets", anonymous=True)
        self.all_orders = {}

        # Subscribing to ROS Topics: "/ros_iot_bridge/mqtt/sub" and "dispatch_ship_info"
        rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.incoming_orders,
                         queue_size=5)
        rospy.Subscriber("dispatch_ship_info", msgDispatchAndShip,
                         self.dispatched_and_shipped_sheet, queue_size=5)

    def incoming_orders(self, order):
        '''
            incoming_orders() method updates the Incoming Orders Spreadsheet with incoming order

            Parameters:
                        self: object of UpdateSpreadheets class
                        order: incoming order

            Return: None
        '''

        # Converting incoming order to dictionary
        incoming_order = eval(order.message.decode('utf-8'))
        # Updating all_orders dictionary
        self.all_orders[incoming_order["order_id"]] = incoming_order

        global ITEMDATA, URL

        item = incoming_order["item"]

        # Create message to update spreadsheet
        payload = {"Id" : "IncomingOrders",
                   "Team Id" : "VB#1194",
                   "Unique Id" : "PaThJaPa",
                   "Order Id" : incoming_order["order_id"],
                   "Order Date and Time" : incoming_order["order_time"],
                   "Item" : item,
                   "Priority" : ITEMDATA["priority"][item],
                   "Order Quantity" : incoming_order["qty"],
                   "City" : incoming_order["city"],
                   "Longitude" : incoming_order["lon"],
                   "Latitude" : incoming_order["lat"],
                   "Cost" : ITEMDATA["cost"][item]
                  }

        # Updating Incoming Orders Spreadsheet
        iot.spreadsheet_write(URL, payload=payload)

    def dispatched_and_shipped_sheet(self, order):
        '''
            dispatched_and_shipped_sheet() method updates the Dispatched Orders Spreadsheet
            or Shipping Orders Spreadsheet according to the task performed.

            Parameters:
                        self: object of UpdateSpreadheets class
                        order: Dispatched/Shipping order

            Return: None
        '''

        global ITEMDATA, URL

        order_info = self.all_orders[order.Order_Id]

        # Create first part message to update spreadsheet
        payload1 = {"Team Id":"VB#1194",
                    "Unique Id":"PaThJaPa",
                    "Order Id":order.Order_Id,
                    "City":order_info["city"],
                    "Item":order_info["item"],
                    "Priority":ITEMDATA["priority"][order_info["item"]],
                    "Cost":ITEMDATA["cost"][order_info["item"]]
                   }

        # Second part of message is performed on the basis of task performed: Dispatched/Shipping
        if order.task_done == "Dispatched":
            payload2 = {"Id":"OrdersDispatched",
                        "Dispatch Quantity":"1",
                        "Dispatch Status":"Yes",
                        "Dispatch Date and Time":order.Date_and_Time
                       }
        else:
            delv_margin = ITEMDATA["delivery_margin"][order_info["item"]]
            payload2 = {"Id":"OrdersShipped",
                        "Shipped Quantity":"1",
                        "Shipped Status":"Yes",
                        "Shipped Date and Time":order.Date_and_Time,
                        "Estimated Time of Delivery":estimated_time_delivery(delv_margin,
                                                                             order.Date_and_Time)
                       }

        # Combining the messages
        payload1.update(payload2)

        # Updating Dispatched/Shipping Orders Spreadsheet
        iot.spreadsheet_write(URL, payload=payload1)

def estimated_time_delivery(delivery_margin, shipped_time):
    '''
        estimated_time_delivery() method calculates and returns the date and time of delivery

        Parameters:
                    delivery_margin: time required for delivery
                    shipped_time: date and time of shipping

        Return:
                etd: estimated time of delivery
    '''
    # Estimated time is determined in the desired time format and returned to the calling function
    # after incorporating the delivery margin
    ship_time = datetime.strptime(shipped_time, '%Y-%m-%d %H:%M:%S')
    est_time_of_delivery = ship_time+timedelta(int(delivery_margin))
    etd = datetime.strftime(est_time_of_delivery, "%Y-%m-%d")
    return etd

def update_inventory_sheet():
    '''
        update_inventory_sheet() method updates the Inventory sheet just after the color of
        packages are identified

        Parameters: None

        Return: None
    '''

    global ITEMDATA, URL

    # obtaining the information about color of packages from parameter 'pkg_clr'
    package_data = rospy.get_param("/pkg_clr/")

    sort_data = od(sorted(package_data.items()))
    print sort_data

    today = date.today().strftime("%d/%m%Y")

    # Iterating through the packages data and updating the inventory sheet
    for key, value in sort_data.items():

        pkg_name = key
        pkg_color = value.capitalize()
        storage_number = pkg_name[8:]
        sku = pkg_color[0] + storage_number + today[3:]
        storage_pos = "R" + storage_number[0] + "C" + storage_number[1]
        clr = ITEMDATA["item_pkg_color"]
        item = [k for k in clr.keys() if clr[k] == pkg_color][0]

        parameters = {"Id":"Inventory",
                      "Team Id":"VB#1194",
                      "Unique Id":"PaThJaPa",
                      "SKU":sku,
                      "Storage Number":storage_pos,
                      "Cost":ITEMDATA["cost"][item],
                      "Quantity":"1",
                      "Priority":ITEMDATA["priority"][item],
                      "Item":item
                     }
        # Updating inventory sheet
        iot.spreadsheet_write(URL, payload=parameters)




def main():
    '''
    creates an object of update_spreadsheets class which leads to the creation of
    node_update_spreadheets. This node is kept alive.
    '''
    UpdateSpreadheets()
    rospy.spin()

if __name__ == "__main__":
    # Updating inventory sheet using a thread
    INVENSHEET_THREAD = Thread(target=update_inventory_sheet, args=())
    INVENSHEET_THREAD.start()
    # Inventory sheet gets updated by the thread and other sheets are updated according to
    # the publications in their respective ROS Topics, therefore, we call main() here
    main()
