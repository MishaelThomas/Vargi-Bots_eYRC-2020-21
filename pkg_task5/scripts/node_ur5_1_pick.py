#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from threading import Thread 
import rospkg
import yaml
import os
import math
import time
import sys
import copy
from datetime import date

# Importing modules required for performing functions related to computer vision and QR decoding
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages


# Object of Camera class is used for QR decoding
class Camera():

    # Constructor
    def __init__(self):
        self.bridge = CvBridge()

    # Function to obtain attributes of packages from an image of shelf containing packages
    def get_qr_data(self,data):

        global package_data

        # Obtaining the image in CV2 format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Enhancing the contrast for a clear image
        image=cv_image*1.5999999999999998667732370449812151491641998291015625

        # qr_result object contains the decoded data
        qr_result = decode(image)

        # Using the decoded value to identify color of packages
        # Using the for loop, we iterate through each package and update the dicitionary from the attributes of package
        if ( len( qr_result ) > 0):
            for i in range(0, len(qr_result)):
                if qr_result[i].rect.left in range(125,131):
                    if qr_result[i].rect.top in range(313,317):
                        package_data["packagen00"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(494,499):
                        package_data["packagen10"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(640,645):
                        package_data["packagen20"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(795,800):
                        package_data["packagen30"] = str(qr_result[i].data)
                elif qr_result[i].rect.left in range(313,319):
                    if qr_result[i].rect.top in range(313,317):
                        package_data["packagen01"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(494,499):
                        package_data["packagen11"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(640,645):
                        package_data["packagen21"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(795,800):
                        package_data["packagen31"] = str(qr_result[i].data)

                elif qr_result[i].rect.left in range(499,506):
                    if qr_result[i].rect.top in range(313,317):
                        package_data["packagen02"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(494,499):
                        package_data["packagen12"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(640,645):
                        package_data["packagen22"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(795,800):
                        package_data["packagen32"] = str(qr_result[i].data)


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_ur5_1_pick', anonymous=True)

        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"
        
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
    
        rospy.Subscriber('/ros_iot_bridge/mqtt/sub',msgMqttSub,self.cb_incoming_order)

        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		return ret
    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts) )
		
		return True

    #function to pick boxes from shelf and place them on conveyer belt
    def pick_place(self,pkg_to_pick):
        
        if(pkg_to_pick=="packagen31"):

            rospy.logwarn("1. Playing home_to_pkg31 Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_pkg31.yaml',3)

            result = self.gripper_service_call(True)
            rospy.logwarn("1. Playing cp31_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'cp31_place.yaml',3)

            rospy.logwarn("1. Playing pkg31_to_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'pkg31_to_place.yaml',3)
            result = self.gripper_service_call(False)

        elif(pkg_to_pick=="packagen01"):

            rospy.logwarn("1. Playing place_to_pkg01 Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'place_to_pkg01.yaml',3)

            result = self.gripper_service_call(True)
            rospy.logwarn("1. Playing pkg01_to_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'pkg01_to_place.yaml',3)
            result = self.gripper_service_call(False)
        
        else:
            
            m = pkg_to_pick[8]
            n = pkg_to_pick[9]

            rospy.logwarn("1. Playing place_to_pkg"+m+n+" Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'place_to_pkg'+m+n+'.yaml',3)

            result = self.gripper_service_call(True)
            rospy.logwarn("1. Playing cp"+m+n+"_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'cp'+m+n+'_place.yaml',3)

            rospy.logwarn("1. Playing pkg"+m+n+"_to_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'pkg'+m+n+'_to_place.yaml',3)
            result = self.gripper_service_call(False)
    
    
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
   
def update_inventory_sheet():
    
    global package_data
    global item_info

    URL_inventory = "https://script.google.com/macros/s/AKfycbxB63R6GHpzV8YqhqyVeU_mPOxpCR7ucoZ4DWKKbJgFt5uM4kDhbFio/exec"
    
    for key,value in package_data.items():

        package_colour = value.capitalize()
        package_location = key
        
        request=iot.spreadsheet_write(  URL_inventory,Id = "Inventory",
                                        Team_Id = "VB#1194",
                                        Unique_Id = "PaThJaPa",
                                        SKU = package_colour[0]+package_location[8]+package_location[9]+str("%02d"%date.today().month)+str(date.today().year)[2]+str(date.today().year)[3],
                                        Item = item_info[package_colour]["item_type"],
                                        Priority = item_info[package_colour]["Priority"],
                                        Storage_Number = "R"+package_location[8]+" "+"C"+package_location[9],
                                        Cost = item_info[package_colour]["Cost"],
                                        Quantity = "1"
                                    )

        if(request=="success"):
            print("value is updated in inventory")
        else:
            print("request is unsuccessful")

def cb_incoming_order(order_data):
        
        global item_info

        incoming_order = eval(order_data.incoming_order_message.decode('utf-8')) 
        #a dictionary containing whole data of incoming order
        
        Priority_and_Cost = [ [item_info[key]["Priority"],item_info[key]["Cost"]] for key in item_info.keys() 
                                                                               if item_info[key]["item_type"]==incoming_order["item"]]
        
        URL_incoming_orders = "https://script.google.com/macros/s/AKfycbxB63R6GHpzV8YqhqyVeU_mPOxpCR7ucoZ4DWKKbJgFt5uM4kDhbFio/exec"
    
        request = iot.spreadsheet_write(URL_incoming_orders,Id = "IncomingOrders",
                                        Team_Id = "VB#1194",
                                        Unique_Id = "PaThJaPa",
                                        Order_Id = incoming_order["order_id"],
                                        Order_Date_and_Time = incoming_order["order_time"],
                                        Item = incoming_order["item"],
                                        Priority = Priority_and_Cost[0][0],
                                        Order_Quantity = incoming_order["qty"],
                                        City = incoming_order["city"],
                                        Longitude = incoming_order["lon"],
                                        Latitude = incoming_order["lat"],
                                        Cost = Priority_and_Cost[0][1]
                                        )

        if(request=="success"):
            print("value get updated in incoming")
        else:
            print("request is unsuccessful for incoming")

def main():

    ur5_1 = Ur5Moveit()

    # Creating an object of Camera class
    camera2D = Camera()
    shelf_image=rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image,timeout=None)
    
    # Updating the packages dictionary using QR Decoding
    camera2D.get_qr_data(shelf_image)
    #print(package_data)

    #thread for updating inventory_spreadsheet
    Inventory_sheet_thread = Thread(target = update_inventory_sheet())
    Inventory_sheet_thread.start()
    
    '''num_pkg_to_pick=9
    pkgs_picked_and_placed = 0
    pkg_to_pick=['packagen31', 'packagen10', 'packagen11', 'packagen12', 'packagen20', 'packagen21', 'packagen30', 'packagen32', 'packagen01']
    
    while(pkgs_picked_and_placed < num_pkg_to_pick and not rospy.is_shutdown()):

        ur5_1.pick_place(pkg_to_pick[pkgs_picked_and_placed])
        pkgs_picked_and_placed = pkgs_picked_and_placed + 1

'''
    del ur5_1

    
    """
    ur5._scene.add_box(ur5._box_name,ur5._box_pose, size=(0.15, 0.15, 0.15))


    rospy.logwarn("1. Playing home_to_pkg21 Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'home_to_pkg21.yaml')
    ur5._scene.add_box(ur5._box_name,ur5._box_pose, size=(0.15, 0.15, 0.15))

    rospy.logwarn("1. Playing home_to_pkg21 Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'home_to_pkg21.yaml')


    rospy.logwarn("1. Playing cp21_pick Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'cp21_pick.yaml')

    result = ur5.gripper_service_call(True)
    touch_links = ur5._robot.get_link_names(group=ur5._planning_group)  
    ur5._scene.attach_box(ur5._eef_link,ur5._box_name, touch_links = touch_links)
    print(ur5.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=4))

    rospy.logwarn("1. Playing cp21_place Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'cp21_place.yaml')

    rospy.logwarn("1. Playing pkg21_to_place Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'pkg21_to_place.yaml')

    result = ur5.gripper_service_call(False)
    ur5._scene.remove_attached_object(ur5._eef_link, name=ur5._box_name)
    print(ur5.wait_for_state_update(box_is_attached=False, box_is_known=True, timeout=4))


    # Removing the box from planning scene 	
    ur5._scene.remove_world_object(ur5._box_name)"""



if __name__ == '__main__':
    main()

