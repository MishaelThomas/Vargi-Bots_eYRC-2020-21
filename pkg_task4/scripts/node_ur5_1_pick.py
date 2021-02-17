#! /usr/bin/env python

''' Importing necessary modules for using Moveit Motion Planning framework,
    implementing ROS Action commands and Playing saved trajectories'''

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
<<<<<<< HEAD
import time
import datetime

=======
>>>>>>> origin/main
import yaml
import sys

# Service files are required for implementing Vacuum Gripper
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
<<<<<<< HEAD
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_ros_iot_bridge.msg import msgOrder   
from pkg_task5.msg import dispatch_ship_msg
from pkg_task5.msg import msgur51_to_ur52

task_status=False
=======
from pkg_task4.msg import msgDisOrder
>>>>>>> origin/main

class Ur5_Moveit:

    # Constructor
    def __init__(self):

        # Initialzing the ROS Node
        rospy.init_node('node_ur5_1_pick', anonymous=True)

        # Defining attributes required for Moveit
        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
    
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
<<<<<<< HEAD

        #TESTING CODE
        rospy.Subscriber("/order_to_ur5_1",msgOrder,self.order_callback)        
        self.dispatch_pub=rospy.Publisher("/dispatching_shipping_info",dispatch_ship_msg,queue_size=10)
        self.to_ur52_pub=rospy.Publisher("/ur51_to_ur52",msgur51_to_ur52,queue_size=10)
        #########################################
=======
        
        # Initiating Vacuum Gripper service
>>>>>>> origin/main
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        self.dispatched_order_pub = rospy.Publisher('dispatched_order',msgDisOrder,queue_size=5)
        
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


<<<<<<< HEAD

=======
        # Settings for file path from where we will be playing the saved trajectories files
>>>>>>> origin/main
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/ur5_1_saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # Function to move to a predefined pose in Moveit
    def go_to_predefined_pose(self, arg_pose_name):

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Function to execute a saved trajectory
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan) # Execution of trajectory file

		return ret
    
    # Function to confirm the execution of saved trajectories in few attempts
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts) )
		
		return True

    # Function to pick boxes from shelf and place them on conveyer belt. It uses the package attributes to decide
    # which trajectory is to be played
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
            
            m = pkg_to_pick[8] # Values of m and n are used ahead to decide the trajectory file to be executed
            n = pkg_to_pick[9]

            rospy.logwarn("1. Playing place_to_pkg"+m+n+" Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'place_to_pkg'+m+n+'.yaml',3)

            result = self.gripper_service_call(True)
            rospy.logwarn("1. Playing cp"+m+n+"_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'cp'+m+n+'_place.yaml',3)

            rospy.logwarn("1. Playing pkg"+m+n+"_to_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'pkg'+m+n+'_to_place.yaml',3)
            result = self.gripper_service_call(False)

    #Testing Code##################################################
    def get_time_str(self):
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%d/%m/%Y %H:%M:%S')

        return str_time

    def order_callback(self,order):
        #self.pick_place("packagen31")
        rospy.sleep(10)
        self.dispatch_pub.publish(Order_Id=order.Order_Id,Date_and_Time=self.get_time_str(),task_done="Dispatched")
        self.to_ur52_pub.publish(Order_Id=order.Order_Id,Pkg_color="Red")
    ##################################################

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
   


def main():

<<<<<<< HEAD
    ur5_1 = Ur5Moveit()
    """
    num_pkg_to_pick=9
    pkgs_picked_and_placed = 0
    pkg_to_pick=['packagen31', 'packagen10', 'packagen11', 'packagen12', 'packagen20', 'packagen21', 'packagen30', 'packagen32', 'packagen01']
    
    while(pkgs_picked_and_placed < num_pkg_to_pick and not rospy.is_shutdown()):

        ur5_1.pick_place(pkg_to_pick[pkgs_picked_and_placed])
        pkgs_picked_and_placed = pkgs_picked_and_placed + 1"""
    rospy.spin()
=======
    # Creating the object of Ur5_Moveit class
    ur5_1 = Ur5_Moveit()
    
    # List of packages that are to be picked
    pkg_to_pick=['packagen31', 'packagen10', 'packagen11', 'packagen12', 'packagen20', 'packagen21', 'packagen30', 'packagen32', 'packagen01']
    
    # Using for-loop to implement pick and place one at a time
    for pkg in pkg_to_pick:
        ur5_1.pick_place(pkg)
        msg = msgDisOrder()
        msg.pkg_name = pkg
        ur5_1.dispatched_order_pub.publish(msg)

    # Going to initial position after completing the required task
    ur5_1.go_to_predefined_pose("allZeros")
>>>>>>> origin/main

    # Removing the object of Ur5Moveit Class
    del ur5_1

# main function is called when we execute this python file
if __name__ == '__main__':
    main()

