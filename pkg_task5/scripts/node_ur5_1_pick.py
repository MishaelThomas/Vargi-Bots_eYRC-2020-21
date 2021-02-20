#! /usr/bin/env python

''' Importing necessary modules for using Moveit Motion Planning framework,
    implementing ROS Action commands and Playing saved trajectories'''

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import yaml
import sys
import time
import datetime

# Service files are required for implementing Vacuum Gripper
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
from pkg_task5.msg import msgDisOrder_to_ur5_2
from pkg_ros_iot_bridge.msg import msgIncOrder
from pkg_task5.msg import dispatch_ship_msg

# This dictionary is created to store the color of packages as decoded using QR code. It is updated later in main()
package_data = {}

exec_list = []
priority_list = []

pkg_count, current, = 0, 0

item_info = { "Medicine":["red",3], "Food":["yellow",2], "Clothes":["green",1]}

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
        
        # Initiating Vacuum Gripper service
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        rospy.Subscriber("incoming_order",msgIncOrder,self.cb_update_exec_dict)

        self.to_ur5_2pub = rospy.Publisher('DispatchedOrder/to_ur5_2',msgDisOrder,queue_size=10)
        self.dispatchOrder_pub=rospy.Publisher("/dispatching_shipping_info",dispatch_ship_msg,queue_size=10)
        
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        # Settings for file path from where we will be playing the saved trajectories files
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/ur5_1_saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def cb_update_exec_dict(self, msg):
        
        global package_data, pkg_count, current, exec_list, priority_list
        
        priority = item_info[msg.Item_type][1]
        pkg = package_data.keys()[package_data.values().index(item_info[msg.Item_type][0])]
        
        del package_data[pkg]

        j = current

        for i in range(current,len(priority_list)):
            if priority_list[i] >= priority:
                j += 1
            else:
                break
        
        priority_list.insert(j,priority)
        exec_list.insert(j,[pkg,msg.Order_Id])

        print(exec_list)
        pkg_count += 1
        

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

        m = pkg_to_pick[8] # Values of m and n are used ahead to decide the trajectory file to be executed
        n = pkg_to_pick[9]
       
        rospy.logwarn("1. Playing place_to_pkg"+m+n+" Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'place_to_pkg'+m+n+'.yaml',3)

        result = self.gripper_service_call(True)

        if(pkg_to_pick not in ["packagen00","packagen01","packagen02"]):
            
            rospy.logwarn("1. Playing cp"+m+n+"_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'cp'+m+n+'_place.yaml',3)

        rospy.logwarn("1. Playing pkg"+m+n+"_to_place Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'pkg'+m+n+'_to_place.yaml',3)
        
        result = self.gripper_service_call(False)

    def get_time_str(self):
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%d/%m/%Y %H:%M:%S')
        return str_time

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
   


def main():

    # Creating the object of Ur5_Moveit class
    ur5_1 = Ur5_Moveit()
    
    global package_data, pkg_count, current 
    
    package_data = rospy.get_param("pkg_clr")

    print(package_data)   

    ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'home_to_place_pose.yaml',3)
    
    while current <= pkg_count and current < 9:

        if current != pkg_count:
            pkg = exec_list[current][0]
            order_id = exec_list[current][1]

            current += 1
            
            ur5_1.pick_place(pkg)
            
            print(str(pkg) + "dispatched")         # Here code regarding pick and place needs to be substituted       
            dispatch_message = msgDisOrder_to_ur5_2()
            dispatch_message.pkg_name = pkg
            dispatch_message.order_id = order_id
        
            ur5_1.to_ur5_2pub.publish(dispatch_message)
            ur5_1.dispatchOrder_pub.publish(Order_Id=order_id,Date_and_Time= ur5_1.get_time_str(),task_done="Dispatched")
        
            print(dispatch_message)

    # Removing the object of Ur5Moveit Class
    del ur5_1

# main function is called when we execute this python file
if __name__ == '__main__':
    main()

