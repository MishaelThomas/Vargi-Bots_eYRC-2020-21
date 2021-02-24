#! /usr/bin/env python

'''
    This python file is reponsible for directing ur5_2 arm so as to pick incoming packages
    from the conveyor belt and dropping them on their respective bins after sorting.

    node_control_ur5_2.py sets up the ROS node: node_control_ur5_2_and_belt which is responsible
    for directing the ur5_2 arm to pick and place packages according to the messages published
    on ROS message: msgDispatchOrder by ROS node: node_control_ur5_1. It sorts the packages on the
    basis of their color. It uses trajectories stored in folder:
    ur5_2_saved_trajectories ('pkg_task5/config') and executes them using Moveit
    Motion planning Framework. After shipping the package, it informs ROS node:
    node_update_spreadsheets by publishing on ROS message: msgDispathAndShip so that it
    updates the information on Dispatched Orders Spreadsheet using IoT.
'''


# Following messages are used to convey current status of the order:
# msgDispatchOrder: to obtain information about dispatched packages
# from ROS node: node_control_ur5_1
# msgDispatchAndShip: to inform ROS node: node_update_spreadsheets about the shipped order
# so as to update the Shipped Orders Spreadsheet.

import sys
import threading
import datetime
import time

from pkg_task5.msg import msgDispatchAndShip, msgDispatchOrder

# Service files are required for implementing Vacuum Gripper and Conveyor Belt.
# Hence, we can use Ros Service to execute them
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg

# Importing msg file for obtaining the feed of Logical cameras
from pkg_vb_sim.msg import LogicalCameraImage

# Importing necessary modules for using ROS, Moveit Motion Planning framework, threading.

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import yaml

from pyiot import iot

# WORK_DONE object is used to manage the two threads used in cb_exec_sort()
# method of class Ur5_2_Moveit. Its use is explained there in detail.
WORK_DONE = False

# FLAG object is created to initiate the belt after first order is placed,
# for rest of the cases, the belt is controlled by threading
FLAG = True

PKGS_SORTED = 0   # PKGS_SORTED object is used to keep a count of total packages shipped

class Ur52Moveit:
    '''
        Ur52Moveit class contains methods to implement tasks of ur5_2 arm
        i.e. sorting and shipping of packages. Following are the attributes and methods of
        this class

        Attributes:
                    _robot_ns
                    _planning_group
                    _commander
                    _robot
                    _scene
                    _group
                    _display_trajectory_publisher
                    _et_client
                    _planning_frame
                    _eef_link
                    _group_names
                    box_length
                    vacuum_gripper_width
                    delta
                    model
                    ship_spreadsheet_pub
                    gripper_service_call
                    conveyor_belt_service_call
                    _pkg_path
                    _file_path
                    lock

        Methods:
                __init__()
                cb_exec_sort()
                place_pkg()
                control_conveyor_belt()
                cb_capture_model()
                moveit_play_planned_path_from_file()
                moveit_hard_play_planned_path_from_file()
                go_to_pose()
                set_joint_angles()
                hard_set_joint_angles()
                calculate_package_distance()
                pick_pkg()
                get_time()
                __del__()

        Detailed explanation for attributes and methods are given along with their use.
        '''
    # Constructor
    def __init__(self):
        '''
            Constructor of class Ur5_2_Moveit. It does the following:
                1. Sets up the ROS Node: 'node_control_ur5_2_and_belt'
                2. Defines various attributes required by Moveit Motion planning framework
                3. Creates handle for vacuum gripper service, conveyor_belt_service
                4. Creates subscriber for incoming orders ROS message:
                    msgDispatchOrder, LogicalCameraImage
                5. Creates handle for publishing to ROS message: msgDispatchAndShip
                6. Sets up path for accessing saved trajectory files

            Parameters:
                        self: object of class Ur5_2_Moveit

            Return: None
        '''
        # Initializing the ROS node.
        rospy.init_node('node_control_ur5_2_and_belt', anonymous=True)

        # Defining attributes required for MoveIt!
        self._robot_ns = "/ur5_2"
        self._planning_group = "manipulator"
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns +
                                                      "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns +
                                                          "/robot_description", ns=self._robot_ns)

        # Handle to publish planned path on ROS topic '/move_group/display_planned_path'
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns+
                                                             '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=1)
        # Handle to execute planned path using ROS action client
        self._et_client = actionlib.SimpleActionClient(self._robot_ns+
                                                       '/execute_trajectory',
                                                       moveit_msgs.msg.ExecuteTrajectoryAction)
        self._et_client.wait_for_server()

        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = vacuum_gripper_width + (box_length/2) # 0.19

        # Object to store the information sent by logical_camera_2
        self.model = LogicalCameraImage()

        # Subscibing to ROS topic "/eyrc/vb/logical_camera_2"
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.cb_capture_model)

        # Subscribing to ROS Topic: "dispatched_order_to_ur5_2" to obtain dispatched orders
        rospy.Subscriber("dispatched_order_to_ur5_2", msgDispatchOrder, self.cb_exec_sort)

        # Handle for publishing to ROS topic: 'dispatch_ship_info'
        self.ship_spreadsheet_pub = rospy.Publisher("dispatch_ship_info", msgDispatchAndShip,
                                                    queue_size=10)

        # Creating a handle to use Vacuum Gripper service
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', timeout=1)
        self.gripper_service_call = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)

        # Creating a handle to use Conveyor Belt service with desired power
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        self.conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
                                                             conveyorBeltPowerMsg)

        # Setting the file path where the saved trajectory files are stored
        rpack = rospkg.RosPack()
        self._pkg_path = rpack.get_path('pkg_task5')
        self.file_path = self._pkg_path + '/config/ur5_2_saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self.file_path))

        # This object is used to make sure that in later part of code,
        # when we need to update object WORK_DONE in thread1,
        # thread2 is no allowed to access the value of WORK_DONE
        self.lock = threading.Lock()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def cb_exec_sort(self, msg):
        '''
            cb_exec_sort() method is the callback for Subsciber handle of ROS message:
            msgDispatchOrder. It is responsible for sorting the packages according to color.

            Parameters:
                        self: object of class Ur5_2_Moveit
                        msg: message published in ROS topic: 'dispatched_order_to_ur5_2'

            Return: None
        '''

        global WORK_DONE, FLAG

        # Obtaining the color of package from parameter server
        pkg_color = rospy.get_param('pkg_clr/'+str(msg.pkg_name))

        # This block is executed only once, when the dispatch message for first package is published
        # because, in the code FLAG object is set to False and is not updated to True later on
        if FLAG:
            self.conveyor_belt_service_call(100)   # Powering up the belt
            # Waiting till the package is detected by logical_camera_2
            while len(self.model) == 1 and self.model[0].type == 'ur5':
                pass
            # Powering down the belt when the package reaches a desired position
            while self.model[1].pose.position.y > 0.0001:
                pass
            self.conveyor_belt_service_call(0)
            FLAG = False

        # Picking the package from the belt
        self.pick_pkg(self.model[1].pose)

        # Coming back to the intial pick posiiton
        joint_angles = [0.14648733592875196, -2.38179315608067, -0.7257155148810783,
                        -1.6048803093744644, 1.570796326803042, 0.14648733591716212]
        self.hard_set_joint_angles(joint_angles, 3)

        # WORK_DONE is updated to True when ur5_2 arm is done with shipping of the current package
        WORK_DONE = False

        # Two threads are started simultaneously, one for dropping the package to the
        # corresponding bin and other one for controlling the belt so that when the new
        # package arrives, it stops the belt
        thread1 = threading.Thread(target=self.place_pkg, args=(pkg_color, msg.order_id))
        thread2 = threading.Thread(target=self.control_conveyor_belt,
                                   args=(self.model[1].type,))

        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()

    def place_pkg(self, package_color, order_id):
        '''
            place_pkg() method drops the package in the corresponding bin

            Parameters:
                        self: object of class Ur5_1_Moveit
                        package_color: color of the package, which is later used for sorting
                        order_id: Order Id of the package to be shipped
        '''

        global WORK_DONE, PKGS_SORTED

        # Accessing the file according to the color of the package
        # file_1_name contains trajectory of initial position to drop position
        # file_2_name contains trajectory of drop position to initial position
        if package_color == "red":
            file_1_name = 'initial_pose_to_red_bin.yaml'
            file_2_name = 'red_bin_to_initial_pose.yaml'

        elif package_color == "yellow":
            file_1_name = 'initial_pose_to_yellow_bin.yaml'
            file_2_name = 'yellow_bin_to_initial_pose.yaml'

        elif package_color == "green":
            file_1_name = 'initial_pose_to_green_bin.yaml'
            file_2_name = 'green_bin_to_initial_pose.yaml'

        self.moveit_hard_play_planned_path_from_file(self.file_path, file_1_name, 3)

        self.gripper_service_call(False)   # Dropping the package

        # Publishing the message to ROS Topic: "dispatch_ship_info" about the shipped package
        self.ship_spreadsheet_pub.publish(Order_Id=order_id, Date_and_Time=get_time_str(),
                                          task_done="Shipped")

        self.moveit_hard_play_planned_path_from_file(self.file_path, file_2_name, 3)

        # Updating the value of WORK_DONE after shipping
        with self.lock:
            WORK_DONE = True

        PKGS_SORTED += 1   # Incrementing the sorted packages count

    def control_conveyor_belt(self, pkg_picked):
        '''
            control_conveyor_belt() method controls the belt so that it stops the belt
            when a new package is detected by the logical_camera_2.

            Parameters:
                        self: object of class Ur52Moveit
                        pkg_picked: package that is currently  being shipped
        '''

        global WORK_DONE
        self.conveyor_belt_service_call(100)

        # Storing the data of logical_camera_2 for further processing in the while loop
        log_cam_feed = self.model

        # Here, the belt is being controlled with the help of inputs from logical_camera_2
        # When the new package arrives, at that point of time, control jumps out of the
        # while loop. Three condiitons are described below, when the belt should be powered
        # up:
        #    1. No model is being detected
        #    2. ur5 arm is detected
        #    3. ur5 arm and package(currently shipping) is detected
        while len(log_cam_feed) <= 2:

            if len(log_cam_feed) == 0:
                pass
            elif len(log_cam_feed) == 1:
                if log_cam_feed[0].type == "ur5" or log_cam_feed[0].type == pkg_picked:
                    pass
                else:
                    break
            elif len(log_cam_feed) == 2:
                if log_cam_feed[1].type == "ur5" or log_cam_feed[1].type == pkg_picked:
                    pass
                else:
                    break

            log_cam_feed = self.model

        # Stopping the new package at the desired position
        while self.model[1].pose.position.y > 0.0001:
            pass

        self.conveyor_belt_service_call(0)

        # The following while loop is used to tackle the case when the new package arrives earlier
        # than the ur5 arm. It waits till the ur5 arm reaches its initial position after
        # shipping.
        while not WORK_DONE:
            pass

    def cb_capture_model(self, model):
        '''
            cb_capture_model() is the callback function for the subscriber to ROS topic:
            "/eyrc/vb/logical_camera_2". It updates the node with models recently scanned
            by logical camera.

            Parameters:
                        self: object of class Ur52Moveit
                        model: models detected by logical_camera_2

            Return: None
        '''
        self.model = model.models

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''
            moveit_play_planned_path_from_file() method executes trajectory 
            according to the argument passed

            Parameters:
                        self: object of class Ur52Moveit
                        arg_file_path: path where trajectories are stored
                        arg_file_name: trajectory file to be executed

            Return:
                    ret: Bool value from execute() method of move group
        '''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan) # Execution of loaded trajectory

        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name,
                                                arg_max_attempts):
        '''
            moveit_play_planned_path_from_file() method executes trajectory according 
            to the argument passed

            Parameters:
                        self: object of class Ur52Moveit
                        arg_file_path: path where trajectories are stored
                        arg_file_name: trajectory file to be executed
                        arg_max_attempts: maximum number of attempts

            Return:
                    ret: Bool value
        '''
        number_attempts = 0
        flag_success = False

        while ((number_attempts < arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            rospy.logwarn("attempts: {}".format(number_attempts))
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)

        return True

    def go_to_pose(self, arg_pose):
        '''
            go_to_pose() sets the pose, plans and executes the trajectory

            Parameters:
                        self: object of class Ur52Moveit
                        arg_pose: desired position and orientation

            return:
                    flag_plan: confirmation for achieving desired pose
        '''

        flag_plan = False
        attempts = 0

        # Commanding ur5 arm to head towards desired position
        self._group.set_pose_target(arg_pose)

        # Confirming that go_to_pose is executed
        while attempts < 3 and (not flag_plan):
            flag_plan = self._group.go(wait=True)  # wait=False for Async Move
            attempts += 1

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):
        '''
            set_joint_angles() sets the joint angles, plans and executes the trajectory

            Parameters:
                        self: object of class Ur52Moveit
                        arg_list_joint_angles: desired joint angles

            return: 
                    flag_plan: confirmation for achieving desired joint angles
        '''

        self._group.set_joint_value_target(arg_list_joint_angles)
        flag_plan = self._group.go(wait=True)

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''
            hard_set_joint_angles() confirmation for success of set_joint_angles()

            Parameters:
                        self: object of class Ur52Moveit
                        arg_list_joint_angles: desired joint angles
                        arg_max_attempts: maximum attempts permissible

            return: None
        '''

        number_attempts = 0
        flag_success = False

        while ((number_attempts < arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def calculate_pkg_distance(self, package_pose_wrt_camera):
        '''
            calculate_pkg_distance() method provides the exact position of package using the 
            frame of logical_camera_2 

            Parameters:
                        self: object of Ur52Moveit
                        package_pose_wrt_camera: pose of package in logical_camera_2's frame
            Return:
                    pkg_dist_xyz: distance of the package from end-effector along x,y,z axes
        '''

        # Transforming the package position from logical camera's frame
        pose_package_wrt_world = [-0.8+package_pose_wrt_camera.position.z,
                                  package_pose_wrt_camera.position.y,
                                  2-package_pose_wrt_camera.position.x]

        pose_ee_wrt_world = self._group.get_current_pose().pose

        # Distance to be translated by the ur5 arm is determined below
        pkg_dist_xyz = (pose_package_wrt_world[0]-pose_ee_wrt_world.position.x,
                        pose_package_wrt_world[1]-pose_ee_wrt_world.position.y,
                        pose_package_wrt_world[2]-pose_ee_wrt_world.position.z+self.delta)

        return pkg_dist_xyz

    # This function calculates the distance to move and attaches the package
    def pick_pkg(self, package_pose):
        '''
            pick_pkg() picks the package from conveyor belt 

            Parameters:
                        self: object of Ur52Moveit
                        package_pose: pose of package in logical_camera_2's frame

            Return: None
        '''
        x_val, y_val, z_val = self.calculate_pkg_distance(package_pose)

        pose_values = self._group.get_current_pose().pose

        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = pose_values.position.x + x_val
        wpose.position.y = pose_values.position.y + y_val
        wpose.position.z = pose_values.position.z + z_val
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
        self.go_to_pose(wpose)

        # Attaching the package
        self.gripper_service_call(True)

    # Destructor
    def __del__(self):
        '''
            Destructor of class Ur52Moveit
        '''

        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_moveit Deleted." + '\033[0m')

def get_time_str():
    '''
        get_time_str() method provides current time in string format

        Parameters:
                    None

        Return: 
                str_time: current time in string format  
    '''
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%d/%m/%Y %H:%M:%S')
    return str_time

def main():
    '''
        Carries out the task of ur5_2 arm i.e. sorting the packages into corresponding bins
    '''

    # Creating an object of Ur5_moveit class
    ur5_2 = Ur52Moveit()

    global PKGS_SORTED

    # Heading towards intial pick posiiton
    ur5_2.moveit_hard_play_planned_path_from_file(ur5_2.file_path, 'start_to_initial_pose.yaml',
                                                  3)

    # Wait till nine packages are shipped
    while PKGS_SORTED < 9:
        pass

    # Removing the object of Ur5_Moveit class
    del ur5_2

# main() is implemented when we execute this python file
if __name__ == '__main__':
    main()
