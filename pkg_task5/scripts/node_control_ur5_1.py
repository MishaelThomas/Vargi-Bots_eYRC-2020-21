#! /usr/bin/env python
'''
    This python file is reponsible for directing ur5_1 arm so as to pick desired packages
    from the shelf and drop them on the conveyor belt.

    node_control_ur5_1.py sets up the ROS node: node_control_ur5_1 which is responsible for
    directing the ur5_1 arm to pick and place packages according to the messages published
    on ROS Topic: "order_to_ur5_1" by ROS node: node_ros_iot_bridge. It sorts the incoming
    orders according to their priorities, then it uses trajectories stored in folder:
    ur5_1_saved_trajectories ('pkg_task5/config') and executes them using Moveit
    Motion planning Framework. After dispatching the package, it informs ROS node:
    node_control_ur5_2_and_belt by publishing on the ROS Topic: 'dispatched_order_to_ur5_2'.
    Also, it publishes on ROS Topic: 'dispatch_ship_info' so that ROS node:
    node_update_spreadsheets can update the information on Orders Dispatched Spreadsheet using
    IoT.
'''

# Importing necessary modules for using ROS, Moveit Motion Planning framework.

import sys
import time
import datetime
from collections import OrderedDict as od
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg
import yaml

# Following messages are used to convey current status of the order:
# msgDispatchOrder: to inform ROS node: node_control_ur5_2_and_belt about the dispatched order
#                   so as to sort out accordingly.
# msgDispatchAndShip: to inform ROS node: node_update_spreadsheets about the dispatched order
#                     so as to update the Orders Dispatched Spreadsheet.
# msgIncOrder: provides the details of incoming order from ROS node: node_ros_iot_bridge

from pkg_task5.msg import msgDispatchOrder, msgDispatchAndShip
from pkg_ros_iot_bridge.msg import msgIncOrder

# Service files are required for implementing Vacuum Gripper
from pkg_vb_sim.srv import vacuumGripper

# Dictionary PACKAGE_DATA is created to store the color of packages as decoded using QR code.
# It is updated later in cb_update_exec_list()
PACKAGE_DATA = od()

EXEC_LIST = []   # This list stores the packages to be dispatched
PRIORITY_LIST = []   # This list stores the priorities which is further used for sorting the
                     # incoming order

# PKG_COUNT and CURRENT are used to iterate through EXEC_LIST and select package to be dispatched.
# Also, PKG_COUNT keeps a count of total number of packages processed.
PKG_COUNT, CURRENT = 0, 0

class Ur51Moveit:
    '''
        Ur51Moveit class contains methods to implement tasks of ur5_1 arm that is,
        picking from the shelf and placing them on conveyor belt.

        Following are the attributes and methods of this class

        Attributes:
                    _robot_ns
                    _planning_group
                    _commander
                    _robot
                    _scene
                    _group
                    _display_trajectory_publisher
                    _et_client
                    gripper_service_call
                    disp_ur5_2_pub
                    disp_spreadsheet_pub
                    _pkg_path
                    _file_path

        Methods:
                __init__()
                moveit_play_planned_path_from_file()
                moveit_hard_play_planned_path_from_file()
                pick_place()
                __del__()

        Detailed explanation for attributes and methods are given along with their use.
    '''

    # Constructor
    def __init__(self):
        '''
            Constructor of class Ur51Moveit. It does the following:
                1. Sets up the ROS Node: 'node_control_ur5_1'
                2. Defines various attributes required by Moveit Motion planning framework
                3. Creates handle for vacuum gripper service
                4. Creates subscriber for incoming orders from ROS Topic: 'order_to_ur5_1'
                5. Creates handles for publishing to ROS Topics: 'dispatch_ship_info' and
                    'dispatched_order_to_ur5_2'
                6. Sets up path for accessing saved trajectory files

            Parameters:
                        self: object of class Ur51Moveit

            Return: None
        '''

        # Initialzing the ROS Node
        rospy.init_node('node_control_ur5_1', anonymous=True)

        # Defining attributes required for Moveit
        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns
                                                      + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns
                                                          + "/robot_description",
                                                          ns=self._robot_ns)
        # Handle to publish planned path on ROS topic '/move_group/display_planned_path'
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns
                                                             + '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=1)
        # Handle to execute planned path using ROS action client
        self._et_client = actionlib.SimpleActionClient(self._robot_ns +
                                                       '/execute_trajectory',
                                                       moveit_msgs.msg.ExecuteTrajectoryAction)
        self._et_client.wait_for_server()

        # Initiating Vacuum Gripper service
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        # Subscribing to ROS topic: 'order_to_ur5_1' to obtain incoming orders
        rospy.Subscriber("order_to_ur5_1", msgIncOrder, cb_update_exec_list)

        # Handle for publishing to ROS topic: 'dispatched_order_to_ur5_2'
        self.disp_ur5_2_pub = rospy.Publisher('dispatched_order_to_ur5_2',
                                              msgDispatchOrder,
                                              queue_size=10)

        # Handle for publishing to ROS topic: 'dispatch_ship_info'
        self.disp_spreadsheet_pub = rospy.Publisher("dispatch_ship_info",
                                                    msgDispatchAndShip,
                                                    queue_size=10)

        # Setting the file path where the saved trajectory files are stored
        rpack = rospkg.RosPack()
        self._pkg_path = rpack.get_path('pkg_task5')
        self.file_path = self._pkg_path + '/config/ur5_1_saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self.file_path))

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''
            moveit_play_planned_path_from_file() method executes trajectory according to the
            argument passed.

            Parameters:
                        self: object of class Ur51Moveit
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
            moveit_play_planned_path_from_file() method executes trajectory according to the
            argument passed.

            Parameters:
                        self: object of class Ur51Moveit
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

    def pick_place(self, pkg_to_pick):
        '''
            pick_place() method picks the package given in argument from shelf and drops it in
            the conveyor belt.

            Parameters:
                        self: object of class Ur51Moveit
                        pkg_to_pick: package to be picked and placed in the belt
        '''

        # Values of m and n are used ahead to decide the trajectory file to be executed
        row = pkg_to_pick[8]
        col = pkg_to_pick[9]

        rospy.logwarn("1. Playing place_to_pkg"+row+col+" Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self.file_path,
                                                     'place_to_pkg'+row+col+'.yaml', 3)

        self.gripper_service_call(True)   # Picking the package

        # Extra trajectory file is required for some packages
        if(pkg_to_pick in ["packagen20", "packagen22", "packagen30", "packagen31", "packagen32"]):

            rospy.logwarn("1. Playing cp"+row+col+"_place Trajectory File")
            self.moveit_hard_play_planned_path_from_file(self.file_path,
                                                         'cp'+row+col+'_place.yaml', 3)

        rospy.logwarn("1. Playing pkg"+row+col+"_to_place Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self.file_path,
                                                     'pkg'+row+col+'_to_place.yaml', 3)

        self.gripper_service_call(False)   # Dropping the package

    # Destructor
    def __del__(self):
        '''
            Destructor of class Ur51Moveit
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur51Moveit Deleted." + '\033[0m')

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

def cb_update_exec_list(msg):
    '''
        cb_update_exec_list() method is the callback for Subsciber handle of
        ROS Topic: "order_to_ur5_1". It is responsible for sorting the incoming order
        according to priority and updating the list: EXEC_LIST

        Parameters:
                    msg: message published in ROS topic: 'order_to_ur5_1'

        Return: None
    '''

    global PACKAGE_DATA, PKG_COUNT, CURRENT, EXEC_LIST, PRIORITY_LIST
    item_info = {"Medicine":[3, 'red'], "Food":[2, 'yellow'], "Clothes":[1, 'green']}

    # Obtaining the priority in numerical form for incoming order
    priority = item_info[msg.Item_type][0]
    pkg_color = item_info[msg.Item_type][1]
    # Obtaining a suitable package according to the incoming order
    pkg = PACKAGE_DATA.keys()[PACKAGE_DATA.values().index(pkg_color)]
    # current chosen package is deleted from PACKAGE_DATA
    del PACKAGE_DATA[pkg]

    ind = CURRENT   # ind object contains the index until which ur5_1 has done dispatching

    # Here, we implement the sorting algorithm. The PRIORITY_LIST is iterated till the
    # current priority is greater than or equal to the priority of the element in the list.
    # It breaks out of the loop when the condition becomes false and we obtain the index
    # where the package shpuld be inserted.
    for var in range(CURRENT, len(PRIORITY_LIST)):
        if PRIORITY_LIST[var] >= priority:
            ind += 1
        else:
            break

    # Once the desired index is obtained, we update PRIORITY_LIST and EXEC_LIST
    PRIORITY_LIST.insert(ind, priority)
    EXEC_LIST.insert(ind, [pkg, msg.Order_Id])
    PKG_COUNT += 1

def main():
    '''
        Carries out the task of ur5_1 arm i.e. picking from the shelf and placing on the
        conveyor belt.
    '''

    # Creating the object of Ur51Moveit class
    ur5_1 = Ur51Moveit()

    global PACKAGE_DATA, PKG_COUNT, CURRENT

    # Obtaining the information regarding colors of package from parameter 'pkg_clr' and
    # arranging it in an ordered manner
    PACKAGE_DATA = od(sorted(rospy.get_param("/pkg_clr/").items()))
    print PACKAGE_DATA

    # Directing ur5_1 arm to its dropping position
    ur5_1.moveit_hard_play_planned_path_from_file(ur5_1.file_path, 'home_to_place_pose.yaml', 3)

    # Following loop is executed to dispatch packages according to the contents of list:
    # EXEC_LIST. After 9 packages are dispatched, the loop terminates
    while CURRENT < 9:

        # Whenever a package is added to list: EXEC_LIST, value of PKG_COUNT is also
        # incremented (as seen in cb_update_exec_list() method of Ur51Moveit class)
        # This causes the following block to be executed as condition turns out to be false
        if CURRENT != PKG_COUNT:
            # obtaining the package to be dispatched and its Order ID
            pkg = EXEC_LIST[CURRENT][0]
            order_id = EXEC_LIST[CURRENT][1]

            CURRENT += 1

            ur5_1.pick_place(pkg)   # executing pick and place

            # Message is pushed to ROS Topic: "dispatch_ship_info" about the dispatched package
            ur5_1.disp_spreadsheet_pub.publish(Order_Id=order_id,
                                               Date_and_Time=get_time_str(),
                                               task_done="Dispatched")

            dispatch_message = msgDispatchOrder()
            dispatch_message.pkg_name = pkg
            dispatch_message.order_id = order_id
            ur5_1.disp_ur5_2_pub.publish(dispatch_message)

            print dispatch_message

    # Removing the object of Ur51Moveit Class
    del ur5_1

# main function is called when we execute this python file
if __name__ == '__main__':
    main()
