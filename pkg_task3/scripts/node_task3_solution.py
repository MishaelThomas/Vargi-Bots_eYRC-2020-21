#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.msg import LogicalCameraImage

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_task3_solution', anonymous=True)
        
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
       
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self.model_type = ''
        self.model_pose = []

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


    def cb_capture_model(self, model):
    	if model != None:
    		self.model_type = model.models[0].type
    		self.model_pose = model.models[0].pose

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():
    
    ur5 = CartesianPath()
    
    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,ur5.cb_capture_model)
    
    rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
    gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    # Teams may use this info in Tasks
    
    conveyor_belt_service_call(100)
    
    while not rospy.is_shutdown():
    	if ur5.model_type == 'packagen1' or ur5.model_type == 'packagen2' or ur5.model_type == 'packagen3':
    		
    		rospy.sleep(0.5)
    		conveyor_belt_service_call(0)
    		pos = ur5.model_pose.position
    		ur5_2_home_pose = geometry_msgs.msg.Pose()
    		ur5_2_home_pose.position.x = -0.8 + pos.z
    		ur5_2_home_pose.position.y = pos.y
    		ur5_2_home_pose.position.z = 2 + delta - pos.x
    		ur5_2_home_pose.orientation.x = -0.5
    		ur5_2_home_pose.orientation.y = -0.5
    		ur5_2_home_pose.orientation.z = 0.5
    		ur5_2_home_pose.orientation.w = 0.5
    		ur5.go_to_pose(ur5_2_home_pose)
    		rospy.loginfo('\033[96m' + "Home pose reached!!" + '\033[0m')
    		
    		rospy.sleep(0.5)
    		gripper_service_call(True)
    		
    		ur5_2_place_pose = geometry_msgs.msg.Pose()
    		ur5_2_place_pose.position.x = 0.817
    		ur5_2_place_pose.position.y = 0.109
    		ur5_2_place_pose.position.z = 0.994
    		ur5_2_place_pose.orientation.x = -0.0
    		ur5_2_place_pose.orientation.y = -0.0
    		ur5_2_place_pose.orientation.z = 0.0
    		ur5_2_place_pose.orientation.w = 0.0
    		ur5.go_to_pose(ur5_2_place_pose)
    		rospy.loginfo('\033[96m' + "place pose reached!!" + '\033[0m')
    		
    		rospy.sleep(0.5)
    		gripper_service_call(False)
    		
    		conveyor_belt_service_call(100)
   
    del ur5

if __name__ == '__main__':
    main()

