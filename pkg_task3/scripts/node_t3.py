#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.msg import LogicalCameraImage

class Ur5_moveit:

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
        
        self.model_type = ''
        self.model_pose = []

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def cb_capture_model(self, model):
    	if model.models != []:
    		rospy.loginfo('\033[96m' + str(model.models[0].type) + '\033[0m')
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
            '\033[94m' + "Object of class Ur5_moveit Deleted." + '\033[0m')

def place_pkg(ur5,px,py,pz,ox,oy,oz,ow):
	ur5_2_place_pose = geometry_msgs.msg.Pose()
	ur5_2_place_pose.position.x = px
	ur5_2_place_pose.position.y = py
	ur5_2_place_pose.position.z = pz
	ur5_2_place_pose.orientation.x = ox
	ur5_2_place_pose.orientation.y = oy
	ur5_2_place_pose.orientation.z = oz
	ur5_2_place_pose.orientation.w = ow
	ur5.go_to_pose(ur5_2_place_pose)
	rospy.loginfo('\033[96m' + "place pose reached!!" + '\033[0m')
    
def main():
    
    ur5 = Ur5_moveit()
    pkg_list = ['packagen1','packagen2','packagen3']
    
    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,ur5.cb_capture_model)
    
    rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
    gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2) # 0.19
    # Teams may use this info in Tasks
    
    rospy.loginfo('\033[96m' + "BEGIN" + '\033[0m')
    conveyor_belt_service_call(100)
    '''
    rospy.sleep(2)
    pos = ur5.model_pose.position
    ur5_2_pick_pose = geometry_msgs.msg.Pose()
    ur5_2_pick_pose.position.x = -0.8 + pos.z
    ur5_2_pick_pose.position.y = pos.y - 0.005
    ur5_2_pick_pose.position.z = 2 + delta - pos.x
    ur5_2_pick_pose.orientation.x = -0.5
    ur5_2_pick_pose.orientation.y = -0.5
    ur5_2_pick_pose.orientation.z = 0.5
    ur5_2_pick_pose.orientation.w = 0.5
    ur5.go_to_pose(ur5_2_pick_pose)
    rospy.loginfo('\033[96m' + "pick pose reached!!" + '\033[0m')
    		
    rospy.sleep(1)
    gripper_service_call(True)
    place_pkg(ur5,0.817,0.109,0.995,0.0,0.0,0.0,0.0)
    rospy.sleep(1)
    gripper_service_call(False)
    '''
    while not rospy.is_shutdown():
    	if ur5.model_type in pkg_list:
    		
    		curr_pkg = ur5.model_type
    		rospy.sleep(0.5)
    		conveyor_belt_service_call(0)
    		rospy.loginfo('\033[96m' + "STOP" + '\033[0m')
    		pos = ur5.model_pose.position
    		ur5_2_pick_pose = geometry_msgs.msg.Pose()
    		ur5_2_pick_pose.position.x = -0.8 + pos.z
    		ur5_2_pick_pose.position.y = pos.y - 0.05 
    		ur5_2_pick_pose.position.z = 2 + delta - pos.x
    		ur5_2_pick_pose.orientation.x = -0.5
    		ur5_2_pick_pose.orientation.y = -0.5
    		ur5_2_pick_pose.orientation.z = 0.5
    		ur5_2_pick_pose.orientation.w = 0.5
    		ur5.go_to_pose(ur5_2_pick_pose)
    		rospy.loginfo('\033[96m' + "pick pose reached!!" + '\033[0m')
    		rospy.loginfo('\033[96m' + str(ur5_2_pick_pose) + '\033[0m')
    		rospy.loginfo('\033[96m' + str(pos) + '\033[0m')
    		
    		rospy.sleep(1)
    		gripper_service_call(True)
    		rospy.loginfo('\033[96m' + "ATTACHED" + '\033[0m')
    		if curr_pkg == 'packagen1':
    			place_pkg(ur5,0.817,0.109,0.995,-0.5,-0.5,0.5,0.5)
    		elif curr_pkg == 'packagen2':
    			place_pkg(ur5,0.817,0.109,0.995,-0.5,-0.5,0.5,0.5)
    		elif curr_pkg == 'packagen3':
    			place_pkg(ur5,0.817,0.109,0.995,-0.5,-0.5,0.5,0.5)
    		
    		rospy.sleep(1)
    		gripper_service_call(False)
    		rospy.loginfo('\033[96m' + "DETACHED" + '\033[0m')
    		conveyor_belt_service_call(100)
   
    del ur5

if __name__ == '__main__':
    main()

