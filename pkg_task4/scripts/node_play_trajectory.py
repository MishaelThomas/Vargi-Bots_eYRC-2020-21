#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy

from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_moveit_eg6', anonymous=True)

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

        self._group.set_planning_time(99)

        # Allow some leeway in position (meters) and orientation (radians)
        

        rospy.wait_for_service('//eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        box_name = self._box_name
        scene = self._scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    joint_angles=[0.14655978301275052, -2.4608101683915473, -1.0175133809253598, -1.1476540717685673, 1.5579328111748776, 0.1060079478849465]
    ur5._group.go(joint_angles,wait=True)
    
    print(ur5._group.get_current_joint_values())

    rospy.logwarn("1. Playing place_to_pkg10 Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'place_to_pkg12.yaml')

    result = ur5.gripper_service_call(True)

    rospy.logwarn("1. Playing cp10_place Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'cp12_place.yaml')

    rospy.logwarn("1. Playing place_to_pkg10 Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'pkg12_to_place.yaml')

    result = ur5.gripper_service_call(False)

    del ur5



if __name__ == '__main__':
    main()


