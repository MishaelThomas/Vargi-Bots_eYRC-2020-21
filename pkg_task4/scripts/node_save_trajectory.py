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

        #self._group.set_planning_time(99)

        self._group.set_goal_position_tolerance(0.001)

        rospy.wait_for_service('//eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        self._box_name = 'packagen32'
        self._box_pose = geometry_msgs.msg.PoseStamped()
        
        self._box_pose.header.frame_id = "world"	
        self._box_pose.pose.position.x = -0.28
        self._box_pose.pose.position.y = 6.589954 - 7
        self._box_pose.pose.position.z = 1.197499
        self._box_pose.pose.orientation.w = 1.0

        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

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

    def calculate_cartesian_path (self,package_pose_wrt_world):
       
        pose_ee_wrt_world=self._group.get_current_pose().pose
        
        cartesian_path=(package_pose_wrt_world[0]-pose_ee_wrt_world.position.x, 
        				package_pose_wrt_world[1]-pose_ee_wrt_world.position.y-7+0.19, 
        				package_pose_wrt_world[2]-pose_ee_wrt_world.position.z)
        
        return cartesian_path
    
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []
        
        pose_values = self._group.get_current_pose().pose

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(pose_values)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y) 
        wpose.position.z = waypoints[0].position.z + (trans_z)
        wpose.orientation.x = -0.9999997
        wpose.orientation.y = 0
        wpose.orientation.z = 0
        wpose.orientation.w = 0.0007963


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")


        self._computed_plan = plan

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        self._computed_plan = plan

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def set_joint_angles(self, arg_list_joint_angles):

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._computed_plan = self._group.plan()
		flag_plan = self._group.go(wait=True)
		print(type(self._computed_plan))

		return flag_plan

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        
        self._group.stop()
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
    
    def go_to_predefined_pose(self, arg_pose_name):

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

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

    
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    ur5._scene.add_box(ur5._box_name,ur5._box_pose, size=(0.15, 0.15, 0.15))
    joint_value_0 = [-2.6011339293255, -2.3590931034680374, 1.8892899715041231, 0.32159706536877586, 0.47790973258301417, 0.036733309660952784]
    ur5._group.go(joint_value_0,wait=True)

    joint_angles=[0.14655978301275052, -2.4608101683915473, -1.0175133809253598, -1.1476540717685673, 1.5579328111748776, 0.1060079478849465]
    ur5.hard_set_joint_angles(joint_angles,3)
    rospy.sleep(0.1)
    #print(ur5._group.get_current_joint_values())
    #lst_joint_angles_1 = [3.0961934425438518, -1.3963754984801797, -1.0265399546726863, -3.0762337959665755, -0.13615785709219352, -0.774448375073403]
    #ur5.set_joint_angles(lst_joint_angles_1)
    #print(ur5._group.get_pose_reference_frame())
    #ur5.go_to_predefined_pose('straightUp')
    x,y,z=ur5.calculate_cartesian_path([-0.28,6.589954,1.197499])
    #ur5.ee_cartesian_translation(x,y,z)
    pose_values = ur5._group.get_current_pose().pose
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = pose_values.position.x + x
    wpose.position.y = pose_values.position.y + y
    wpose.position.z = pose_values.position.z + z
    wpose.orientation.x = -0.9999997
    wpose.orientation.y = 0
    wpose.orientation.z = 0
    wpose.orientation.w = 0.0007963
    ur5.go_to_pose(wpose)
    rospy.sleep(0.5)
    file_name = 'place_to_pkg32.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    
    

    result = ur5.gripper_service_call(True)
    touch_links = ur5._robot.get_link_names(group=ur5._planning_group)  
    ur5._scene.attach_box(ur5._eef_link,ur5._box_name, touch_links = touch_links)
    print(ur5.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=4))

    ur5.ee_cartesian_translation(0,0.25,0)
    rospy.sleep(0.5)
    file_name = 'cp32_place.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    
    
    lst_joint_angles_2 = [0.14655978301275052, -2.4608101683915473, -1.0175133809253598, -1.1476540717685673, 1.5579328111748776, 0.1060079478849465]
    ur5.hard_set_joint_angles(lst_joint_angles_2,3)
    rospy.sleep(0.5)
    file_name = 'pkg32_to_place.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
		yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    result = ur5.gripper_service_call(False)
    ur5._scene.remove_attached_object(ur5._eef_link, name=ur5._box_name)
    print(ur5.wait_for_state_update(box_is_attached=False, box_is_known=True, timeout=4))

    # Removing the box from planning scene 	
    ur5._scene.remove_world_object(ur5._box_name)

    del ur5



if __name__ == '__main__':
    main()


