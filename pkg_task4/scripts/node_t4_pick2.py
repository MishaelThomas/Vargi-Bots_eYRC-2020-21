#! /usr/bin/env python


# Importing required modules, msg files, srv files and so on
import rospy
import sys
import copy
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

# Service files are required for implementing Vacuum Gripper and Conveyor Belt. Hence, we can use Ros Service to execute them
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse

# Importing msg file for obtaining the feed of Logical camera
from pkg_vb_sim.msg import LogicalCameraImage

class Ur5_moveit:

    # Constructor
    def __init__(self):

        # Initializing the ROS node.
        rospy.init_node('node_task4_pick_arm', anonymous=True)
        
        # Defining attributes required for MoveIt!
        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        
        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = vacuum_gripper_width + (box_length/2) # 0.19
        
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Allow replanning to increase the odds of a solution
        #self._group.allow_replanning(True)
        self._group.set_planning_time(99)

        # Allow some leeway in position (meters) and orientation (radians)
        self._group.set_goal_position_tolerance(0.01)
        self._group.set_goal_orientation_tolerance(0.1)
			
        # Initializing the attributes obtained from Logical camera
        #self.model=LogicalCameraImage()
        # Handle for subscibing to ROS topic "/eyrc/vb/logical_camera_2"
        #rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,self.cb_capture_model)
    
        # Creating a handle to use Vacuum Gripper service
        rospy.wait_for_service('//eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        
    
        # Creating a handle to use Conveyor Belt service with desired power
        #rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        #self.conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

        self._box_name = 'packagen21'
        self._box_pose = geometry_msgs.msg.PoseStamped()
        
        self._box_pose.header.frame_id = "world"	
        self._box_pose.pose.position.x = 0
        self._box_pose.pose.position.y = 6.589954 - 7
        self._box_pose.pose.position.z = 1.427499
        self._box_pose.pose.orientation.w = 1.0

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

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def calculate_cartesian_path (self,package_pose_wrt_world):
       
        print('------------------Initial pose of EE------------------')
        pose_ee_wrt_world=self._group.get_current_pose().pose
        print(pose_ee_wrt_world)
        
        cartesian_path=(package_pose_wrt_world[0]-pose_ee_wrt_world.position.x, 
        				package_pose_wrt_world[1]-pose_ee_wrt_world.position.y-7+self.delta, 
        				package_pose_wrt_world[2]-pose_ee_wrt_world.position.z)
        
        print("---------Cartesian path--------------")
        print(cartesian_path)
        
        return cartesian_path

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
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

    def init_pose(self):
        joint_angles=[0.13686832396868986, -2.3780854418447985, -0.8477707268506842, -1.4858327222534857, 1.5697997509806312, 0.13785032539154152]
        self._group.go(joint_angles,wait=True)
    
    def pick_pkg(self,package_pose):
        #self.conveyor_belt_service_call(0)
        x,y,z = self.calculate_cartesian_path(package_pose)
        self.ee_cartesian_translation(x,y,z)
        #self.gripper_service_call(True)
                
	# Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5_moveit Deleted." + '\033[0m')
                    
def main():
    
    # Creating an object of Ur5_moveit class
    ur5 = Ur5_moveit()

	# Adding the box to the scene
    #ur5._scene.add_box(ur5._box_name,ur5._box_pose, size=(0.15, 0.15, 0.15))
    
    '''package_pose = [
    		[0.28,6.689954,1.647499],
    		[0,6.689954,1.647499],
    		[-0.28,6.689954,1.647499],
    		[0.28,6.689954,1.427499],
    		[0,6.689954,1.427499],
    		[-0.28,6.689954,1.427499],
    		[0.28,6.689954,1.197499],
    		[0,6.689954,1.197499],
    		[-0.28,6.689954,1.197499]]'''
    
    package_pose = [
    		[-0.28,6.689954,1.427499]]

    joint_values = []

    for current_pkg_pose in package_pose:
    	
    	x,y,z=ur5.calculate_cartesian_path(current_pkg_pose)
    	
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

        joint_value=ur5._group.get_current_joint_values()
        joint_values.append(joint_value)

    print(joint_values)
    '''
    joint_values = [[3.0961934425438518, -1.3963754984801797, -1.0265399546726863, -3.0762337959665755, -0.13615785709219352, -0.774448375073403], 
                    [-2.594161603628664, -0.9850697094404168, -1.0762917915013288, 1.9376494913181297, 0.48750919502248813, 0.02489612242544137], 
                    [0.7701424773555852, -1.3229593678177114, -1.569949921495847, -0.28907541687933236, 2.297089879902943, 3.0933279054482377], 
                    [-0.7853496398665101, -1.7825198462401932, 1.6327786172349033, 0.274402621950232, 2.3102420430608746, 0.18039976226303356],
                    [2.9045875238583676, -0.6364245016276779, -1.903785319209736, 2.9853703633234776, -0.19226567771241054, -0.3952494192884446], 
                    [-3.1117644656533665, 0.014097466075142506, -2.149155695206158, -0.16302046871382814, -0.03483056533010753, 2.3187371114251274], 
                    [-0.7462554347364483, -1.7528648202726522, 2.185082828159974, -0.5270300660187441, 2.4257301137190623, -0.052334840847803044], 
                    [2.6720701386007235, -0.6762202761818568, -2.4925834555524267, 3.0379795019802565, -0.45791864489519263, 0.05317379659126864], 
                    [0.7643454228991988, -1.3785890194097448, -2.181436345581874, -2.65148888042412, -2.3237549412893426, -0.04347946372959832]]
    x,y,z=ur5.calculate_cartesian_path([0,6.689954,1.427499])
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
    joint_value_0 = [-2.6011339293255, -2.3590931034680374, 1.8892899715041231, 0.32159706536877586, 0.47790973258301417, 0.036733309660952784]
    ur5._group.go(joint_value_0,wait=True)
    x,y,z=ur5.calculate_cartesian_path([0,6.589954,1.427499])
    ur5.ee_cartesian_translation(x,y,z)
    joint_value_1 = [2.074430144400317, -1.0963980516344831, -2.2479738864668946, 0.2403598626789636, 1.0501801208377302, 3.046324064107367]
    ur5._group.go(joint_value_1,wait=True)

    result = ur5.gripper_service_call(True)
    touch_links = ur5._robot.get_link_names(group=ur5._planning_group)  
    ur5._scene.attach_box(ur5._eef_link,ur5._box_name, touch_links = touch_links)
    print(ur5.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=4))
    ur5.ee_cartesian_translation(0,0.25,0)
    joint_value_1 = [0.14655978301275052, -2.4608101683915473, -1.0175133809253598, -1.1476540717685673, 1.5579328111748776, 0.1060079478849465]
    ur5._group.go(joint_value_1,wait=True)
    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1.19
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5
    ur5.go_to_pose(ur5_2_home_pose)

    # Deactivating the Gripper
    result = ur5.gripper_service_call(False)
    ur5._scene.remove_attached_object(ur5._eef_link, name=ur5._box_name)
    print(ur5.wait_for_state_update(box_is_attached=False, box_is_known=True, timeout=4))

    # Removing the box from planning scene 	
    ur5._scene.remove_world_object(ur5._box_name)'''
  

    del ur5    
    

if __name__ == '__main__':
    main()
    
