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
        
        box_length = 0.2               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = vacuum_gripper_width + (box_length/2) # 0.19
        
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
			
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

    # Function: cb_capture_model() is the callback function for the subscriber to ROS topic "/eyrc/vb/logical_camera_2".
    # It updates the node with models recently scanned by logical camera
    '''def cb_capture_model(self, model):
        self.model=model
    	if model.models != [] and model.models
    		rospy.loginfo('\033[96m' + str(model.models[0].type) + '\033[0m')
    		self.model_type = model.models[0].type
    		self.model_pose = model.models[0].pose'''

    
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        #self._group.set_planning_time(99)
        waypoints = []
        
        pose_values = self._group.get_current_pose().pose

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(pose_values)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        #wpose.position.z = waypoints[0].position.z + (trans_z)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        '''wpose.orientation.x = pose_values.orientation.x
        wpose.orientation.y = pose_values.orientation.y
        wpose.orientation.z = pose_values.orientation.z
        wpose.orientation.w = pose_values.orientation.w'''
        wpose.orientation.x = 0.707
        wpose.orientation.y = -0.0
        wpose.orientation.z = -0.707
        wpose.orientation.w = 0.00735


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))
        print('------------waypoints ------------------')
        print(waypoints)
        tries=100
        attempts=0
        fraction=0.0
        # 5. Compute Cartesian Path connecting the waypoints in the list of waypointis
        while fraction<1 and attempts<tries:
            (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0,True)
            print("-----fraction---")
            print(fraction)
            print("---printed")
            attempts=attempts+1
        # Jump Threshold
        
        if(fraction>=1.0):
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)
            print('------------plan ------------------')
            print(plan.joint_trajectory.points)
            if (num_pts >= 3):
                del plan.joint_trajectory.points[0]
                del plan.joint_trajectory.points[1]
            #del plan.joint_trajectory.points[2]
            self._group.execute(plan,wait=True)
        else:
            rospy.loginfo("could not compute successfully")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        """num_pts = len(plan.joint_trajectory.points)
        print('------------plan ------------------')
        print(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]
            #del plan.joint_trajectory.points[2]
        
        print('------------modified plan ------------------')
        print(plan.joint_trajectory.points)
        
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self._display_trajectory_publisher.publish(display_trajectory)

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan,wait=True)"""
        
        print('------------Final pose of EE------------------')
        pose_ee_wrt_world=self._group.get_current_pose().pose
        print(pose_ee_wrt_world)

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

    def pick_pkg(self,package_pose):
        #self.conveyor_belt_service_call(0)
        x,y,z=self.calculate_cartesian_path(package_pose)        
        self.ee_cartesian_translation(x,y,z)
        #self.gripper_service_call(True)

    def init_pose(self,joint_angles):
        #joint_angles=[0.13686832396868986, -2.3780854418447985, -0.8477707268506842, -1.4858327222534857, 1.5697997509806312, 0.13785032539154152]
        
        self._group.go(joint_angles,wait=True)
    
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
    
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
              
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_moveit Deleted." + '\033[0m')

    '''def place_pkg(self):
        self.init_pose()
        rospy.sleep(0.1)
        self.gripper_service_call(False)
        #self.gripper_service_call(False)
        #self.conveyor_belt_service_call(27)
        #self.init_pose()'''

def main():
    
    # Creating an object of Ur5_moveit class
    ur5 = Ur5_moveit()
    #ur5.go_to_predefined_pose("straightUp")
    #ur5.go_to_predefined_pose("allZeros")
    '''pose_values = ur5._group.get_current_pose().pose
    print(pose_values)
    joint_values=ur5._group.get_current_joint_values()
    print(joint_values)'''
    '''STRAIGHT UP
    position: 
  x: 0.0952678525115
  y: 0.109172185771
  z: 1.85633654969
orientation: 
  x: -0.000102824460388
  y: 0.707365978999
  z: 6.18392670866e-05
  w: 0.706847478143

    lst_joint_angles_1=[-2.7869520887002244, -1.644141704569205, 1.982549083095181, 2.8052613005103098, -0.35381797364769074, 3.138701773800692]
    ur5.init_pose(lst_joint_angles_1)'''
    #ur5.pick_pkg([0,6.589954,1.647499])
    #ur5.ee_cartesian_translation(0.817242,0.108936,0.944310)
    
    '''wpose = geometry_msgs.msg.Pose()
    wpose.position.x = 0.232207335449
    wpose.position.y = -0.214327922571
    wpose.position.z = 1.89485233305
    wpose.orientation.x = -0.706811211785
    wpose.orientation.y = -5.59655056263e-05
    wpose.orientation.z = 0.707402220467
    wpose.orientation.w = 7.89934095612e-05
    ur5.go_to_pose(wpose)'''
    
    '''wpose = geometry_msgs.msg.Pose()
    wpose.position.x = 0.0
    wpose.position.y = -0.220046
    wpose.position.z = 1.647499
    wpose.orientation.x = -0.706811211785
    wpose.orientation.y = 0.0
    wpose.orientation.z = 0.707402220467
    wpose.orientation.w = 0.0
    ur5.go_to_pose(wpose)'''
    wpose=ur5._group.get_current_pose().pose
    #wpose = geometry_msgs.msg.Pose()
    #wpose.position.x = -0.28
    #wpose.position.y = -0.218
    #wpose.position.z = 1.19
    wpose.orientation.x =0.707
    wpose.orientation.y = -0.0
    wpose.orientation.z = -0.707
    wpose.orientation.w = 0.007353
    ur5.go_to_pose(wpose)

    """x,y,z=ur5.calculate_cartesian_path([-0.28,-0.218,1.19])
    ur5.ee_cartesian_translation(x,y,z)
    pose_values = ur5._group.get_current_pose().pose"""
    #print("-----current pose-----")
    #print(pose_values)
    """
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = pose_values.position.x+x
    wpose.position.y = pose_values.position.y+y
    wpose.position.z = pose_values.position.z+z
    wpose.orientation.x =0.707
    wpose.orientation.y = -0.0
    wpose.orientation.z = -0.707
    wpose.orientation.w = 0.007353
    print("-----wpose-----")
    print(wpose)
    ur5.go_to_pose(wpose)"""
    """ur5.gripper_service_call(True)
    ur5._group.go([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],wait=True)"""
    """ur5.go_to_pose(wpose)
    ur5.gripper_service_call(Fals)"""
    #ur5.conveyor_belt_service_call(50)
    #ur5.init_pose()
    
    package_pose = [(0.28,-0.2185,1.867),
    		(-0.280,-0.2185,1.64),
    		(0.00,-0.2185,1.42),
    		(0.28,-0.2185,1.19),
    		(-0.28,-0.2185,1.867),
    		(0.0,-0.2185,1.64),
    		(0.28,-0.2185,1.42),
    		(0,-0.2185,1.19),
    		(0.00,-0.2185,1.867),
    		(0.28,-0.2185,1.64),
    		(-0.28,-0.2185,1.42),
    		(-0.28,-0.2185,1.19)]
    
<<<<<<< HEAD
    #package pose are in serial wise first four are red_boxes(0 to 3),then yellow,then green
    joint_values=[-2.0939635597777393, -2.039775269802985, 1.7916300486682637, 0.23670112837617197, 1.0380217784575692, -1.5646336761168156]

    x,y,z=package_pose[10]

        #pose_values = ur5._group.get_current_pose().pose

    wpose = geometry_msgs.msg.Pose()
    wpose.position.x =  x
    wpose.position.y =  y
    wpose.position.z =  z
    wpose.orientation.x = 0.707
    wpose.orientation.y = -0.0
    wpose.orientation.z = -0.707
    wpose.orientation.w = 0.007353
    ur5.go_to_pose(wpose)
    joint_value=ur5._group.get_current_joint_values()
    joint_values.append(joint_value)
    print(joint_value)
    
    
    #ur5._group.go(joint_values,wait=True)
    ur5.gripper_service_call(True)
    touch_links = ur5._robot.get_link_names(group=ur5._planning_group)
    ur5._scene.attach_box(ur5._eef_link,"green_pkg.obj_3", touch_links = touch_links)
    ur5._group.set_planning_time(99)
    ur5.init_pose()
    ur5.gripper_service_call(False)
    
    """
    for cuurent_pkg_pose in joint_values:

    	#ur5.init_pose()
    	#ur5.pick_pkg(current_pkg_pose)
    	#ur5.place_pkg()
    	x,y,z=current_pkg_pose
=======
    '''x,y,z=ur5.calculate_cartesian_path([-0.28,6.589954,1.427499])
    pose_values = ur5._group.get_current_pose().pose
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = pose_values.position.x + x
    wpose.position.y = pose_values.position.y + y
    wpose.position.z = pose_values.position.z + z
    wpose.orientation.x = -0.9999997
    wpose.orientation.y = 0
    wpose.orientation.z = 0
    wpose.orientation.w = 0.0007963
    ur5.go_to_pose(wpose)'''
    '''joint_angles=[0.13686832396868986, -2.3780854418447985, -0.8477707268506842, -1.4858327222534857, 1.5697997509806312, 0.13785032539154152]
        
    ur5.gripper_service_call(True)
    rospy.sleep(1)
    #ur5.conveyor_belt_service_call(50)
    ur5.init_pose(joint_angles)
    ur5.gripper_service_call(False)
    ur5.conveyor_belt_service_call(50)
    rospy.sleep(1)
    ur5.conveyor_belt_service_call(0)'''
    '''package_pose = [[0.28,6.59,1.917499],
    		[0,6.59,1.917499],
    		[-0.28,6.59,1.917499],
    		[0.28,6.589954,1.647499],
    		[0,6.589954,1.647499],
    		[-0.28,6.589954,1.647499],
    		[0.28,6.589954,1.427499],
    		[0,6.589954,1.427499],
    		[-0.28,6.589954,1.427499],
    		[0.28,6.589954,1.197499],
    		[0,6.589954,1.197499],
    		[-0.28,6.589954,1.197499]]
    
    joint_values = []

    for current_pkg_pose in package_pose:
    	
    	x,y,z=ur5.calculate_cartesian_path(current_pkg_pose)
>>>>>>> af904abc59dda901ddfea39d191851f6a7e6ce18
    	
        #pose_values = ur5._group.get_current_pose().pose
        
        wpose = geometry_msgs.msg.Pose()
<<<<<<< HEAD
        wpose.position.x =  x  
        wpose.position.y =  y
        wpose.position.z =  z
        wpose.orientation.x = 0.707
        wpose.orientation.y = -0.0
        wpose.orientation.z = -0.707
        wpose.orientation.w = 0.007353
        ur5.go_to_pose(wpose)
        joint_value=ur5._group.get_current_joint_values()
        joint_values.append(joint_value)
        #ur5._group.go(joint_value,wait=True)
=======
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
>>>>>>> af904abc59dda901ddfea39d191851f6a7e6ce18

    print(joint_values)'''

    print(joint_values)"""
    """
    # List of packages that we want to pick
    pkg_list = ['packagen1','packagen2','packagen3']
    
    # Handle for subscibing to ROS topic "/eyrc/vb/logical_camera_2"
    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,ur5.cb_capture_model)
    
    # Creating a handle to use Vacuum Gripper service
    rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
    gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    
    # Creating a handle to use Conveyor Belt service with desired power
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2) # 0.19
    
    rospy.loginfo('\033[96m' + "BEGIN" + '\033[0m')
    conveyor_belt_service_call(100)
    
    while not rospy.is_shutdown():    	
    	# In case, one of the packages is detected
    	if ur5.model_type in pkg_list:   
    		
    		# curr_pkg: a variable that stores details of detected package
    		curr_pkg = ur5.model_type
    		
    		# Breaking the power supply of conveyor belt
    		rospy.sleep(0.5)
    		conveyor_belt_service_call(0)
    		rospy.loginfo('\033[96m' + "STOP" + '\033[0m')
    		
    		# Determining the pick pose for ur5 arm using the pose of package obtained from logical camera
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
    		
    		# Activating the vacuum gripper
    		rospy.sleep(1)
    		gripper_service_call(True)
    		rospy.loginfo('\033[96m' + "ATTACHED" + '\033[0m')
    		
    		# Placing the packages in assigned bins
    		if curr_pkg == 'packagen1':
    			ur5.place_pkg("Red")
    		elif curr_pkg == 'packagen2':
    			ur5.place_pkg("Green")
    		elif curr_pkg == 'packagen3':
    			ur5.place_pkg("Blue")
    		
    		# Deactivating the gripper
    		rospy.sleep(1)
    		gripper_service_call(False)
    		rospy.loginfo('\033[96m' + "DETACHED" + '\033[0m')
    		
    		# Resuming the power supply of conveyor belt
    		conveyor_belt_service_call(100)
   
    # Removing the object of Ur5_moveit class"""
    del ur5

if __name__ == '__main__':
    main()
    