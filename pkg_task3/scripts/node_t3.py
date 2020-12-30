#! /usr/bin/env python


# Importing required modules, msg files, srv files and so on
import rospy
import sys
import copy

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
        rospy.init_node('node_task3_solution', anonymous=True)
        
        # Defining attributes required for MoveIt!
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = vacuum_gripper_width + (box_length/2) # 0.19
        # Handle to publish planned path on ROS topic '/move_group/display_planned_path'
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        
        # Handle to execute planned path using ROS action client
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.currentpose=self._group.get_current_pose().pose
        # Initializing the attributes obtained from Logical camera
        self.model=LogicalCameraImage()
        # Handle for subscibing to ROS topic "/eyrc/vb/logical_camera_2"
        rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,self.cb_capture_model)
    
        # Creating a handle to use Vacuum Gripper service
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    
        # Creating a handle to use Conveyor Belt service with desired power
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        self.conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    # Function: cb_capture_model() is the callback function for the subscriber to ROS topic "/eyrc/vb/logical_camera_2".
    # It updates the node with models recently scanned by logical camera
    def cb_capture_model(self, model):
        self.model=model
    	"""if model.models != [] and model.models
    		rospy.loginfo('\033[96m' + str(model.models[0].type) + '\033[0m')
    		self.model_type = model.models[0].type
    		self.model_pose = model.models[0].pose"""

    # Function: go_to_pose() controls the ur5 arm and takes it to the provided position and orientation
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Commanding ur5 arm to head towards desired position
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        # Displaying final pose and joint values
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


    
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


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

    def calculate_cartesian_path (self,package_pose_wrt_camera):
        print("------wrt_camera------")
        print(package_pose_wrt_camera)
        pose_package_wrt_world=[-0.8+package_pose_wrt_camera.position.z,package_pose_wrt_camera.position.y,2-package_pose_wrt_camera.position.x]
        print("-------wrt_world-----")
        print(pose_package_wrt_world)
        pose_ee_wrt_world=self._group.get_current_pose().pose
        cartesian_path=(-(pose_ee_wrt_world.position.x-pose_package_wrt_world[0]-self.delta),-(pose_ee_wrt_world.position.y-pose_package_wrt_world[1]),-(pose_ee_wrt_world.position.z-pose_package_wrt_world[2]))
        print(cartesian_path)
        return cartesian_path

    def pick_pkg(self,package_pose):
        self.conveyor_belt_service_call(0)
        x,y,z=self.calculate_cartesian_path(package_pose)
        self.ee_cartesian_translation(x,y,z)
        self.gripper_service_call(True)

    def init_pose(self):
        joint_angles=[2.9850250068587805, 0.2066173353226901, -1.298452655156396, -0.47854773274707796, -1.570387970006058, -0.15562259264570333]
        self._group.go(joint_angles,wait=True)
        #self.ee_cartesian_translation(0,0,0.3)
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_moveit Deleted." + '\033[0m')

    def place_pkg(self,package_name):
        joint_values=[]
        self.ee_cartesian_translation(0,0,0.2)
        self.conveyor_belt_service_call(40)
        if(package_name=="package1"):
            joint_values=[-1.5722165746417147, -2.0156468764887974, -1.4441489746467298, -1.253079896204322, 1.5716701789574419, -1.5731922855980915]
        if (package_name=="package2"):
            joint_values=[-0.10597267010129219, -0.9232539830623159, 0.8675774939714938, -1.5159454519701585, -1.5716655689094967, 3.034847263597408]
        if(package_name=="package3"):
            joint_values=[-1.677851795541076, -1.193402632379506, 1.2809171265671475, -1.6590883174733078, -1.5713969000516617, 1.4627511449783928]
        self._group.go(joint_values,wait=True)
        self.gripper_service_call(False)
        self.init_pose()

        """
        ur5_2_place_pose = geometry_msgs.msg.Pose()
        #self.ee_cartesian_translation(0.0,0.0,1.2)
        if(package_name==""):
            ur5_2_place_pose.position.x = 0.11
            ur5_2_place_pose.position.y = 0.65
            ur5_2_place_pose.position.z = 1.3
            ur5_2_place_pose.orientation.x = -0.5
            ur5_2_place_pose.orientation.y = -0.5
            ur5_2_place_pose.orientation.z = 0.5
            ur5_2_place_pose.orientation.w = 0.5
            self.go_to_pose(ur5_2_place_pose)
            rospy.loginfo('\033[96m' + "place pose reached!!" + '\033[0m')

        elif(package_name=="Green"):
            ur5_2_place_pose.position.x = 0.75
            ur5_2_place_pose.position.y = 0.03
            ur5_2_place_pose.position.z =1.4
            ur5_2_place_pose.orientation.x = -0.5
            ur5_2_place_pose.orientation.y = -0.5
            ur5_2_place_pose.orientation.z = 0.5
            ur5_2_place_pose.orientation.w = 0.5
            self.go_to_pose(ur5_2_place_pose)
            rospy.loginfo('\033[96m' + "place pose reached!!" + '\033[0m')


        elif(package_name=="Blue"):
            ur5_2_place_pose.position.x = 0.04
            ur5_2_place_pose.position.y = -0.65
            ur5_2_place_pose.position.z = 1.4
            ur5_2_place_pose.orientation.x = -0.5
            ur5_2_place_pose.orientation.y = -0.5
            ur5_2_place_pose.orientation.z = 0.5
            ur5_2_place_pose.orientation.w = 0.5
            self.go_to_pose(ur5_2_place_pose)
            rospy.loginfo('\033[96m' + "place pose reached!!" + '\033[0m')
        else :
            rospy.loginfo("package name given is not in correct format.The names are Red ,Green ,Blue")"""

def main():
    
    # Creating an object of Ur5_moveit class
    ur5 = Ur5_moveit()
    #ur5.init_pose()
    ur5.conveyor_belt_service_call(40)
    ur5.init_pose()
    while not rospy.is_shutdown():
        if(len(ur5.model.models)>=2):
            for x in ur5.model.models:
                if x.type != "ur5":
                    package=x
                    print("----package_type-----")
                    print(package)
            #ur5.conveyor_belt_service_call(0)
            
            #ur5.calculate_cartesian_path(x.pose)
            
            ur5.pick_pkg(x.pose)
            ur5.place_pkg(x.type)
        else:
            continue


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
    	 6666663if ur5.model_type in pkg_list:   
    		
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
    
