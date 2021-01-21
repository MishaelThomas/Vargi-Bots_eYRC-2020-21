#! /usr/bin/env python


# Importing required modules, msg files, srv files and so on
import rospy
import sys
import copy
import threading
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_task4.msg import picked_pkg_info

# Service files are required for implementing Vacuum Gripper and Conveyor Belt. Hence, we can use Ros Service to execute them
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse

# Importing msg file for obtaining the feed of Logical camera
from pkg_vb_sim.msg import LogicalCameraImage
work_done=False
class Ur5_moveit:

    # Constructor
    def __init__(self):

        # Initializing the ROS node.
        rospy.init_node('node_task3_solution', anonymous=True)
        
        # Defining attributes required for MoveIt!
        self._robot_ns="/ur5_2"
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
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
        #Initializing the attributes obtained from topic "ur51/picked_pkg_info"
        self.sent_pkg=picked_pkg_info()

        # Handle for subscibing to ROS topic "/eyrc/vb/logical_camera_2"
        rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,self.cb_capture_model)
        #Handle for subscribimg to ROS topic "ur51/picked_pkg_info"
        rospy.Subscriber("ur51/picked_pkg_info",picked_pkg_info,self.cb_sent_pkg)
        
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

    def cb_sent_pkg(self,data):
        self.sent_pkg=data

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

# function to control conveyer belt
    def control_conveyor_belt(self):
        global work_done
        if (work_done):
            return
        self.conveyor_belt_service_call(100)
        while len(self.model.models) == 0 or len(self.model.models)==2:
            if len(self.model.models)==2:
                for x in self.model.models:
                    if(x.type in pkg_picked):
                        picked_present=True
                        break
                    else:
                        picked_present=False
                if(picked_present):
                    continue
                else:
                    break    
        while len(self.model.models)==1 and self.model.models[0].type=="ur5":
            continue
        rospy.sleep(0.5)
        self.conveyor_belt_service_call(0)
        
        
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
        pose_package_wrt_world=[-0.8+package_pose_wrt_camera.position.z,package_pose_wrt_camera.position.y,2-package_pose_wrt_camera.position.x]
        pose_ee_wrt_world=self._group.get_current_pose().pose
        cartesian_path=(-(pose_ee_wrt_world.position.x-pose_package_wrt_world[0]),-(pose_ee_wrt_world.position.y-pose_package_wrt_world[1]),-(pose_ee_wrt_world.position.z-pose_package_wrt_world[2])+self.delta)
        x,y,z=cartesian_path
        return cartesian_path

    def pick_pkg(self,package_pose):
        x,y,z=self.calculate_cartesian_path(package_pose)
        self.ee_cartesian_translation(x,y,z)
        self.gripper_service_call(True)
        

    def init_pose(self):
        joint_angles=[0.13686832396868986, -2.3780854418447985, -0.8477707268506842, -1.4858327222534857, 1.5697997509806312, 0.13785032539154152]
        self._group.go(joint_angles,wait=True)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_moveit Deleted." + '\033[0m')

    def place_pkg(self,package_color):
        if(package_color=="red"):
            joint_values=[-1.5722165746417147, -2.0156468764887974, -1.4441489746467298, -1.253079896204322, 1.5716701789574419, -1.5731922855980915]
        if (package_color=="green"):
            joint_values=[-0.10597267010129219, -0.9232539830623159, 0.8675774939714938, -1.5159454519701585, -1.5716655689094967, 3.034847263597408]
        if(package_color=="blue"):
            joint_values=[-1.677851795541076, -1.193402632379506, 1.2809171265671475, -1.6590883174733078, -1.5713969000516617, 1.4627511449783928]
        self._group.go(joint_values,wait=True)
        rospy.sleep(0.1)
        self.gripper_service_call(False)
        #self.init_pose()

def main():
    
    # Creating an object of Ur5_moveit class
    global work_done
    ur5=Ur5_moveit()
    while not rospy.is_shutdown():
        ur5.init_pose()
        ur5.conveyor_belt_service_call(100)
        while len(ur5.model.models) <= 1:
            continue
        rospy.sleep(0.5)
        #ur5.conveyor_belt_service_call(0)
        for x in ur5.model.models:
            if x.type != "ur5":
                package=x
                package.type=self.sent_pkg.package_colour
        ur5.conveyor_belt_service_call(0)
        while (work_done):
            if(ur5.sent_pkg.task_done):
                work_done=True
            ur5.pick_pkg(package.pose)
            pkg_picked.append(package.type)
            thread1=threading.Thread(target=ur5.place_pkg,args=(pkg,))
            thread2=threading.Thread(target=ur5.control_conveyor_belt)
            thread1.start()
            thread2.start()
            thread1.join()
            thread2.join()
            for x in ur5.model.models:
                if  x.type != "ur5":
                    package=x

    """ur5 = Ur5_moveit()
    
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
    
    conveyor_belt_service_call(100)
    
    while not rospy.is_shutdown():
    	
    	# In case, one of the packages is detected
    	if ur5.model_type in pkg_list:   
    		
    		# curr_pkg: a variable that stores details of detected package
    		curr_pkg = ur5.model_type
    		
    		# Breaking the power supply of conveyor belt
    		rospy.sleep(0.5)
    		conveyor_belt_service_call(0)
    		
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
    		
    		# Activating the vacuum gripper
    		gripper_service_call(True)
    		
    		# Placing the packages in assigned bins
    		if curr_pkg == 'packagen1':
    			place_pkg(ur5,0.1105,0.65,0.995,0.0,0.0,0.0,0.0)
    		elif curr_pkg == 'packagen2':
    			place_pkg(ur5,0.75,0.00,0.995,0.0,0.0,0.0,0.0)
    		elif curr_pkg == 'packagen3':
    			place_pkg(ur5,0.04,-0.65,0.995,0.0,0.0,0.0,0.0)
    		
    		# Deactivating the gripper
    		gripper_service_call(False)
    		
    		# Resuming the power supply of conveyor belt
    		conveyor_belt_service_call(100)
   
    # Removing the object of Ur5_moveit class
>>>>>>> b056369073f3e034f9454a3ecd4ab020c54f76ba"""
    del ur5

if __name__ == '__main__':
    main()
    
