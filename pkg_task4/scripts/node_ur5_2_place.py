#! /usr/bin/env python


# Importing required modules, msg files, srv files and so on
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import yaml
import threading

# Importing modules required for performing functions related to computer vision and QR decoding
'''import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode'''

# Service files are required for implementing Vacuum Gripper and Conveyor Belt. Hence, we can use Ros Service to execute them
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse

# Importing msg file for obtaining the feed of Logical cameras
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task4.msg import msgDisOrder

work_done = False
flag = True
pkgs_sorted = 0

# This dictionary is created to store the color of packages as decoded using QR code. It is updated later in main()
package_data = {'packagen00':'red',
                'packagen01':'green',
                'packagen02':'yellow',
                'packagen10':'green',
                'packagen11':'yellow',
                'packagen12':'red',
                'packagen20':'yellow',
                'packagen21':'red',
                'packagen22':'green',
                'packagen30':'red',
                'packagen31':'yellow',
                'packagen32':'green'}

'''# A list of packages that are sent by ur5_1 arm and needs to be sorted out
pkg_to_pick=['packagen31', 'packagen10', 'packagen11', 'packagen12', 'packagen20', 'packagen21',
             'packagen30', 'packagen32', 'packagen01']
# A list to keep a check on packages sorted
pkg_picked=[]'''

# Object of Camera class is used for QR decoding
'''class Camera():

    # Constructor
    def __init__(self):
        self.bridge = CvBridge()

    # Function to obtain attributes of packages from an image of shelf containing packages
    def get_qr_data(self,data):

        global package_data

        # Obtaining the image in CV2 format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Enhancing the contrast for a clear image
        image=cv_image*1.5999999999999998667732370449812151491641998291015625

        # qr_result object contains the decoded data
        qr_result = decode(image)

        # Using the decoded value to identify color of packages
        # Using the for loop, we iterate through each package and update the dicitionary from the attributes of package
        if ( len( qr_result ) > 0):
            for i in range(0, len(qr_result)):
                if qr_result[i].rect.left in range(125,131):
                    if qr_result[i].rect.top in range(313,317):
                        package_data["packagen00"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(494,499):
                        package_data["packagen10"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(640,645):
                        package_data["packagen20"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(795,800):
                        package_data["packagen30"] = str(qr_result[i].data)
                elif qr_result[i].rect.left in range(313,319):
                    if qr_result[i].rect.top in range(313,317):
                        package_data["packagen01"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(494,499):
                        package_data["packagen11"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(640,645):
                        package_data["packagen21"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(795,800):
                        package_data["packagen31"] = str(qr_result[i].data)

                elif qr_result[i].rect.left in range(499,506):
                    if qr_result[i].rect.top in range(313,317):
                        package_data["packagen02"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(494,499):
                        package_data["packagen12"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(640,645):
                        package_data["packagen22"] = str(qr_result[i].data)
                    elif qr_result[i].rect.top in range(795,800):
                        package_data["packagen32"] = str(qr_result[i].data)'''


# Ur5_moveit class for sorting the packages
class Ur5_Moveit:

    # Constructor
    def __init__(self):

        # Initializing the ROS node.
        rospy.init_node('node_ur5_2_place', anonymous=True)
        
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
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns+
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        # Handle to execute planned path using ROS action client
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns+
            '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        
        # Initializing the attributes obtained from Logical camera
        self.model=LogicalCameraImage()

        # Handle for subscibing to ROS topic "/eyrc/vb/logical_camera_2"
        rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,self.cb_capture_model)
        
        # Creating a handle to use Vacuum Gripper service
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',timeout=1)
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
    
        # Creating a handle to use Conveyor Belt service with desired power
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        self.conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)

        rospy.Subscriber('dispatched_order',msgDisOrder,self.cb_exec_sort)

        # Settings for file path from where we will be playing the saved trajectories files
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/ur5_2_saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def cb_exec_sort(self, msg):
        
        global work_done, flag, package_data
        print(msg.pkg_name)
        item_color = package_data[msg.pkg_name]

        if flag:
            self.conveyor_belt_service_call(100)
            while len(self.model) == 1 and self.model[0].type == 'ur5':
                pass
            while self.model[1].pose.position.y > 0.00001:
                pass
            self.conveyor_belt_service_call(0)
            flag = False
            print(self.model[1].pose.position.y)
        
        self.pick_pkg(self.model[1].pose)
        joint_angles = [0.14648733592875196, -2.38179315608067, -0.7257155148810783, -1.6048803093744644, 1.570796326803042, 0.14648733591716212]
        self.hard_set_joint_angles(joint_angles,3)
        work_done = False
        thread1 = threading.Thread (target = self.place_pkg,args = (item_color,))
        thread2 = threading.Thread (target = self.control_conveyor_belt, args = (self.model[1].type,))
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()

    # This function sorts the attached package on the basis of its color and calls hard_set_joint_angles() to execute the same
    def place_pkg(self,package_color):

        global work_done, pkgs_sorted
        
        if(package_color=="red"):
            file_1_name = 'initial_pose_to_red_bin.yaml'
            file_2_name = 'red_bin_to_initial_pose.yaml'
        
        elif (package_color=="yellow"):
            file_1_name = 'initial_pose_to_yellow_bin.yaml'
            file_2_name = 'yellow_bin_to_initial_pose.yaml'
        
        elif(package_color=="green"):
            file_1_name = 'initial_pose_to_green_bin.yaml'
            file_2_name = 'green_bin_to_initial_pose.yaml'
        
        self.moveit_hard_play_planned_path_from_file(self._file_path, file_1_name ,3)
        self.gripper_service_call(False)
        self.moveit_hard_play_planned_path_from_file(self._file_path, file_2_name ,3)
        work_done = True
        pkgs_sorted += 1

    def control_conveyor_belt(self, pkg_picked):
        
        global work_done
        print('starting thread 2')
        self.conveyor_belt_service_call(100)
        a = self.model
        while len(a) <= 2:
            if len(a) == 0:
                pass
            elif len(a) == 1:
                if a[0].type == "ur5" or a[0].type == pkg_picked:
                    pass 
                else:
                    break
            elif len(a) == 2:
                if a[1].type == "ur5" or a[1].type == pkg_picked:
                    pass 
                else:
                    break
            a = self.model
        print('belt stopped')
        while self.model[1].pose.position.y > 0.00001:
            pass 
        self.conveyor_belt_service_call(0)
        while not work_done:
            pass

        ''' while len(self.model.models) == 0 or len(self.model.models)==2:
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
        self.conveyor_belt_service_call(0)'''

    # Function: cb_capture_model() is the callback function for the subscriber to ROS topic "/eyrc/vb/logical_camera_2".
    # It updates the node with models recently scanned by logical camera
    def cb_capture_model(self, model):
        self.model=model.models

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan) # Execution of trajectory file

		return ret
    
    # Function to confirm the execution of saved trajectories in few attempts
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts) )
		
		return True

    # Function: go_to_pose() controls the ur5 arm and takes it to the provided position and orientation
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        flag_plan = False
        attempts = 0

        # Commanding ur5 arm to head towards desired position
        self._group.set_pose_target(arg_pose)
        
        # Confirming that go_to_pose is executed
        while   attempts < 3 and (not flag_plan):
            flag_plan = self._group.go(wait=True)  # wait=False for Async Move
            attempts += 1

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


        
    # calculate_cartesian_path function provides the exact position of package using the frame of logical_camera_2 
    def calculate_package_distance (self,package_pose_wrt_camera): 
        
        # Transforming the package position from logical camera's frame
        pose_package_wrt_world=[-0.8+package_pose_wrt_camera.position.z,
                                package_pose_wrt_camera.position.y,
                                2-package_pose_wrt_camera.position.x]
        
        pose_ee_wrt_world = self._group.get_current_pose().pose
        
        # Distance to be translated by the ur5 arm is determined below
        cartesian_path=(pose_package_wrt_world[0]-pose_ee_wrt_world.position.x,
                        pose_package_wrt_world[1]-pose_ee_wrt_world.position.y,
                        pose_package_wrt_world[2]-pose_ee_wrt_world.position.z+self.delta)
        
        return cartesian_path

    # This function calculates the distance to move and attaches the package
    def pick_pkg(self,package_pose):
        
        x,y,z = self.calculate_package_distance(package_pose)
        
        pose_values = self._group.get_current_pose().pose
        
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = pose_values.position.x + x
        wpose.position.y = pose_values.position.y + y
        wpose.position.z = pose_values.position.z + z
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
        self.go_to_pose(wpose)

        # Attaching the package
        self.gripper_service_call(True)
        
    # Function to take ur5 arm to specified joint angles
<<<<<<< HEAD
    """def set_joint_angles(self, arg_list_joint_angles):
        self._group.set_joint_value_target(arg_list_joint_angles)
=======
    def set_joint_angles(self, arg_list_joint_angles):

		self._group.set_joint_value_target(arg_list_joint_angles)
>>>>>>> origin/main
		flag_plan = self._group.go(wait=True)

		return flag_plan"""

    # Function to confirm set_joint_angles() is a success
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {}".format(number_attempts) )

    # Function to take ur5 to an intial pose from where picking will be easier
    '''def initial_pose(self):
        joint_angles=[0.14655978301275052, -2.4608101683915473, -1.0175133809253598, -1.1476540717685673, 1.5579328111748776, 0.1060079478849465]
        self.hard_set_joint_angles(joint_angles,3)'''

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5_moveit Deleted." + '\033[0m')

def main():
    
    # Creating an object of Ur5_moveit class
    ur5_2 = Ur5_Moveit()
    
    '''# Creating an object of Camera class
    camera2D = Camera()
    shelf_image=rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image,timeout=None)
    
    # Updating the packages dictionary using QR Decoding
    camera2D.get_qr_data(shelf_image)

    # Initiating the conveyor belt and heading ur5 arm towards initial pick position 
    ur5_2.conveyor_belt_service_call(100)'''
    ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, 'start_to_initial_pose.yaml',3)

    while pkgs_sorted < 9:
        pass

    # This loop is used to reach to detected packages, attaching them to ur5 arm using pick_pkg() and sorting 
    # them using place_pkg(). Meanwhile it stops the conveyor belt.
    '''while len(pkg_picked) != len(pkg_to_pick):        
        
        if len(ur5_2.model.models) >= 1:
            
            for obj in ur5_2.model.models:
                if obj.type != "ur5":
                    rospy.sleep(0.5)
                    ur5_2.conveyor_belt_service_call(0)
                    rospy.sleep(0.1)
                    ur5_2.pick_pkg(ur5_2.model.models[1].pose)
                    joint_angles=[0.14648733592875196, -2.38179315608067, -0.7257155148810783, -1.6048803093744644, 1.570796326803042, 0.14648733591716212]
                    ur5_2.hard_set_joint_angles(joint_angles,3)
                    pkg_picked.append(obj.type)
                    print(obj.type)
                    ur5_2.place_pkg(obj.type)'''
    
    # Removing the object of Ur5_Moveit class
    del ur5_2

# main() is implemented when we execute this python file
if __name__ == '__main__':
    main()
    