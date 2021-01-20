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

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
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
        self._display_trajectory_publisher.publish(display_trajectory);

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan,wait=True)
        
        print('------------Final pose of EE------------------')
        pose_ee_wrt_world=self._group.get_current_pose().pose
        print(pose_ee_wrt_world)

    def calculate_cartesian_path (self,package_pose_wrt_world):
        
        print('------------------Initial pose of EE------------------')
        pose_ee_wrt_world=self._group.get_current_pose().pose
        print(pose_ee_wrt_world)
        
        cartesian_path=(package_pose_wrt_world[0]-pose_ee_wrt_world.position.x, package_pose_wrt_world[1]-pose_ee_wrt_world.position.y, package_pose_wrt_world[2]-pose_ee_wrt_world.position.z)
        
        print("---------Cartesian path--------------")
        print(cartesian_path)
        
        return cartesian_path

    def pick_pkg(self,package_pose):
        #self.conveyor_belt_service_call(0)
        x,y,z=self.calculate_cartesian_path(package_pose)        
        self.ee_cartesian_translation(x,y,z)
        #self.gripper_service_call(True)

    def init_pose(self):
        joint_angles=[0.13686832396868986, -2.3780854418447985, -0.8477707268506842, -1.4858327222534857, 1.5697997509806312, 0.13785032539154152]
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

    
'''   
    joint_values=[[-0.9742846406182437, -0.9917583362181022, -0.3044312064171226, -1.8589697757597214, -2.1563512010967676, 1.562280573474328], 
                    [0.9742668182429552, -1.4659269066858105, -1.4609586675319859, -0.20311025825669926, 2.1767961072848934, 1.5780870003597052], 
                    [2.0954672572238664, -1.1040302206419952, -2.2779576933540033, 0.2519873900043903, 1.055668843381759, 1.5649762425172211], 
                    [2.792818737494608, -1.9273700150774529, -2.273842564851071, 1.087400283762733, 0.35908518601560413, 1.5456822723922903], 
                    [0.9741739402203127, -1.856966303326617, -0.30608721592726873, -0.9653763570206557, 2.177579536797378, 1.5777966536990968], 
                    [2.0951560955520137, -1.126527612354514, -1.697611650156361, -0.3049292783573261, 1.0567490774496333, 1.5635946458394425],
                    [2.792597493184873, -1.5078921387304458, -1.9931345143800385, 0.38836144543341167, 0.3603945307919565, 1.5431553344050633], 
                    [2.0954113312191396, -1.6856253856031262, -2.6584439240320856, 1.2149635676408872, 1.0557889131097546, 1.5642165738563092], 
                    [2.0952084811952663, -1.455065190717173, -0.7708972822471774, -0.9025718084162104, 1.0564405059662603, 1.5637454983483776],
                    [2.792616491595858, -1.4698850830415804, -1.4572608030846084, -0.18533835408597987, 0.3604727669817427, 1.542679965686629], 
                    [0.9742738087282525, -1.4479008357650365, -1.5565234162766242, 3.0176299371242905, -2.1772563045227624, -1.5642725992780422], 
                    [-2.7925244938701477, -1.6041027166316688, 2.0787888506688716, -0.5056704658566646, 0.338095847211874, -1.5413858371296545]
                    ]
    joint_values1 = [[0.9741739402203127, -1.856966303326617, -0.30608721592726873, -0.9653763570206557, 2.177579536797378, 1.5777966536990968], 
                    [-2.792829775861766, -1.2775112171128802, 0.29464571137468365, -2.188389717471538, -0.3388635928051169, 1.599719828699656], 
                    [2.792597493184873, -1.5078921387304458, -1.9931345143800385, 0.38836144543341167, 0.3603945307919565, 1.5431553344050633], 
                    [2.0954113312191396, -1.6856253856031262, -2.6584439240320856, 1.2149635676408872, 1.0557889131097546, 1.5642165738563092], 
                    [-2.0949727848547433, -2.015286673961919, 1.6977560370475588, -2.835937375719584, -1.0370184643197167, 1.575820093473328],
                    ]            
    
    for joint_value in joint_values:
        ur5._group.go(joint_value,wait=True)
        print('done')
        print(joint_value)'''
def main():
    
    # Creating an object of Ur5_moveit class
    ur5 = Ur5_moveit()
    package_pose = [(0.28,-0.218,1.867),
    		(-0.280,-0.218,1.64),
    		(0.00,-0.218,1.42),
    		(0.28,-0.218,1.19),
    		(-0.28,-0.218,1.867),
    		(0.0,-0.218,1.64),
    		(0.28,-0.218,1.42),
    		(0,-0.218,1.19),
    		(0.00,-0.218,1.867),
    		(0.28,-0.218,1.64),
    		(-0.28,-0.218,1.42),
    		(-0.28,-0.218,1.19)] #package pose are in serial wise first four are red_boxes(0 to 3),then yellow,then green
    
    '''package_pose = [(0.00,-0.218,1.867),
    		(0.0,-0.218,1.64)]'''
    joint_values = []
    for current_pkg_pose in package_pose:

    	x,y,z=current_pkg_pose
        
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
        #ur5._group.go(joint_value,wait=True)


    print(joint_values)
    '''
    ur5._box_name = 'packagen21'
    ur5._box_pose = geometry_msgs.msg.PoseStamped()
    
    ur5._box_pose.header.frame_id = "world"	
    ur5._box_pose.pose.position.x = 0.0
    ur5._box_pose.pose.position.y = 6.589954 - 7
    ur5._box_pose.pose.position.z = 1.427499
    ur5._box_pose.pose.orientation.w = 1.0

    service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

	# Adding the box to the scene
    ur5._scene.add_box(ur5._box_name,ur5._box_pose, size=(0.15, 0.15, 0.15))

    joint_value = [2.0954672572238664, -1.1040302206419952, -2.2779576933540033, 0.2519873900043903, 1.055668843381759, 1.5649762425172211] 
    ur5._group.go(joint_value,wait=True)
    
    result = service_call(True)
    touch_links = ur5._robot.get_link_names(group=ur5._planning_group)  
    ur5._scene.attach_box(ur5._eef_link,ur5._box_name, touch_links = touch_links)

    print(ur5._scene.get_attached_objects([ur5._box_name]))
    print(ur5._scene.get_known_object_names())

    ur5.init_pose()

    ur5._scene.remove_attached_object(ur5._eef_link, name=ur5._box_name)
    
    # Removing the box from planning scene 	
    ur5._scene.remove_world_object(ur5._box_name)
  
    # Deactivating the Gripper
    result = service_call(False)
'''
    del ur5

if __name__ == '__main__':
    main()
    