#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import yaml
import os
import math
import time
import sys
import copy
from pkg_task4.msg import picked_pkg_info
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.msg import LogicalCameraImage

task_status=False

package_data = {'packagen00':'',
                                                                                'packagen01':'',
                                                                                'packagen02':'',
                                                                                'packagen10':'',
                                                                                'packagen11':'',
                                                                                'packagen12':'',
                                                                                'packagen20':'',
                                                                                'packagen21':'',
                                                                                'packagen22':'',
                                                                                'packagen30':'',
                                                                                'packagen31':'',
                                                                                'packagen32':'',}



class Camera():
    def __init__(self):
        self.bridge = CvBridge()
       





    def get_qr_data(self,data):
        global package_data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image=cv_image*1.5999999999999998667732370449812151491641998291015625

        qr_result = decode(image)

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
                        package_data["packagen32"] = str(qr_result[i].data)

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
        # logical Camera 1 
        self.model=LogicalCameraImage()
        rospy.Subscriber("/eyrc/vb/logical_camera_1",LogicalCameraImage,self.cb_capture_model)
        # Allow some leeway in position (meters) and orientation (radians)
        self._group.set_goal_position_tolerance(0.1)
        self._group.set_goal_orientation_tolerance(0.1)
        self._group.set_goal_joint_tolerance(0.2)
        
        rospy.wait_for_service('//eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.gripper_service_call = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        self._box_name = 'packagen21'
        self._box_pose = geometry_msgs.msg.PoseStamped()
        
        self._box_pose.header.frame_id = "world"	
        self._box_pose.pose.position.x = 0
        self._box_pose.pose.position.y = 6.589954 - 7
        self._box_pose.pose.position.z = 1.427499
        self._box_pose.pose.orientation.w = 1.0

        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        #creating publisher to publish picked package information
        self._pkg_info_pub=rospy.Publisher("ur51/picked_pkg_info",picked_pkg_info,queue_size=10)
        self.rate=rospy.Rate(10)
        
        
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
    

    # Function: cb_capture_model() is the callback function for the subscriber to ROS topic "/eyrc/vb/logical_camera_2".
    # It updates the node with models recently scanned by logical camera
    def cb_capture_model(self, models):
        global package_data
        global task_status
        if(len(models)>1):
            for x in models:
                if(x.type!="ur5"):
                    self.model=x
                    self._pkg_info_pub.publish(package_color=package_data[x.type],work_done=task_status)
                    self.rate.sleep()


    #function to pick boxes from shelf and place them on conveyer belt
    def pick_place(self,pkg_to_pick):
        if(pkg_to_pick=="package21"):
            rospy.logwarn("1. Playing home_to_pkg21 Trajectory File")
            self.moveit_play_planned_path_from_file(self._file_path, 'home_to_pkg21.yaml')
            rospy.logwarn("1. Playing cp21_pick Trajectory File")
            self.moveit_play_planned_path_from_file(self._file_path, 'cp21_pick.yaml')
            result = self.gripper_service_call(True)
            touch_links = self._robot.get_link_names(group=self._planning_group)  
            ur5._scene.attach_box(self._eef_link,"red_pkg.obj_2", touch_links = touch_links)
            print(ur5.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=4))
            rospy.logwarn("1. Playing cp21_place Trajectory File")
            ur5.moveit_play_planned_path_from_file(ur5._file_path, 'cp21_place.yaml')
            rospy.logwarn("1. Playing pkg21_to_place Trajectory File")
            ur5.moveit_play_planned_path_from_file(ur5._file_path, 'pkg21_to_place.yaml')
            result = ur5.gripper_service_call(False)
            ur5._scene.remove_attached_object(ur5._eef_link, name="red_pkg.obj_2")
            print(ur5.wait_for_state_update(box_is_attached=False, box_is_known=True, timeout=4))




    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
   


def main():
    global task_status
    ur51 = Ur5Moveit()
    2d_camera=Camera()
    rospy.sleep(10)
    shelf_image=rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image,timeout=None)
    2d_camera.get_qr_data(shelf_image)
    no_pkg_to_pick=9
    pkg_picked_placed=0
    pkg_to_pick=['packagen10', 'packagen11', 'packagen12', 'packagen20', 'packagen21', 'packagen22', 'packagen30', 'packagen32', 'packagen33']
    
    while(pkg_picked_placed < no_pkg_to_pick and not ropsy.is_shutdown()):

        ur51.pick_place(pkg_to_pick[pkg_picked_placed])
        pkg_picked_placed=pkg_picked_placed+1
    if(pkg_picked_placed==no_pkg_to_pick):
        task_status=True


    del ur5

    
    """
    ur5._scene.add_box(ur5._box_name,ur5._box_pose, size=(0.15, 0.15, 0.15))


    rospy.logwarn("1. Playing home_to_pkg21 Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'home_to_pkg21.yaml')
    ur5._scene.add_box(ur5._box_name,ur5._box_pose, size=(0.15, 0.15, 0.15))

    rospy.logwarn("1. Playing home_to_pkg21 Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'home_to_pkg21.yaml')


    rospy.logwarn("1. Playing cp21_pick Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'cp21_pick.yaml')

    result = ur5.gripper_service_call(True)
    touch_links = ur5._robot.get_link_names(group=ur5._planning_group)  
    ur5._scene.attach_box(ur5._eef_link,ur5._box_name, touch_links = touch_links)
    print(ur5.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=4))

    rospy.logwarn("1. Playing cp21_place Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'cp21_place.yaml')

    rospy.logwarn("1. Playing pkg21_to_place Trajectory File")
    ur5.moveit_play_planned_path_from_file(ur5._file_path, 'pkg21_to_place.yaml')

    result = ur5.gripper_service_call(False)
    ur5._scene.remove_attached_object(ur5._eef_link, name=ur5._box_name)
    print(ur5.wait_for_state_update(box_is_attached=False, box_is_known=True, timeout=4))


    # Removing the box from planning scene 	
    ur5._scene.remove_world_object(ur5._box_name)"""
    

    del ur5



if __name__ == '__main__':
    main()


