#! /usr/bin/env python


# Importing required modules, msg files, srv files and so on
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from pkg_vb_sim.srv import conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse
from pkg_task5.msg import msgDisOrder

# Importing msg file for obtaining the feed of Logical cameras
from pkg_vb_sim.msg import LogicalCameraImage

powerup_belt_flag = False

# Ur5_moveit class for sorting the packages
class Belt_Controller:

    # Constructor
    def __init__(self):

        # Initializing the ROS node.
        rospy.init_node('node_Belt_Controller', anonymous=True)

        rospy.Subscriber('dispatched_order',msgDisOrder,update_powerup_belt_flag_state)

        # Creating a handle to use Conveyor Belt service with desired power
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        conveyor_belt_service_call = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    
    def update_powerup_belt_flag_state(self):
        powerup_belt_flag = True

def main():
    
    bc = Belt_Controller()
    

# main() is implemented when we execute this python file
if __name__ == '__main__':
    main()
    