#! /usr/bin/env python

import rospy

# Importing modules required for performing functions related to computer vision and QR decoding
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

def main():

    rospy.init_node('node_pkg_color_detection',anonymous = True)

    bridge = CvBridge()

    shelf_image = rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image, timeout=None)
    
    print("DECODing")

    # Obtaining the image in CV2 format
    try:
        cv_image = bridge.imgmsg_to_cv2(shelf_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    
    # Enhancing the contrast for a clear image
    #image = cv_image*1.5999999999999998667732370449812151491641998291015625
    i = 1.4
    qr_result = []
    while i <= 2 and len(qr_result) < 12:
        image = cv_image * i
        qr_result = decode(image)
        i += 0.005
    # qr_result object contains the decoded data
    
    print("______________contrast______________")
    print(i)
    
    # Using the decoded value to identify color of packages
    # Using the for loop, we iterate through each package and update the dicitionary from the attributes of package
    if ( len( qr_result ) > 0):
        for i in range(0, len(qr_result)):

            if qr_result[i].rect.left in range(70,170):
                if qr_result[i].rect.top in range(280,360):
                    rospy.set_param('/pkg_clr/packagen00',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(450,540):
                    rospy.set_param('/pkg_clr/packagen10',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(600,690):
                    rospy.set_param('/pkg_clr/packagen20',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(750,850):
                    rospy.set_param('/pkg_clr/packagen30',str(qr_result[i].data))

            elif qr_result[i].rect.left in range(250,350):
                if qr_result[i].rect.top in range(280,360):
                    rospy.set_param('/pkg_clr/packagen01',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(450,540):
                    rospy.set_param('/pkg_clr/packagen11',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(600,690):
                    rospy.set_param('/pkg_clr/packagen21',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(750,850):
                    rospy.set_param('/pkg_clr/packagen31',str(qr_result[i].data))

            elif qr_result[i].rect.left in range(450,550):
                if qr_result[i].rect.top in range(280,360):
                    rospy.set_param('/pkg_clr/packagen02',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(450,540):
                    rospy.set_param('/pkg_clr/packagen12',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(600,690):
                    rospy.set_param('/pkg_clr/packagen22',str(qr_result[i].data))
                elif qr_result[i].rect.top in range(750,850):
                    rospy.set_param('/pkg_clr/packagen32',str(qr_result[i].data))

    print(len(qr_result))
    print(rospy.get_param('pkg_clr'))
    
if __name__ == '__main__':
    main()

