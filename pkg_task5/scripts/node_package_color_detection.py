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

    shelf_image = rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image,timeout=None)

    # Obtaining the image in CV2 format
    try:
        cv_image = bridge.imgmsg_to_cv2(shelf_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    
    # Enhancing the contrast for a clear image
    image = cv_image*1.5999999999999998667732370449812151491641998291015625

    # qr_result object contains the decoded data
    qr_result = decode(image)

    # Using the decoded value to identify color of packages
    # Using the for loop, we iterate through each package and update the dicitionary from the attributes of package
    if ( len( qr_result ) > 0):
        for i in range(0, len(qr_result)):
            if qr_result[i].rect.left in range(125,131):
                if qr_result[i].rect.top in range(313,317):
                    rospy.set_param("packagen00",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(494,499):
                    rospy.set_param("packagen10",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(640,645):
                    rospy.set_param("packagen20",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(795,800):
                    rospy.set_param("packagen30",str(qr_result[i].data))
            elif qr_result[i].rect.left in range(313,319):
                if qr_result[i].rect.top in range(313,317):
                    rospy.set_param("packagen01",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(494,499):
                    rospy.set_param("packagen11",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(640,645):
                    rospy.set_param("packagen21",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(795,800):
                    rospy.set_param("packagen31",str(qr_result[i].data))

            elif qr_result[i].rect.left in range(499,506):
                if qr_result[i].rect.top in range(313,317):
                    rospy.set_param("packagen02",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(494,499):
                    rospy.set_param("packagen12",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(640,645):
                    rospy.set_param("packagen22",str(qr_result[i].data))
                elif qr_result[i].rect.top in range(795,800):
                    rospy.set_param("packagen32",str(qr_result[i].data))


if __name__ == '__main__':
    main()

