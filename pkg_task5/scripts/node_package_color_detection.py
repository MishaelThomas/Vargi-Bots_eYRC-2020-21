#! /usr/bin/env python

import rospy

# Importing modules required for performing functions related to computer vision and QR decoding
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import numpy as np

def detect_pkgs_of_clr(image,hsv_image,pkg_color):
    ''' A function to detect  all the packages of a particular colour.

        Paramter:
                image: Image in "bgr8" format.
                hsv_image: Image in HSV format.
                pkg_color: Colour of the package to be detected. 

        Return : A list containing bounding box coordinates of all the detected packages of a particular colour.
    '''

    bounding_boxes=[] 
    if pkg_color=="red":
        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        lower_red = np.array([170,120,70])
        upper_red = np.array([180,255,255])
        mask2 = cv2.inRange(hsv_image,lower_red,upper_red)
        mask=mask1+mask2    #creating a mask to detect red coloured packages
    elif pkg_color=="green":
        lower_green= np.array([36,120,70])
        upper_green = np.array([70,255,255])
        mask = cv2.inRange(hsv_image, lower_green, upper_green) #creating a mask to detect green coloured packages
    elif pkg_color=="yellow":
        lower_yellow= np.array([20,120,70])
        upper_yellow = np.array([30,255,255])
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow) #creating a mask to detect yellow coloured packages

    _,contours,_=cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Finding all the contours in a masked image

     #A For Loop to filter contours with square shape
    for cnt in contours : 
        approx = cv2.approxPolyDP(cnt,  0.009 * cv2.arcLength(cnt, True), True) #
        # Checking if the no. of sides of the selected region is 4. 
        if (len(approx) == 4):
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h) # Computing ratio of Height and Width of bounding box
            if (ar>=0.95 and ar<=1.05): #Comparing ratio to actual height to width ratio of a square to  filter out rectangle shaped contours
              bounding_boxes.append((x,y,w,h)) #Appending  bounding box coordinates of detected packages to bounding_boxes list

    identify_pkgs(bounding_boxes,pkg_color) #calling identify_pkgs() to identify the name of detected packages
    
def identify_pkgs(bounding_boxes,pkg_color):
    '''
    A function to identify the name of detected packages of a particular colour using the co-ordinates of bounding boxes.
    It updates the configuration file "config_pkg_color" at parameter server.

    Parameter:
                bounding_boxes: A list containing the all the bounding boxes of detected packages
                pkg_color: Colour of the detected packages
    Return : None
    '''
    for x,y,w,h in bounding_boxes:
       if w<=50 and h<=50: #Comparing height and width of bounding box
            if x in range(60,70):
                if y in range(150,160):
                    rospy.set_param('/pkg_clr/packagen00',pkg_color)
                elif y in range(240,250):
                    rospy.set_param('/pkg_clr/packagen10',pkg_color)
                elif y in range(315,325):
                    rospy.set_param('/pkg_clr/packagen20',pkg_color)
                elif y in range(390,400):
                    rospy.set_param('/pkg_clr/packagen30',pkg_color)
            elif x in range(150,160):
                if y in range(150,160):
                    rospy.set_param('/pkg_clr/packagen01',pkg_color)
                elif y in range(240,250):
                    rospy.set_param('/pkg_clr/packagen11',pkg_color)
                elif y in range(315,325):
                    rospy.set_param('/pkg_clr/packagen21',pkg_color)
                elif y in range(390,400):
                    rospy.set_param('/pkg_clr/packagen31',pkg_color)

            elif x in range(245,255):
                if y in range(150,160):
                    rospy.set_param('/pkg_clr/packagen02',pkg_color)
                elif y in range(240,250):
                    rospy.set_param('/pkg_clr/packagen12',pkg_color)
                elif y in range(315,325):
                    rospy.set_param('/pkg_clr/packagen22',pkg_color)
                elif y in range(390,400):
                    rospy.set_param('/pkg_clr/packagen32',pkg_color)
    
    

def detect_pkgs_color(shelf_image):
    '''
        detect_pkgs_color() is a function that detects the colour of all packages placed on the shelf.

        Parameters:
                    shelf_image: a resized image of the shelf having all the packages 

        Return:None

    '''
    pkg_colors=("red","green","yellow")
    bridge = CvBridge()
    try:
      image = bridge.imgmsg_to_cv2(shelf_image, "bgr8") #Converting image to "BGR" format
    except CvBridgeError as e:
      rospy.logerr(e)
    
    resized_image=cv2.resize(image, (720/2, 1280/2)) #Resizing Image

    hsv_image=cv2.cvtColor(resized_image,cv2.COLOR_BGR2HSV) #Converting BGR format image to HSV format image 
    #A For LOOP to detect all the packages of all colours present in pkg_colors
    for clr in pkg_colors:
        detect_pkgs_of_clr(resized_image,hsv_image,clr) #Calling detect_pkgs_of_clr() to detect packages of a particular colour 



def main():

    rospy.init_node('node_pkg_color_detection',anonymous = True)

    shelf_image = rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image, timeout=None) 
    detect_pkgs_color(shelf_image) # detecting colour of all the packages
    print(rospy.get_param("/pkg_clr/"))
    
if __name__ == '__main__':
    main()

