#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  # Ref: GeekforGeeks.com, https://www.geeksforgeeks.org/detect-the-rgb-color-from-a-webcam-using-python-opencv/
  def get_dominant_colour(self, arg_img):
    # setting values for base colors 
    b = arg_img[:, :, :1] 
    g = arg_img[:, :, 1:2] 
    r = arg_img[:, :, 2:] 
  
    # computing the mean 
    b_mean = np.mean(b) 
    g_mean = np.mean(g) 
    r_mean = np.mean(r) 

    # displaying the most prominent color 
    if (g_mean > r_mean and g_mean > b_mean): 
        return 'green' 
    else: 
        return 'something else'
  
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(image, (720/2, 1280/2)) 

    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
    
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)[1]
    cv2.imshow("thresholded_image",thresh)
   
    image,contours,_=cv2.findContours(thresh, cv2.RETR_TREE, 
                            cv2.CHAIN_APPROX_SIMPLE) 
    cv2.imshow("image",image)
   
# Searching through every region selected to  
# find the required polygon. 
    for cnt in contours : 
        area = cv2.contourArea(cnt) 
   
    # Shortlisting the regions based on there area. 
         
        approx = cv2.approxPolyDP(cnt,  0.04 * cv2.arcLength(cnt, True), True) 
   
        # Checking if the no. of sides of the selected region is 7. 
        #if (len(approx) == 4):
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
            # 
            #(x, y, w, h) = cv2.boundingRect(approx)
            #ar = w / float(h)
            #if (ar>=0.95 and ar<=1.05):
        cv2.drawContours(resized_image,[cnt],-1,(0,255,0),2)
			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
           

   
# Showing the image along with outlined arrow. 
    cv2.imshow('image2', resized_image)  
    #rospy.loginfo(self.get_dominant_colour(image))

    cv2.waitKey(3)


def main(args):
  
  rospy.init_node('node_image_analysis', anonymous=True)

  ic = Camera1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
