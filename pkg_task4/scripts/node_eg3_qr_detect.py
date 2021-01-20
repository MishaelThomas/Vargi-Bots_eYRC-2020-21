#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
    self.package_data = {'packagen00':'',
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
	
  
  def get_qr_data(self, arg_image):
    qr_result = decode(arg_image)

    if ( len( qr_result ) > 0):
      for i in range(0, len(qr_result)):
      	if qr_result[i].rect.left in range(125,131):
      		if qr_result[i].rect.top in range(313,317):
      			self.package_data["packagen00"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(494,499):
      			self.package_data["packagen10"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(640,645):
      			self.package_data["packagen20"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(795,800):
      			self.package_data["packagen30"] = str(qr_result[i].data)
      			
      	elif qr_result[i].rect.left in range(313,319):
      		if qr_result[i].rect.top in range(313,317):
      			self.package_data["packagen01"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(494,499):
      			self.package_data["packagen11"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(640,645):
      			self.package_data["packagen21"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(795,800):
      			self.package_data["packagen31"] = str(qr_result[i].data)
      			
      	elif qr_result[i].rect.left in range(499,506):
      		if qr_result[i].rect.top in range(313,317):
      			self.package_data["packagen02"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(494,499):
      			self.package_data["packagen12"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(640,645):
      			self.package_data["packagen22"] = str(qr_result[i].data)
      		elif qr_result[i].rect.top in range(795,800):
      			self.package_data["packagen32"] = str(qr_result[i].data)
      		

  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image=cv_image*1.5999999999999998667732370449812151491641998291015625
    self.get_qr_data(image)


def main(args):
  
  rospy.init_node('node_eg3_qr_decode', anonymous=True)

  ic = Camera1()
  
  while ic.package_data['packagen00'] == '':
  	pass
  
  rospy.loginfo(ic.package_data)
  

if __name__ == '__main__':
    main(sys.argv)

