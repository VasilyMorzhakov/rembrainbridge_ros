#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge,CvBridgeError

bridge=CvBridge()

def callback(data):
    
    try:
        cv_image=bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)

    height,width,channels=cv_image.shape
   
    rospy.loginfo('image received '+str(width)+' '+str(height))

def listener():

    rospy.init_node('rembrain_bridge_main', anonymous=True)

    rospy.Subscriber('cv_camera/image_raw',Image,callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print('start node')
    listener()
