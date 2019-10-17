#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge,CvBridgeError

from communication.register import connection_manager
import time
import config

bridge=CvBridge()#cv2-ros bridge for image convertion

#Register robot on the server
def register(server,login,password,robot_id):
    cm = connection_manager()
    cm.connect(server, login, password,robot_id)

    timeout_count=config.connection_timeout
    while (True):
        time.sleep(1.0)
        if cm.checkReadyToWork():
            print('registered and ready to work')
            break
        timeout_count-=1
        if timeout_count==0:
            print('timeout connection')
            exit(0)


    return

#listener callback
def callback(data):
    
    try:
        cv_image=bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)

    height,width,channels=cv_image.shape
   
    rospy.loginfo('image received '+str(width)+' '+str(height))
#start listening
def listener():

    rospy.init_node('rembrain_bridge_main', anonymous=True)

    rospy.Subscriber('cv_camera/image_raw',Image,callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#main
if __name__ == '__main__':
    print('start node')

    #get parameters
    server_address=str(rospy.get_param('/rembrain_bridge_main/SERVER_ADDRESS'))#to string
    login=str(rospy.get_param('/rembrain_bridge_main/LOGIN'))#to string 
    password=str(rospy.get_param('/rembrain_bridge_main/PASSWORD'))#to string
    robot_id=str(rospy.get_param('/rembrain_bridge_main/ROBOT_ID'))#to string

    print('server_address: ',server_address)
    print('robot_id: ',robot_id)

    #register robot in the system
    register(server_address,login,password,robot_id)    
    #start listener
    listener()
