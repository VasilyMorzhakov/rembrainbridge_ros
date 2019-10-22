#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge,CvBridgeError

from communication.register import connection_manager
from communication import consumer
from communication import jpeg_meta

import time
import config
import os

global width
global height
global rgb_topic
global depth_topic
import sys

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
            consumer.startConsumers(cm.newConsumers)
            break
        timeout_count-=1
        if timeout_count==0:
            print('timeout connection')
            sys.exit(0)


    return

#listener callbacks
lastTime=time.time()
def callback_rgb(data,args):
    global lastTime
        

    width=args[0]
    height=args[1]

    try:
        cv_image=bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)

    if cv_image.shape[0]!=height or cv_image.shape[1]!=width:
        cv_image=cv2.resize(cv_image,(width,height))

    height,width,channels=cv_image.shape

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
    result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
    buf = encimg.reshape((encimg.shape[0]))

    buffer=jpeg_meta.insertMetaData(buf,[])
    
    consumer.SendImage(buffer)

    if(time.time()-lastTime>10):
        rospy.loginfo('10s message: last rgb image received '+str(width)+' '+str(height))
        lastTime=time.time()


def callback_depth(data):
    width=args[0]
    height=args[1]

    try:
        cv_image=bridge.imgmsg_to_cv2(data,'16UC1')
    except CvBridgeError as e:
        print(e)

    if cv_image.shape[0]!=height or cv_image.shape[1]!=width:
        cv_image=cv2.resize(cv_image,(width,height))

   
    height,width,channels=cv_image.shape   
    #rospy.loginfo('depth image received '+str(width)+' '+str(height))


#start listening
def listener():

    rospy.init_node('rembrain_bridge_main', anonymous=True)

    rospy.Subscriber(rgb_topic,Image,callback_rgb,(width,height))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#main
if __name__ == '__main__':
    


    print('start node')

    #get parameters
    server_address=os.environ["ROBOT_SERVER"]
    login=os.environ["ROBOT_LOGIN"]
    password=os.environ["ROBOT_PASSWORD"]

    robot_id=str(rospy.get_param('/rembrain_bridge_main/ROBOT_ID'))#to string
    width=int(rospy.get_param('/rembrain_bridge_main/width'))
    height=int(rospy.get_param('/rembrain_bridge_main/height'))
    rgb_topic=rospy.get_param('/rembrain_bridge_main/rgb_topic')
    depth_topic=rospy.get_param('/rembrain_bridge_main/depth_topic')
    

    print('width '+str(width))
    print('height '+str(height))

    print('server_address: '+server_address)
    print('robot_id: '+robot_id)

    #register robot in the system
    register(server_address,login,password,robot_id)    
    #start listener
    listener()
