#!/usr/bin/env python3
# coding: utf-8

"""
PROJET SY31 P21: HO Xuan Vinh
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from turtlebot3_msgs.msg import SensorState
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_euler, euler_from_quaternion

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        # rospy.param is used to get parameters from ROS
        self.bgr_min = np.array([ 90, 100, 30], dtype=np.uint8)
        self.bgr_max = np.array([130, 140, 70], dtype=np.uint8)
        self.hsv_min = np.array([0.5*70, 2.55*40, 2.55*40], dtype=np.uint8) # (internally, OpenCV use ranges between 0 and 180, 255, 255)
        self.hsv_max = np.array([0.5*80, 2.55*60, 2.55*60], dtype=np.uint8) # (internally, OpenCV use ranges between 0 and 180, 255, 255)
        
        # Initialize the node parameters
        # TODO

        self.position = Point32()
        self.target_surface = Float32()

        #Blue Objet
        self.Cmin=np.array([171,74,0])
        self.Cmax=np.array([221,114,40])

        # Publisher to the output topics.
        self.position_target = rospy.Publisher('position',Point32,queue_size = 10)
        self.surface_target = rospy.Publisher('surface',Float32,queue_size=10)
        self.pub_img = rospy.Publisher('~output', Image, queue_size=1)

        # Subscriber to the input topic. self.function_camera is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_rect_color', Image, self.function_camera, queue_size=1)

    def function_camera(self, msg):
        '''
        Function called when an image is received.
        msg: Image message received
        '''
        # Convert ROS Image -> OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn('ROS->OpenCV %s', e)
            return
        
        #calcul width
        width = img_bgr.shape[1]

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask_bgr = cv2.inRange(img_bgr, self.bgr_min, self.bgr_max)
        mask_hsv = cv2.inRange(img_hsv, self.hsv_min, self.hsv_max)
        mask = cv2.morphologyEx(mask_hsv, cv2.MORPH_CLOSE, np.ones((20,20)))

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if (length(contours) == 0):
            print('Can not see target\n')
            self.target_surface = 0
            self.surface_target.publish(self.target_surface)

            self.position.x = 0
            self.position.y = 0
            self.position.z = 0
            self.position_target.publish(self.position)
        else: #if (length(contours) >0))
            valMax = cv2.contourArea(contours[0])
            k = 0
            for i in range(len(contours)) :
                if valMax < cv2.contourArea(contours[i]):
                    k = i
                    valMax = cv2.contourArea(contours[i])

            center = np.mean(contours[k], axis=0)[0]
        
            radius = np.abs(center - contours[k][0])

            #If it is a circle
            self.target_surface = np.pi*radius*radius
            self.surface_target.publish(self.target_surface)
            
            for i in range(len(contours[k])):
                if radius < np.abs(center - contours[k][0]):
                    radius = np.abs(center - contours[k][0])
            
            cv2.circle(img_bgr, tuple(center), radius)

            if ( center < width/3 ):   #left
                self.position.x=1
                self.position.y=0
                self.position.z=0
                self.position_target.publish(self.position)
            elif ( center > 2*width/3 ): #right
                self.position.z=1
                self.position.x=0
                self.position.y=0
                self.position_target.publish(self.position)
            else:
                self.position.y=1
                self.position.x=0
                self.position.z=0
                self.position_target.publish(self.position)

            cont = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cont)>100:
                hull = cv2.convexHull(cont)
                x,y,w,h = cv2.boundingRect(cont)

                # self.target_surface = ?? x*y or w*h
                # self.surface_target.publish(self.target_surface)

                cv2.drawContours(img_bgr, [cont], -1, (0,255,0), 10)
                cv2.drawContours(img_bgr, [hull], -1, (0,0,255), 10)
                cv2.rectangle(img_bgr, (x,y), (x+w,y+h),(255,0,0),2)
                cv2.circle(img_bgr, (x+int(w/2),y+int(h/2)), 10, (50,50,50), -1)
            else:
                x,y,w,h = cv2.boundingRect(cont)
                # self.target_surface = ?? x*y or w*h
                # self.surface_target.publish(self.target_surface)


        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_bgr, "bgr8"))  # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn('OpenCV->ROS %s', e)    
            
if __name__ == '__main__':
    try:
        CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass