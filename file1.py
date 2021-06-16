#!/usr/bin/env python3
# coding: utf-8

"""
PROJET SY31 P21: HO Xuan Vinh
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
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

class moving:

    def __init__(self):
        rospy.init_node('moving', anonymous=True)

        self.surface_target= rospy.Subscriber('surface',Float32,self.getsurface)
        self.position_target = rospy.Subscriber('position',Point32, self.move)
        self.subcriber_laser = rospy.Subscriber('/scan', LaserScan, self.function_laser)

        self.publisher_moving = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        self.target_surface = Float32()
        self.target_surface = 0

        self.distance_obstacle = 0.1

        self.d_forward = 0
        self.d_left = 0
        self.d_right = 0
        self.d_back=0

    def move(self,msg):
        msg_cmd_vel = Twist()

        print('Surface target: ', self.target_surface,'\n')

        distance_min = min(self.d_back,self.d_forward,self.d_left,self.d_right)
        distance_max = max(self.d_back,self.d_forward,self.d_left,self.d_right)

        if(distance_max <= self.distance_obstacle):
            print('Everything is so close, can not move anywhere \n')
            msg_cmd_vel.linear.x=0.0
            msg_cmd_vel.angular.z=0.0
            self.publisher_moving.publish(msg_cmd_vel)

        elif(target_surface < 5000):
            print('Can not see target\n')
            msg_cmd_vel.linear.x = 0
            msg_cmd_vel.angular.z = 0
            self.publisher_moving.publish(msg_cmd_vel)
        
        elif(target_surface <= 30000):
            print('See target\n')
            if(msg.x==0 and msg.y==1 and msg.z==0):
                if(self.d_forward > self.distance_obstacle):
                    print("Go forward \n")
                    msg_cmd_vel.linear.x=0.1
                    msg_cmd_vel.angular.z=0.0
                    self.publisher_moving.publish(msg_cmd_vel)
                else:
                    print('Can not go forward, it is so close \n')
            elif(msg.x==1 and msg.y==0 and msg.z==0):
                if(self.d_left > self.distance_obstacle):
                    print("Go to left \n")
                    msg_cmd_vel.angular.z=0.3
                    msg_cmd_vel.linear.x=0.1
                    self.publisher_moving.publish(msg_cmd_vel)
                else:
                    print('Can not go to left, it is so close \n')
            elif(msg.x==0 and msg.y==0 and msg.z==1):
                if(self.d_right > self.distance_obstacle):
                    print("Go to right \n")
                    msg_cmd_vel.linear.x=0.1
                    msg_cmd_vel.angular.z=-0.7
                    self.publisher_moving.publish(msg_cmd_vel)
                else:
                    print('Can not go to right, it is so close \n')
        else:
            print('Target is close engough \n')
            msg_cmd_vel.linear.x=0.0
            msg_cmd_vel.angular.z=0.0
            self.publisher.publish(msg_cmd_vel)
    
    def getsurface(self,surface):
        print('Data surface: ',surface.data,'\n')
        self.target_surface=surface.data
        print('Surface target: ', self.target_surface,'\n')

    def function_laser(self,msg):
        coords = []
        obstacles = []
        for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
            # ToDo: Remove points too close
            if msg.ranges[i] < 0.1:
                continue
            # ToDo: Polar to Cartesian transformation
            coords.append([msg.ranges[i]*np.cos(theta), msg.ranges[i]*np.sin(theta)])
        #print(coords)
        distance = []
        for point in coords:
            distance.append(np.sqrt(point[0]*point[0]+point[1]*point[1]))
        first = min(distance[0:25])
        second = min(distance[175:200])
        self.d_forward =  min(first,second)
        self.d_right = min(distance[26:75])
        self.d_back = min(distance[76:125])
        self.d_left = min(distance[126:175])

if __name__ == '__main__':
    try:
        moving()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass