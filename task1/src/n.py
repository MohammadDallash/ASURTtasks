#!/usr/bin/env python3

#this tells the os the location of the python interpurt.

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


rospy.init_node('only') # dh esm el node 



Xg  = rospy.get_param('Xg') 
Yg = rospy.get_param('Yg')

beta  = rospy.get_param('beta') 
phai = rospy.get_param('phai')

Xinvel = 0.0
Xleft = 0.0
Yleft = 0.0
Zangvel = 0.0

def foo(msg: Pose) -> None:
    
    global Xleft,Yleft,Xinvel,Zangvel
    Xc,Yc = msg.x, msg.y
    
    Xleft= Xg - Xc
    Yleft = Yg - Yc
    Xinvel = beta * math.sqrt(Xleft**2 + Yleft**2)

    thetaC = msg.theta
    targetTheta = math.atan(Yleft/Xleft)

    if ( Xleft < 0 ):
        targetTheta = targetTheta + math.pi
    elif (Yleft<0  ):
        targetTheta = targetTheta + 2*math.pi
     
    if (thetaC < 0):
        thetaC+=2*math.pi

    Zangvel = phai * (targetTheta - thetaC)

    
    
    

sub = rospy.Subscriber('/turtle1/pose', Pose, foo)

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)



while not rospy.is_shutdown(): 
    message = Twist()

    message.linear.x = Xinvel
    message.angular.z=Zangvel

    
    pub.publish(message)

    rospy.sleep(0.2)










