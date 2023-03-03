#!/usr/bin/env python3
import numpy as np
import rospy
import math

from std_msgs.msg import Float64MultiArray, Float64
from modules.sprites import *

rospy.init_node('pure_pursuit_control')
long_drive_pub = rospy.Publisher('controls/throttle', Float64, queue_size=10)
steering_drive_pub = rospy.Publisher('controls/steer', Float64, queue_size=10) 

current_state = np.zeros(6)
def update_car_state(data):
    global current_state
    current_state = np.array(data.data)  # [x, y, theta, speed, beta (slip angle), theta_dot]
    
curr_waypoint = None
def update_waypoint(data):
    global curr_waypoint
    curr_waypoint = np.array(data.data)  # [x, y, yaw_path] of next waypoint


def mytan(Yleft, Xleft):
    targetTheta = np.arctan(Yleft/Xleft)

    if ( Xleft < 0 ):
        targetTheta = targetTheta + np.pi
    elif (Yleft<0  ):
        targetTheta = targetTheta + 2*np.pi

    return targetTheta

rospy.Subscriber("vehicle_model/state", Float64MultiArray, update_car_state)
rospy.Subscriber("waypoints", Float64MultiArray, update_waypoint)

class Controller:
    def __init__(self, L=4.9, kp=20,kd = 15,ki = 1):
        self.L = L
        self.kp = kp
        self.kd = kd 
        self.ki = ki

        self.lastErr = 0
        self.curentIntegralErr = 0
       
    def get_longitudinal_control(self,v_current,v_desired,dt):
        '''
        PID Longitudinal controller
        Parameters
        ----------
        v_current: float
            Current speed of the vehicle
        v_desired: float
            Desired speed of the vehicle
        dt: float
            Delta time since last time the function was called

        Returns
        -------
        throttle_output: float
            Value in the range [-1,1]
        '''

        

        curentErr = v_desired - v_current
        
        curentDerOfErr = (curentErr - self.lastErr ) / dt
        self.curentIntegralErr += curentErr * dt


        self.lastErr = curentErr


        return np.tanh(self.kp * curentErr + self.kd * curentDerOfErr + self.ki * self.curentIntegralErr)
        
        
    
    def get_lateral_pure_pursuit(self,current_xy,current_yaw,next_waypoint):
        '''
        Pure Pursuit, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        next_waypoint: np.array of floats, shape=2
            Next waypoint for the vehicle to reach [x,y]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''


        Xw, Yw = next_waypoint
        Xr, Yr = current_xy


        alpha = math.atan2(Yw-Yr , Xw-Xr) - current_yaw 

        

        Id = np.sqrt ((Yw-Yr)**2 + (Xw-Xr)**2)
        
        ans = math.atan2((2*self.L* np.sin(alpha) ),Id)


        return ans , Id

    def get_lateral_stanley(self,current_xy,current_yaw,current_speed,next_waypoint):
        '''
        Stanley, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        current_speed: float
            Current speed of the vehicle
        next_waypoint: np.array of floats, shape=3
            Next waypoint for the vehicle to reach [x,y,yaw_path]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        Xw, Yw = next_waypoint
        Xr, Yr = current_xy
        k = 0.001



        alpha = math.atan2 ((Yw-Yr ), (Xw-Xr)) - current_yaw 
        


        Id = np.sqrt ((Yw-Yr)**2 + (Xw-Xr)**2)

        e = Id * np.sin(alpha)
        ans = (alpha + math.atan2(k*(e), current_speed))

        
        if (ans>np.pi):
  
            ans =  ans - 2*np.pi 

       

        return ans, Id


        
       
controller = Controller()
 
rate = 60
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    r.sleep()
    if curr_waypoint is None:
        continue

    
    
    # Getting states variables from current car state (position, heading, speed)
    x, y, theta, speed, beta , theta_dot = current_state
 

    xw, yw,_ = curr_waypoint



    
    # Longitudinal and lateral control
    longitudinal_cont = controller.get_longitudinal_control(v_current=speed,v_desired = 5, dt=1.0/rate )
    lateral_cont, Id = controller.get_lateral_stanley(current_xy= [x,y],current_yaw=theta,next_waypoint = [xw, yw], current_speed = speed )
    #lateral_cont, Id = controller.get_lateral_pure_pursuit(current_xy= [x,y],current_yaw=theta,next_waypoint = [xw, yw])

    long_drive_pub.publish(longitudinal_cont)
    steering_drive_pub.publish(lateral_cont* (Id/100.0))

    # Create longitudinal and lateral messages (of type Float64 imported above)

    # Publish the 2 messages

   # print("Torque: {:.2f}, Steering angle: {:.2f}".format(longitudinal_cont,lateral_cont.
    
