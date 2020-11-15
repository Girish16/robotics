#!/usr/bin/python

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import pf_localisation
from threading import Lock
import numpy as np
import sys
import math
from copy import deepcopy

Q=np.diag([0.1,0.1,np.deg2rad(1.0),1.0])**2
R=np.diag([0.1,0.1])**2
DT=1.0
class ExtendedKalmanFilter(object):
    def __init__(self):
        self.xEst=np.zeros((4,1))
        self.xTrue=np.zeros((4,1))
        self.pEst=np.zeros((4,1))




    def calc_input(self):
        v = 2 # [m/s]
        yawrate = 1.5 # [rad/s]
        u = np.array([[v], [yawrate]])
        return u
    def motion_model(self,x, u):
        F = np.array([[1.0, 0, 0, 0],
                    [0, 1.0, 0, 0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 0]])

        B = np.array([[DT * math.cos(x[2, 0]), 0],
                     [DT * math.sin(x[2, 0]), 0],
                     [0.0, DT],
                     [1.0, 0.0]])

        x = np.dot(F,x) +np.dot(B,u)

        return x
    # def observation(self,xTrue, xd, u):
    #     xTrue = motion_model(xTrue, u)
    #
    #     # add noise to gps x-y
    #     z = observation_model(xTrue) + np.dot(0.5,np.random.randn(2, 1))
    #
    #     # add noise to input
    #     ud = u + np.dot(0.3,np.random.randn(2, 1))
    #
    #     xd = motion_model(xd, ud)
    #
    #     return xTrue, z, xd, ud
    def observation_model(self,x):
        H = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0]
                    ])

        z = np.dot(H,x)

        return z

    def jacob_f(self,x, u):
        """
        Jacobian of Motion Model

        motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
            [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF

    def jacob_h(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
            ])

        return jH
    def set_initial_pose(self, pose):
        """ Initialise filter with start pose """
        # ----- Estimated pose has been set, so we should now reinitialise the
        rospy.loginfo("InitalPose%s"%pose)
        rospy.loginfo("Got pose. Intializing the xEst matrix.")
        self.xEst[0,0]=pose.pose.pose.position.x
        self.xEst[1,0]=pose.pose.pose.position.y
        self.xTrue[0,0]=pose.pose.pose.position.x
        self.xTrue[1,0]=pose.pose.pose.position.y
        self.pEst=np.array(list(pose.pose.covariance)).reshape(6,6)
        self.pEst=self.pEst[0:4,0:4]

        rospy.loginfo("intialPEst%s"%pEst)

    def call_ekf(self,odometry):
        odox=odometry.pose.pose.position.x
        odoy=odometry.pose.pose.position.y
        x=np.array([[odox,0,0,0],[0,odoy,0,0],[0,0,0,0],[0,0,0,0]])
        print(x)
        self.u=self.calc_input()

        z=self.observation_model(x)
        rospy.loginfo("intialPest%s"%self.pEst)

        self.xEst, self.pEst = self.ekf_estimation(self.xEst, self.pEst,z,self.u)
        return self.xEst, self.pEst

    def ekf_estimation(self,xEst, pEst, z, u):
        #  Predict
        xPred = self.motion_model(xEst, u)
        jF = self.jacob_f(xEst, u)
        rospy.loginfo("InitilaJF%s"%jF)
        rospy.loginfo("intialJT%s"%jF.T)

        # PPred=reduce(np.dot,[jF,pEst,jF.T])+Q
        PPred = jF .dot(pEst) .dot(jF.T) + Q

        #  Update
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH.dot(PPred).dot(jH.T) + R
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
        xEst = xPred + K.dot(y)
        pEst = (np.eye(len(xEst)) - K.dot(jH)) .dot(PPred)
        return xEst, pEst
