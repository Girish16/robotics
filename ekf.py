#!/usr/bin/python


import rospy

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
import pf_localisation
import numpy as np
import math

Q=np.diag([0.1,0.1,np.deg2rad(1.0),1.0])**2
R=np.diag([0.1,0.1])**2
DT=1.0
class ExtendedKalmanFilter(object):
    def __init__(self):
        self.xEst=np.zeros((4,1))
        self.pEst=np.zeros((4,1))




    def calc_input(self):
        v = 2 # [m/s]
        yawrate = 1.5 # [rad/s]
        u = np.array([[v], [yawrate]])
        print(u)
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
    def observation_model(self,x):
        H = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0]
                    ])

        z = np.dot(H,x)

        return z

    def jacob_f(self,x, u):
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
        pose.pose.pose.orientation.w=0
        rospy.loginfo("InitalPose%s"%pose)
        rospy.loginfo("Got pose. Intializing the xEst matrix.")
        self.xEst[0,0]=pose.pose.pose.position.x
        self.xEst[1,0]=pose.pose.pose.position.y
        self.xEst[3,0]=pose.pose.pose.orientation.z


        rospy.loginfo("orientation%s"%self.xEst[3,0])
        self.pEst=np.array(list(pose.pose.covariance)).reshape(6,6)
        last=[0,0,0,self.pEst[5,5]]

        self.pEst=self.pEst[0:4,0:4]
        self.pEst[3]=last
        rospy.loginfo("Self.pEst%s"%self.pEst)

    def call_ekf(self,odometry):
        odox=odometry.pose.pose.position.x
        odoy=odometry.pose.pose.position.y

        x=np.array([[odox,0,0,0],[0,odoy,0,0],[0,0,0,0],[0,0,0,odometry.pose.pose.orientation.z]])
        print(x)
        self.u=self.calc_input()

        z=self.observation_model(x)

        self.xEst, self.pEst = self.ekf_estimation(self.xEst, self.pEst,z,self.u)
        return self.xEst, self.pEst

    def ekf_estimation(self,xEst, pEst, z, u):
        #  Predict
        xPred = self.motion_model(xEst, u)
        jF = self.jacob_f(xEst, u)

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
    # def recalculate_transform(self, currentTime):
    #     """
    #     Creates updated transform from /odom to /map given recent odometry and
    #     laser data.
    #
    #     :Args:
    #         | currentTime (rospy.Time()): Time stamp for this update
    #      """
    #
    #     transform = Transform()
    #
    #     T_est = transformations.quaternion_matrix([self.estimatedpose.pose.pose.orientation.x,
    #                                                self.estimatedpose.pose.pose.orientation.y,
    #                                                self.estimatedpose.pose.pose.orientation.z,
    #                                                self.estimatedpose.pose.pose.orientation.w])
    #     T_est[0, 3] = self.estimatedpose.pose.pose.position.x
    #     T_est[1, 3] = self.estimatedpose.pose.pose.position.y
    #     T_est[2, 3] = self.estimatedpose.pose.pose.position.z
    #
    #     T_odom = transformations.quaternion_matrix([self.last_odom_pose.pose.pose.orientation.x,
    #                                                self.last_odom_pose.pose.pose.orientation.y,
    #                                                self.last_odom_pose.pose.pose.orientation.z,
    #                                                self.last_odom_pose.pose.pose.orientation.w])
    #     T_odom[0, 3] = self.last_odom_pose.pose.pose.position.x
    #     T_odom[1, 3] = self.last_odom_pose.pose.pose.position.y
    #     T_odom[2, 3] = self.last_odom_pose.pose.pose.position.z
    #     T = np.dot(T_est, np.linalg.inv(T_odom))
    #     q = transformations.quaternion_from_matrix(T) #[:3, :3])
    #
    #     transform.translation.x = T[0, 3]
    #     transform.translation.y = T[1, 3]
    #     transform.translation.z = T[2, 3]
    #     transform.rotation.x = q[0]
    #     transform.rotation.y = q[1]
    #     transform.rotation.z = q[2]
    #     transform.rotation.w = q[3]
    #
    #
    #     # ----- Insert new Transform into a TransformStamped object and add to the
    #     # ----- tf tree
    #     new_tfstamped = TransformStamped()
    #     new_tfstamped.child_frame_id = "/odom"
    #     new_tfstamped.header.frame_id = "/map"
    #     new_tfstamped.header.stamp = currentTime
    #     new_tfstamped.transform = transform
    #
    #     # ----- Add the transform to the list of all transforms
    #    self.tf_message = tfMessage(transforms=[new_tfstamped])
