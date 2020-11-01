from geometry_msgs.msg import Pose,PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random
from nav_msgs.msg import Odometry
from numpy.random import random_sample
from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
                # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE=0.05
        self.ODOM_TRANSLATION_NOISE=0.2
        self.ODOM_DRIFT_NOISE=0.01
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20    # Number of readings to predict
    
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.self.particlecloud) poses of the particles

        """
	#rospy.loginfo("initial pose %s"%initialpose)
	X = initialpose.pose.pose.position.x 
	Y = initialpose.pose.pose.position.y
	th = initialpose.pose.pose.orientation
	rad = 1
	#self.particlecloud=[]
        particleArray=PoseArray()
        particleArray.poses=[Odometry().pose.pose]*250
        #rospy.loginfo("particlearray pose %s"%len(particleArray.poses))
	for i in range(0,250):
           	particle=Pose()
		th1=random()*360
		th2=random()*360
		radius=random()*rad

		x= radius*math.sin(th1)+X
		y= radius*math.cos(th1)+Y
 	        particle.position.x=x
  		particle.position.y=y	
		particle.orientation=rotateQuaternion(th,th2)
		particleArray.poses[i]=particle
		#rospy.loginfo("random pose %s"%particle)
	#rospy.loginfo("particlearray pose %s"%particleArray)
	return particleArray	
	
	
	
        

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
	rospy.loginfo("update_particle_cloud ")
 	weight=[0]*len(self.particlecloud.poses)
	i=0
	for pose in self.particlecloud.poses:
          weight[i]=self.sensor_model.get_weight(scan,pose)
          i=i+1
        newparticles=PoseArray()
 	newparticles.poses=[Odometry().pose.pose]*len(self.particlecloud.poses)
        
       	for i in range(0,len(self.particlecloud.poses)):
		choice=0.004
		cdf=0
		j=0
		while cdf<choice:
			cdf+=weight[j]
                        j+=1
		newparticles.poses[i]=self.particlecloud.poses[j-1]
        self.particlecloud=newparticles				

		
          
        

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.

         """
	x=0
	y=0
	th=0
	
	for particle in self.particlecloud.poses:
		x+= particle.position.x
		y+= particle.position.y
		th+=particle.orientation.z
	estimatedpose=Odometry().pose.pose

        estimatedpose.position.x=x/len(self.particlecloud.poses)
	estimatedpose.position.y=y/len(self.particlecloud.poses)
	estimatedpose.orientation.z=th/len(self.particlecloud.poses)
 	
        return estimatedpose

	
