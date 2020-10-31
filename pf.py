from geometry_msgs.msg import Pose,PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import sensor_model
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
	X = initialpose.pose.pose.position.x 
	Y = initialpose.pose.pose.position.y
	th = initialpose.pose.pose.orientation
	rad = 1
	#self.particlecloud=[]
        particleArray=PoseArray()
        particleArray.poses=[Odometry().pose.pose]*250
	for i in range(0,99):
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
		

	return particleArray	
	
	
	
        pass

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
 	weight=[]
	i=0
	for pose in self.particlecloud.poses:
          weight[i]=sensormodel.get_weight(self,scan,pose)
          i=i+1
        newparticles=PoseArray()
 	newparticles.poses=[Pose()]*len(self.particlecloud.poses)
        
       	for i in range(len(self.particlecloud)):
		choice=0.05
		cdf=0
		j=0
		for particle in particlecloud:
			cdf+=weight[j]
			j=j+1
			if cdf >= choice:
			 newparticles.poses[i]=particle

			 break
        self.particlecloud=newparticles				

		
          
        pass

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
	
	for particle in self.particlecloud:
		x+= particle.pose.position.x
		y+= particle.pose.position.y
		th+=particle.pose.orientation.z
	estimatedpose=Pose()
      
        estimatedpose.postion.x=x/len(self.particlecloud)
	estimatedpose.position.y=y/len(self.particlecloud)
	estimatedpose.orientation.z=th/len(self.particlecloud)
 	
        return (estimatedpose)

	
