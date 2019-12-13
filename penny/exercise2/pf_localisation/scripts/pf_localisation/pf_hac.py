from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
from util import rotateQuaternion, getHeading
import random
from scipy.cluster.hierarchy import dendrogram, linkage, fcluster

from time import time
NUM_PARTICLES = 10

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 1#???? # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 1#???? # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 1#???? # Odometry model y axis (side-to-side) noise

        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
       
    def initialise_particlecloud(self, initialpose):
        # Set particle cloud to initialpose plus noise
        poseArray = PoseArray()
        for i in range(NUM_PARTICLES):
            thisPose = Pose()
            thisPose.position.x = initialpose.pose.pose.position.x + (random.gauss(0, 0.1) * self.ODOM_TRANSLATION_NOISE)
            thisPose.position.y = initialpose.pose.pose.position.y + (random.gauss(0, 0.1) * self.ODOM_DRIFT_NOISE)

            rotationAngle = getHeading(initialpose.pose.pose.orientation) + (random.vonmisesvariate(0, 1) * self.ODOM_ROTATION_NOISE)
            thisPose.orientation = rotateQuaternion(initialpose.pose.pose.orientation, rotationAngle)

            poseArray.poses.append(thisPose);
        return poseArray
 
    
    def update_particlecloud(self, scan):
        # Update particlecloud, given map and laser scan
        poseArray = PoseArray()       
        
        for i in range(NUM_PARTICLES):
            totalWeight = 0
            for pose in self.particlecloud.poses:
                totalWeight += self.sensor_model.get_weight(scan, pose)
                
            value = random.random() * totalWeight;	
            
            for pose in self.particlecloud.poses:
                value -= self.sensor_model.get_weight(scan, pose)
                if value <= 0:
                    break
            poseArray.poses.append(pose)

        # Add some noise to all the particles
        for thisPose in poseArray.poses:
            thisPose.position.x += (random.gauss(0, 0.1) * self.ODOM_TRANSLATION_NOISE)
            thisPose.position.y += (random.gauss(0, 0.1) * self.ODOM_DRIFT_NOISE)

            rotationAngle = getHeading(thisPose.orientation) + (random.vonmisesvariate(0, 1) * self.ODOM_ROTATION_NOISE)
            rotationAngle = rotationAngle % (2 * math.pi)
            thisPose.orientation = rotateQuaternion(thisPose.orientation, rotationAngle)

        self.particlecloud = poseArray 
        
	
    def estimate_pose(self, scan):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
		"""estpose = Pose()
		for i in self.particle_cloud.poses:
			x_sum += i.position.x
			y_sum += i.position.y
			qx_sum += i.orientation.x
			qy_sum += i.orientation.y
			qz_sum += i.orientation.z
			qw_sum += i.orientation.w

		estpose.position.x = x_sum /len(self.particle_cloud.poses)
		estpose.position.y = y_sum/len(self.particle_cloud.poses)
		estpose.orientation.x = qx_sum/len(self.particle_cloud.poses)
		estpose.orientation.y = qy_sum/len(self.particle_cloud.poses)
		estpose.orientation.z = qz_sum/len(self.particle_cloud.poses)
		estpose.orientation.w = qw_sum/len(self.particle_cloud.poses)
		
        return estpose"""
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
        posx = []
        posy = []
        orx = []
        ory = []
        orz = []
        orw = []
        max_d = 10
        
        for particle in self.particlecloud.poses:
             posx.append(particle.position.x)
             posy.append(particle.position.y)
             orx.append(particle.orientation.x)
             ory.append(particle.orientation.y)
             orz.append(particle.orientation.z)
             orw.append(particle.orientation.w)
        
        initialmatrix = np.array(posx, posy, orx, ory, orz, orw)
        x = np.asmatrix(initialmatrix)
        x = np.transpose(x)
        
        z = linkage(x, 'ward')
        clustered = fcluster(z, max_d, criterion ='distance')
        
        clusters = []
        numclusterelements = [] 
        weightcluster = []
        i = 0
        for clusterid in clustered:
             particle = self.particlecloud.poses(i)
             weight = self.sensor_model.get_weight(scan, particle)
             
             if clusterid not in clusters:
             	clusters.append(clusterid)
             	numclusterelements.insert(clusterid-1,1)
             	weightcluster.insert(clusterid-1,weight)
             	
             else:
             	numclusterelements(clusterid-1)+=1
             	weightcluster(clusterid-1) += weight
             i +=1
             
        for j in range(len(clusters)):
             weightcluster(j) = weightcluster(j)/numclusterelements(j)
        reqclusterid = weightcluster.index(max(weightcluster)) + 1
        numelementsreq = numclusterelements(reqclusterid - 1)
        
        k = 0
        x_sum = 0
        y_sum = 0
        qx_sum = 0
        qy_sum = 0
        qz_sum = 0
        qw_sum = 0
        for clusterid in clustered:
           particle = self.particlecloud.poses(k)
           if (clusterid == reqclusterid):
                 x_sum += particle.position.x
	         y_sum += particle.position.y
	         qx_sum += particle.orientation.x
		 qy_sum += particle.orientation.y
		 qz_sum += particle.orientation.z
		 qw_sum += particle.orientation.w
		 
	estpose = Pose()	 
	estpose.position.x = x_sum /numelementsreq
	estpose.position.y = y_sum/numelementsreq
	estpose.orientation.x = qx_sum/numelementsreq
	estpose.orientation.y = qy_sum/numelementsreq
	estpose.orientation.z = qz_sum/numelementsreq
	estpose.orientation.w = qw_sum/numelementsreq
		
               
        return estpose
        
        
             


            
        
        
                    

