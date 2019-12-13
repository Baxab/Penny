from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
import scipy as sp
from util import rotateQuaternion, getHeading
import random
from numpy import hstack, array, vstack
from scipy.cluster.vq import kmeans, kmeans2, vq
from scipy.cluster.hierarchy import linkage, fcluster
import copy
from time import time

TAU = math.pi * 2

class PFLocaliser(PFLocaliserBase):
    # Initializing Parameters   
    def __init__(self):
        # Call the Superclass Constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.05          # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.05       # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.05             # Odometry model y axis (side-to-side) noise

        # Sensor Model Parameters
        self.NUMBER_PREDICTED_READINGS = 20#50      # Number of readings from the laser scan based on  
                                                 # which the weights are predicted.
        self.NUM_PARTICLES = 500                 # Number of particles in the Particle Filter

        # Noise Parameters for Initial Particle Cloud 
        self.INITIAL_X_POSE_NOISE = 0.05#1            # Scaling noise in x co-ordinate 
        self.INITIAL_Y_POSE_NOISE = 0.05#1            # Scaling noise in y co-ordinate
        self.INITIAL_A_POSE_NOISE = 0.1#1            # Scaling the angular rotation noise
        self.INITIAL_GAUSS_SD = 1#8                # Gaussian Standard Deviation - Spread of particles positions
        self.INITIAL_VONMISES_SD = 5#10            # VonMises Standard Deviation - Spread of particles rotations

        # Noise Parameters for Updating Particle Cloud
        self.X_POSE_NOISE = 0.05              
        self.Y_POSE_NOISE = 0.05
        self.A_POSE_NOISE = 0.1
        self.GAUSS_SD = 1
        self.VONMISES_SD = 15

        # Pose Estimation Constants
        self.MAX_D = 5                          # Cut off distance of the hierarchical clustering tree for HAC
        self.K_VALUE = int(math.sqrt(self.NUM_PARTICLES/2))   # Number of clusters for K-Means Algorithm 
        self.SECTOR_WIDTH = 40                  # Width of sector for K Means clustering with Orientations

       # Adaptive Monte Carlo Localisation Parameters 
        self.ADAPTIVE = False                    # Switch on/off AMCL
        self.WEIGHT_THRESHOLD = 5000            # Threshold that decides the number of samples after update
        self.MIN_PARTICLES = 150                # Limits on the number of particles that can be used
        self.MAX_PARTICLES = 500                # for sampling in the Particle Filter.
        self.K_GAUSS_NOISE = 0.007              # Constants for scaling noise in proportion to no. of particles
        self.K_VONMISES_NOISE = 0.05

    # Roulette Wheel Selection of Index - For Resampling in AMCL
    def roulette_wheel_index_selection(self, weightArray, totalWeight):
        value = random.random() * totalWeight   # Function returns the index of the heavy weights with high
        for j in range(len(weightArray)):       # probability.
            value -= weightArray[j]
            index = j
            if value <= 0:
                    break
        return index
        
   # Function to Add Noise in AMCL 
    def adding_noise(self, pose, gaussStDev=None, vonmisesStDev=None):
        if gaussStDev is None:
            gaussStDev = self.GAUSS_SD
        if vonmisesStDev is None:
            vonmisesStDev = self.VONMISES_SD
        pose.position.x += random.gauss(0, gaussStDev) * self.X_POSE_NOISE   # Adding noise to x, y coordinates 
        pose.position.y += random.gauss(0, gaussStDev) * self.Y_POSE_NOISE
        rotationAngle = ((random.vonmisesvariate(0, vonmisesStDev) - math.pi) * self.A_POSE_NOISE)
        rotationAngle = ((rotationAngle + math.pi) % (TAU)) - math.pi
        pose.orientation = rotateQuaternion(pose.orientation, rotationAngle) # Adding noise to rotation angles
        return pose               
                
    # Set Particle Cloud to Initial Pose Plus Noise   
    def initialise_particle_cloud(self, initialpose):
   
        poseArray = PoseArray()                                              # Adding initial gaussian noise to x,y 
        for i in range(self.NUM_PARTICLES):                                  # coordinates and VonMises to angles and 
            thisPose = Pose()                                                # returning a PoseArray
                                                                             
            xnoise = random.gauss(0, self.INITIAL_GAUSS_SD) * self.INITIAL_X_POSE_NOISE
            ynoise = random.gauss(0, self.INITIAL_GAUSS_SD) * self.INITIAL_Y_POSE_NOISE
            
            thisPose.position.x = initialpose.pose.pose.position.x + xnoise
            thisPose.position.y = initialpose.pose.pose.position.y + ynoise

            rotationAngle = ((random.vonmisesvariate(0, self.INITIAL_VONMISES_SD) - math.pi) * self.INITIAL_A_POSE_NOISE)
            rotationAngle = ((rotationAngle + math.pi) % (TAU)) - math.pi
            thisPose.orientation = rotateQuaternion(initialpose.pose.pose.orientation, rotationAngle)

            poseArray.poses.append(thisPose);
        return poseArray
 
    # Update Particlecloud, Given Map and Laser Scan
    def update_particle_cloud(self, scan):                                  # Function resamples based on the weights acquired by 
        self.latest_scan = scan                                             # particles which is obtained from the sensor model   
        poseArray = PoseArray()
        
        weights = []
        totalWeight = 0                                                    
        for pose in self.particlecloud.poses:                               # Finding total weight
            thisWeight = self.sensor_model.get_weight(scan, pose)
            weights.append(thisWeight)
            totalWeight += thisWeight
       # For AMCL 
        if self.ADAPTIVE:
            countWeight = 0
            for i in range(self.MIN_PARTICLES):
                index = self.roulette_wheel_index_selection(weights, totalWeight)     # Resamples particles accoording to Roulette wheel
                countWeight += weights[index]                                         # to achieve minimum no. of particles.
                poseArray.poses.append(copy.deepcopy(self.particlecloud.poses[index]))
            while (countWeight <= self.WEIGHT_THRESHOLD) and (len(poseArray.poses) < self.MAX_PARTICLES): # Resamples particles till the 
                index = self.roulette_wheel_index_selection(weights, totalWeight)                         # threeshold is achieved.
                countWeight += weights[index]
                poseArray.poses.append(copy.deepcopy(self.particlecloud.poses[index]))
            for i, thisPose in enumerate(poseArray.poses):
		#thisPose = self.adding_noise(thisPose, self.K_GAUSS_NOISE*len(poseArray.poses), self.K_VONMISES_NOISE*len(poseArray.poses))
                thisPose = self.adding_noise(thisPose)
       # MCL 
        else:    
            for i in range(len(self.particlecloud.poses)):                             # Roulette Wheel Algorithm picks the highly weighted 
                index = self.roulette_wheel_index_selection(weights, totalWeight)      # particles with high probability and adds noise   
                poseArray.poses.append(copy.deepcopy(self.particlecloud.poses[index])) # to generate updated resampled particles.
            for i, thisPose in enumerate(poseArray.poses):
                thisPose = self.adding_noise(thisPose)
        
        print "Num Particles: " + str(len(poseArray.poses))
        self.particlecloud = poseArray 
       
    # Function to Estimate Pose
    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.

        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        
        # throwing away any which are outliers

##############################################################################
#                                HAC CLUSTERING                              #
##############################################################################
        px = []                                                    
        py = []
        oz = []
        ow = []
        
        for particle in self.particlecloud.poses:                  # Forming a matrix of with the features of the samples      
            px.append(particle.position.x)                         # as columns of a matrix. positions x and y and orienation
            py.append(particle.position.y)                         # w and z are the only features that concern us.
            oz.append(particle.orientation.z)
            ow.append(particle.orientation.w)

        posx = array(px)                                           # Hierarchical Agglomerative Clustering, by minimizing
        posy = array(py)                                           # variance between the particles. The number of clusters 
        orz = array(oz)                                            # is determined by the cut off distance max_d. The distance
        orw = array(ow)                                            # represents the difference between a new cluster and actual 
        x = vstack((posx, posy, orz, orw))                         # particle.
        
        z = linkage(x, 'ward')                                      
        clustered = fcluster(z, self.MAX_D, criterion ='distance') # Function for clustering that outputs an array of cluster  
        clusters = []                                              # IDs for each particle. 
        maxclusters = max(clustered)                                
        numclusterelements = [0] * maxclusters 
        weightcluster = [0] * maxclusters
        i = 0
        for clusterid in clustered:                                
            particle = self.particlecloud.poses[i]
            weight = self.sensor_model.get_weight(self.latest_scan, particle)
             
            if clusterid not in clusters:                          # Arranging total weights of each cluster
                clusters.append(clusterid)                         # and number of elements in each cluster by 
                numclusterelements[clusterid-1] = 1                # cluster ID
                weightcluster[clusterid-1] = weight
                print "new cluster: " + str(clusterid)
            else:
                #print "CLUSTER ID: " + str(clusterid)
                #print "LEN numclusterelemnts: " + str(len(numclusterelements))
             	numclusterelements[clusterid-1] += 1
             	weightcluster[clusterid-1] += weight
            i +=1
 
        for j in range(len(clusters)):
            weightcluster[j]= weightcluster[j]/numclusterelements[j]

        reqclusterid = weightcluster.index(max(weightcluster)) +1   # Finding cluster with maximum weight
        numelementsreq = numclusterelements[reqclusterid-1]
        
        k = 0
        x_sum = 0
        y_sum = 0
        qz_sum = 0
        qw_sum = 0
        for clusterid in clustered:
            particle = self.particlecloud.poses[k]
            if (clusterid == reqclusterid):
                x_sum += particle.position.x
                y_sum += particle.position.y
                qz_sum += particle.orientation.z
                qw_sum += particle.orientation.w
            k += 1 
        estpose = Pose()	 
        estpose.position.x = x_sum/numelementsreq                    # Find average orientation w, orientation z 
        estpose.position.y = y_sum/numelementsreq                    # position x and position y for estimating pose.
        estpose.orientation.z = qz_sum/numelementsreq 
        estpose.orientation.w = qw_sum/numelementsreq
       
        return estpose
##############################################################################
#                                  GLOBAL MEAN                               #
##############################################################################
#
#        estpose = Pose()
#        x_sum = 0
#        y_sum = 0
#        qx_sum = 0
#        qy_sum = 0
#        qz_sum = 0
#        qw_sum = 0
#        for num, i in enumerate(self.particlecloud.poses):
#            x_sum += i.position.x
#            y_sum += i.position.y
#            qz_sum += i.orientation.z
#            qw_sum += i.orientation.w
#
#        estpose.position.x = x_sum / len(self.particlecloud.poses)
#        estpose.position.y = y_sum / len(self.particlecloud.poses)
#        estpose.orientation.z = qz_sum / len(self.particlecloud.poses)
#        estpose.orientation.w = qw_sum / len(self.particlecloud.poses)
#        
#        return estpose
#
##############################################################################
#                                KMEANS CLUSTERING                           #
##############################################################################
#
#        pos_x = [[] for _ in range(len(self.particlecloud.poses))]
#        pos_y = [[] for _ in range(len(self.particlecloud.poses))]
#
#        for i in range(len(self.particlecloud.poses)):
#            pos_x[i].append(self.particlecloud.poses[i].position.x)
#            pos_y[i].append(self.particlecloud.poses[i].position.y)
#
#        f_x = np.array(pos_x)
#        f_y = np.array(pos_y)	
#        #print(f_x)
#        #print(f_y)
#        data = hstack((f_x, f_y))
#        #print(data)
#        
#        # computing K-mean, returning an array of centroids and the labels
#        centroids, labels = kmeans2(data, self.K_VALUE, minit='points')
#
#        count = []
#        for i in range(self.K_VALUE):
#            count.append(0)
#        for i,_ in enumerate(self.particlecloud.poses):
#            index = labels[i]
#            count[index] += 1
#
#        centroidWithMaxXYPoses = count.index(max(count))
#

############                  NOW CHOOSE BETWEEN                  ############
#
#                             KMEANS ORIENTATION                             #
##############################################################################
#
#        sin = [[] for _ in range(len(self.particlecloud.poses))]
#        cos = [[] for _ in range(len(self.particlecloud.poses))]
#
#        for i in range(len(self.particlecloud.poses)):
#            sin[i].append(math.sin(getHeading(self.particlecloud.poses[i].orientation)))
#            cos[i].append(math.cos(getHeading(self.particlecloud.poses[i].orientation)))
#
#        f_sin = np.array(sin)
#        f_cos = np.array(cos)	
#        #print(f_sin)
#        #print(f_cos)
#        data = hstack((f_sin, f_cos))
#        #print(data)
#        
#        # computing K-means, returning an array of centroids and the labels
#        centroids, labels = kmeans2(data, self.K_VALUE, minit='points')
#
#        count = []
#        for i in range(self.K_VALUE):
#            count.append(0)
#        for i,_ in enumerate(self.particlecloud.poses):
#            index = labels[i]
#            count[index] += 1
#
#        centroidWithMaxTetaPoses = count.index(max(count))
#        
#        teta1 = math.asin(centroids[centroidWithMaxTetaPoses][0])
#        sin_teta = []
#        if teta1 > 0:
#            sin_teta = [teta1, math.pi-teta1]
#        else:
#            sin_teta = [teta1, -math.pi+teta1]        
#               
#        teta2 = math.acos(centroids[centroidWithMaxTetaPoses][1])        
#        cos_teta = []
#        cos_teta = [teta2, -teta2]
#        
#        estimatedAngle = 0
#        for i in range(len(sin_teta)):
#            if sin_teta[i] in cos_teta:
#                estimatedAngle = sin_teta[i]
#                break
#        
#        estimatedPose = Pose()
#        estimatedPose.position.x = centroids[centroidWithMaxXYPoses][0]
#        estimatedPose.position.y = centroids[centroidWithMaxXYPoses][1]
#
#        estimatedPose.orientation.x = 1
#        estimatedPose.orientation.w = 1
#        estimatedPose.orientation = rotateQuaternion(estimatedPose.orientation, -getHeading(estimatedPose.orientation))
#        estimatedPose.orientation = rotateQuaternion(estimatedPose.orientation, estimatedAngle)
#
#        return estimatedPose
#
#
#                          SECTORS ORIENTATION                               #
##############################################################################
#
#        orient = []
#        for i,_ in enumerate(self.particlecloud.poses):
#            if labels[i] == centroidWithMaxXYPoses:
#                teta = getHeading(self.particlecloud.poses[i].orientation)
#                teta += math.pi
#                orient.append(teta)
#        orientCount = []
#        for i in range(self.SECTOR_WIDTH):
#           orientCount.append(0)
#        for i in range(len(orient)):
#            for j in np.arange(0, TAU, (math.pi)/self.SECTOR_WIDTH):
#                if orient[i] in np.arange(j, j+(math.pi)/self.SECTOR_WIDTH):
#                    orientCount[j] += 1
#        indexEstimatedOrient = orientCount.index(max(orientCount))
#        valueEstimatedOrient = (indexEstimatedOrient*(math.pi)/self.SECTOR_WIDTH + (math.pi)/(self.SECTOR_WIDTH * 2)) - math.pi
#
#        estimatedPose = Pose()
#        estimatedPose.position.x = centroids[centroidWithMaxXYPoses][0]
#        estimatedPose.position.y = centroids[centroidWithMaxXYPoses][1]#=
#
#        print "TurnVal: " + str(valueEstimatedOrient)
#        estimatedPose.orientation.x = 1
#        estimatedPose.orientation.w = 1
#        estimatedPose.orientation = rotateQuaternion(estimatedPose.orientation, valueEstimatedOrient)
#        return estimatedPose

