#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
from numpy.random import normal
import math
from copy import deepcopy

from random import randint, random

from scipy.stats import norm

from likelihood_field import LikelihoodField

## The angle range for each particle created
angle_range = range(0, 360, 90)

## Taken from measurement_update_likelihood_field script
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        self.occupancy_field = None

        # initialize likelihood field
        self.likelihood_field = LikelihoodField()


        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):
        print("Getting the map")
        self.map = data
        ## data is an OccupancyGrid object
        ## data.info gives metadata
        ## data.data gives list of occupancy probabilities (0-100)

        # They said on the slack this line could be deleted
        # self.occupancy_field = OccupancyField(data)



    def initialize_particle_cloud(self):

        # TODO
        print("Initializing Particle Cloud")
        info = self.map.info #metadata
        map_resolution = info.resolution #(m/cell)
        map_width = info.width #(cells)
        map_height = info.height #(cells)
        map_origin = info.origin
        map_data = self.map.data #occupancy probability for each cell (tuple)
        #print("Map resolution (meters/cell):", map_resolution)
        #print("Map height (meters)", map_height*map_resolution)
        #print("Map width (meters)", map_width*map_resolution)
        # simple attempt: do one particle per open cell
        initial_particle_set = []
        count = 0
        threshold = 100
        for r in range(0, map_width):
            for c in range(0, map_height):
                occupancy_prob = map_data[count]
                if randint(0, 1000) > threshold:
                    count += 1
                    continue
                if occupancy_prob == 0:
                    ## Add a particle: this spot is empty
                    ## This must include the origin offset
                    x_coord = c * map_resolution + map_origin.position.x
                    y_coord = r * map_resolution + map_origin.position.y
                    ## Create particles at different angles for each pos:
                    for a in angle_range:
                        ## format is [x, y, angle(radians)]
                        new_pos = [x_coord, y_coord, (a*math.pi/180.)]
                        initial_particle_set.append(new_pos)
                count += 1
        # threshold is how many particles out of 100 to keep
        # In final draft we will delete and just use them all
        # moved this earlier so each particle has all angles
        #threshold = 3
        ## This code is from the class meeting 06 starter code:
        for i in range(len(initial_particle_set)):
            # Randomly eliminiate particles with probability threshold/10
            #if randint(0, 1000) > threshold:
            #    continue
            p = Pose()
            p.position = Point()
            p.position.x = initial_particle_set[i][0]
            p.position.y = initial_particle_set[i][1]
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, initial_particle_set[i][2])
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            self.particle_cloud.append(new_particle)
        print('created ', len(self.particle_cloud), 'particles')


        self.normalize_particles()
        # Sleep to ensure that the particle publisher has
        # registered. Without this particles may not appear
        rospy.sleep(1)
        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0

        # TODO
        print("Normalizing particles")
        total_weight = 0
        for p in self.particle_cloud:
            weight = p.w
            total_weight += weight
        if total_weight == 0:
            print("No points in particle cloud (map wasn't read in?)")
        else:
            factor = 1.0/total_weight
            for p in self.particle_cloud:
                weight = p.w
                p.w = weight*factor
        print('Done normalizing')




    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        # # TODO:
        print('Resampling particles')
        n = len(self.particle_cloud)
        particle_weights = [p.w for p in self.particle_cloud]
        # Sample new particle cloud according to current weights
        new_cloud = draw_random_sample(self.particle_cloud, particle_weights, n)
        self.particle_cloud = new_cloud
        print('Resampled particles; weights hasve not yet been updated')

        ## I notice when I move the robot forward the number of particles decreases

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        print("Todo: estimate robot pose")
        x_positions = [p.pose.position.x for p in self.particle_cloud]
        y_positions = [p.pose.position.y for p in self.particle_cloud]
        angles = [get_yaw_from_pose(p.pose) for p in self.particle_cloud]
        ## mean and standard deviation of the positions:
        mux, stdx = norm.fit(x_positions)
        muy, stdy = norm.fit(y_positions)
        mua, stda = norm.fit(angles)
        ## creating the pose from these statistics:
        p = Pose()
        p.position = Point()
        p.position.x = mux
        p.position.y = muy
        p.position.z = 0
        p.orientation = Quaternion()
        q = quaternion_from_euler(mux, muy, mua)
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        self.robot_estimate = p
        # print("Estimated position:", p)
        ## unsure if I should be checking some stdev bounds?



    def update_particle_weights_with_measurement_model(self, data):
        print("Updating particle weights")
        ## I lifted part of this from the class meeting 06 code
        ## I am unsure of the way I am iterating through the angles --
        ## since we have multiple particles at the same location I get a lot
        ## of repeated weights in a row, which seems wrong.
        for p in self.particle_cloud:
            q = 1
            particle_pose = p.pose
            particle_weight = p.w
            theta = get_yaw_from_pose(particle_pose)  ## in radians
            theta_degrees = int(theta*180.0/math.pi)
            closest_object = data.ranges[theta_degrees]
            for a in angle_range:
                z_t_k = data.ranges[a]
                if z_t_k > 3.5:
                    continue
                x_z_t_k = particle_pose.position.x + z_t_k*math.cos(theta + (a*math.pi/180.0))
                y_z_t_k = particle_pose.position.y + z_t_k*math.sin(theta + (a*math.pi/180.0))
                closest_obstacle_distance = self.likelihood_field.get_closest_obstacle_distance(x_z_t_k, y_z_t_k)
                ## sometimes it cannot locate a closest object
                if math.isnan(closest_obstacle_distance):
                    closest_obstacle_distance = 3.5
                gaussian_std = 0.1
                prob = compute_prob_zero_centered_gaussian(closest_obstacle_distance, gaussian_std)
                q = q*prob
            # Now I set the particle's weight to q?
            p.w = q
        print("Done updating particle weights")




    def update_particles_with_motion_model(self):
        print("Updating particles with motion")
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        last_pose = self.odom_pose_last_motion_update.pose
        current_pose = self.odom_pose.pose
        ## changes from the last position:
        delta_x = current_pose.position.x - last_pose.position.x
        delta_y = current_pose.position.y - last_pose.position.y
        delta_a = get_yaw_from_pose(current_pose)-get_yaw_from_pose(last_pose)
        ## adjusting particles by these parameters:
        for p in self.particle_cloud:
            # args are mean, std, num_particles for generating gaussian noise
            (x_noise, y_noise) = normal(0, .1, 2)
            ang_noise = normal(0, .1, 1)
            p.pose.position.x += delta_x + x_noise
            p.pose.position.y += delta_y + y_noise
            theta = get_yaw_from_pose(p.pose) 
            theta += delta_a + ang_noise
            q = quaternion_from_euler(p.pose.position.x, p.pose.position.y, theta)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
        ## I think we may need to include noise in these updates.




if __name__=="__main__":


    pf = ParticleFilter()

    rospy.spin()
