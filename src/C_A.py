#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dis import dis
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from math import *
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool, String, Int32
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts, BodyVelocityReq, WorldWaypointActionGoal
from cola2_msgs.srv import Goto, GotoRequest
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker
import time
import threading
import sys
# Import wrapper
sys.path.append('/home/arm792/MRS/src/MRS_stack/collision_avoidance/include/RVO2-3D/')
import rvo_wrapper

class CA:

    def __init__(self, name, robot_name):

        self.robot_name = robot_name

        self.first_time = True # To send the robot to its initial position
        self.initial_reached = False # To indicate the robot has reached its initial position
        self.enable_CA = False # To enable collision avoidance after all the robots have reached their initial position
        self.active_service = None # To know which service is active
        self.start_RVO_simulation = True # To create a new RVO2-3D simulation
        
        self.path_iterator = 0 # To iterate through path waypoints
        self.critical_robots = set() # To store those robots which are inside the critical distance
        self.robots_initial_reached = set() # To store those robots that haves reached their initial position
        self.obj_distance = 0.0 # Distance to the robot objective
        # Potential fields repulsive vector
        self.vx_obs = 0.0
        self.vy_obs = 0.0
        self.vz_obs = 0.0
        self.set_vel_msg = BodyVelocityReq()
        
        # Get Parameters
        
        self.GOTO_VEL = rospy.get_param('~GOTO_VEL')
        self.CA_VEL = rospy.get_param('~CA_VEL')
        self.num_robots = rospy.get_param('~num_robots')
        self.CRIT_DIST = rospy.get_param('~CRIT_DIST')
        self.ALGOR = rospy.get_param('~ALGOR')
        self.AZIMUTH_ERROR = rospy.get_param('~AZIMUTH_ERROR')  
        self.ELEVATION_ERROR = rospy.get_param('~ELEVATION_ERROR')      
        self.STOP_AND_WAIT = rospy.get_param('~STOP_AND_WAIT')
        self.SAW_PRIO = rospy.get_param('~SAW_PRIO')
        self.sphere_radius = rospy.get_param('~sphere_radius')  
        self.W1 = rospy.get_param('~W1')
        self.W2 = rospy.get_param('~W2')
        self.K_ROT_MAX = rospy.get_param('~K_ROT_MAX')
        self.K_ROT_MIN = rospy.get_param('~K_ROT_MIN')
        self.time_step = rospy.get_param('~time_step')
        self.time_horizon = rospy.get_param('~time_horizon') 
        self.path = rospy.get_param('~path')            

        self.robot_priorities = [0] * self.num_robots # To store all the robot priorities
        self.critical_priorities = [self.num_robots] * self.num_robots # To store the priorities of those robots which are inside the critical range
        self.neighbors = [[None] * 6 for _ in range(self.num_robots)] # Neighbors in the RVO2-3D simulation
        self.neighbors_id = [[None] * 6 for _ in range(self.num_robots)] # Neighbors id (robot number) of the RVO2-3D simulation
        self.obs_distances = [[None] * 6 for _ in range(self.num_robots)] # To store robot distances (obstacle distances)
        self.objectives = [[None for _ in range(3)] for _ in range(self.num_robots)] # To store all the robot objectives

        # Thread locking for better synchronization
        self.goto_lock = threading.Lock()
        self.update_lock = threading.Lock()
        self.detection_lock = threading.Lock()

        # Load Robot path

        robot_paths = rospy.get_param("/robot_paths")

        for robot_path in robot_paths:
            if robot_path['name'] == self.path:
                self.robot_waypoints = robot_path['waypoints']
                break

        # Services clients

        # Retry until service is available then start navigation and collision avoidance
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/' + self.robot_name + '/captain/enable_goto', 20)
                self.goto_srv = rospy.ServiceProxy(
                            '/' + self.robot_name + '/captain/enable_goto', Goto)
                rospy.wait_for_service('/' + self.robot_name + '/captain/disable_goto', 20)
                self.stop_goto_srv = rospy.ServiceProxy(
                            '/' + self.robot_name + '/captain/disable_goto', Trigger)
                rospy.wait_for_service('/' + self.robot_name + '/captain/enable_keep_position_holonomic', 20)
                self.keep_position_srv = rospy.ServiceProxy(
                            '/' + self.robot_name + '/captain/enable_keep_position_holonomic', Trigger)
                rospy.wait_for_service('/' + self.robot_name + '/captain/disable_keep_position', 20)
                self.stop_keep_position_srv = rospy.ServiceProxy(
                            '/' + self.robot_name + '/captain/disable_keep_position', Trigger) 
                break
            except rospy.exceptions.ROSException:
                rospy.logwarn('%s: error creating client to goto service, retrying...', self.robot_name)
                continue         

        # Subscribers
        
        rospy.Subscriber(
            '/' + self.robot_name + '/navigator/navigation',
            NavSts,
            self.update_robot_position,
            queue_size=1)
        
        for robot in range(self.num_robots):
            if 'robot' + str(robot) != self.robot_name:
                rospy.Subscriber(
                    '/robot' + str(robot) + '/initial_position_reached',
                    String,
                    self.get_initial_reached,
                    queue_size=1)
                rospy.Subscriber(
                    '/robot' + str(robot) + '/navigator/navigation',
                    NavSts,
                    self.obstacle_detection,
                    callback_args=robot,
                    queue_size=1)
                rospy.Subscriber(
                    '/robot' + str(robot) + '/priority',
                    Int32,
                    self.get_robot_priority,
                    callback_args=robot,
                    queue_size=1)
                rospy.Subscriber(
                    '/robot' + str(robot) + '/pilot/goal',
                    PointStamped,
                    self.get_objectives,
                    callback_args=robot,
                    queue_size=1) 

        # Publishers

        self.pub_marker = rospy.Publisher('/' + self.robot_name + '/sphere', Marker, queue_size=1)
        self.pub_velocity = rospy.Publisher('/' + self.robot_name + '/controller/body_velocity_req', BodyVelocityReq, queue_size=1)
        self.pub_initial_position_reached = rospy.Publisher('/' + self.robot_name + '/initial_position_reached', String, queue_size=1)
        self.pub_priority = rospy.Publisher('/' + self.robot_name + '/priority', Int32, queue_size=1)
    
    def get_initial_reached(self,msg): # Start navigating when all the robots have reached their initial position

        self.robots_initial_reached.add(msg.data) # Add those robots which have reached its initial position

        if len(self.robots_initial_reached) == self.num_robots - 1 and not self.enable_CA:
            self.enable_CA = True # Enable collision avoidance                       

    def get_robot_priority(self, msg, robot_number): # Store all robot priorities by robot number

        self.robot_priorities[robot_number] = msg.data

    def get_objectives(self, msg, robot_number): # Store all robot objectives by robot number

        self.objectives[robot_number][0] = msg.point.x
        self.objectives[robot_number][1] = msg.point.y
        self.objectives[robot_number][2] = msg.point.z                         
        
    def send_goto_strategy(self, position_x, position_y, position_z, keep_position, linear_velocity):

        """Goto to position x, y, z, at velocity vel."""
        # Define waypoint attributes

        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = linear_velocity
        goto_req.position.x = position_x
        goto_req.position.y = position_y
        goto_req.position.z = position_z
        goto_req.position_tolerance.x = 5
        goto_req.position_tolerance.y = 5
        goto_req.position_tolerance.z = 5
        goto_req.blocking = True
        goto_req.keep_position = keep_position
        goto_req.disable_axis.x = False
        goto_req.disable_axis.y = True
        goto_req.disable_axis.z = False
        goto_req.disable_axis.roll = True
        goto_req.disable_axis.yaw = False
        goto_req.disable_axis.pitch = True
        goto_req.priority = 10
        goto_req.reference = 0  # REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2

        # Service as a thread so the suscription doesnt get blocked (so we can get pose information in update_robot_position())
        def goto_thread(): 

            with self.goto_lock:
                self.goto_srv(goto_req)

                # If goto service has not been interrupted then signal no service is active 
                if self.active_service == "goto":
                    self.active_service = None  

                # Once the initial position has been reached publish to indicate so
                if not self.initial_reached:
                    self.initial_reached = True
                    self.path_iterator += 1

                    ipr_msg = String()
                    ipr_msg.data = self.robot_name

                    # Publish robot priority for the Stop&Wait strategy aswell
                    priority_msg = Int32()
                    priority_msg.data = self.SAW_PRIO

                    # Publish more than once to make sure every robot receives this information
                    rate = rospy.Rate(10)
                    start_time = rospy.Time.now()
                    duration = rospy.Duration(0.5)
                    while not rospy.is_shutdown() and rospy.Time.now() < start_time + duration:
                        self.pub_initial_position_reached.publish(ipr_msg)
                        self.pub_priority.publish(priority_msg)
                        rate.sleep()

                else:  
                    self.position_check()              

        goto_thread = threading.Thread(target=goto_thread)
        goto_thread.start()
        # rospy.sleep(1.0)
    
    def position_check(self):
        # Increase iterator to navigate through the robot path after reaching the waypoint
        if self.obj_distance <= 3.0:
            self.path_iterator += 1

        # If the robot has ended the robot path then stop avoiding obstacles
        if self.path_iterator >= len(self.robot_waypoints):
            self.enable_CA = False
            self.path_iterator -= 1

            # Publish lowest priority, so other robots don't get stuck
            priority_msg = Int32()
            priority_msg.data = self.num_robots

            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            duration = rospy.Duration(0.5)
            while not rospy.is_shutdown() and rospy.Time.now() < start_time + duration:
                self.pub_priority.publish(priority_msg)
                rate.sleep()

            print(self.robot_name)
            print(self.pos_x) 
            print(self.pos_y) 
            print(self.pos_z) 
    
    def robot_navigation_control(self, goto, stop_goto, keep_position, stop_keep_position):
        # Service controller

        if goto:
            self.active_service = "goto" 
            self.send_goto_strategy(self.robot_waypoints[self.path_iterator][0],self.robot_waypoints[self.path_iterator][1],
                                    self.robot_waypoints[self.path_iterator][2],False, self.GOTO_VEL) 
                    
        elif stop_goto:
            self.active_service = None
            self.stop_goto_srv()               
    
        elif keep_position:
            self.active_service = "keep_position"
            self.keep_position_srv() 

        elif stop_keep_position:
            self.active_service = None
            self.stop_keep_position_srv()

    def update_robot_position(self,msg):

        with self.update_lock:

            # Get navigation information
            self.pos_x = msg.position.north
            self.pos_y = msg.position.east
            self.pos_z = msg.position.depth
            self.roll = msg.orientation.roll
            self.pitch = msg.orientation.pitch
            self.yaw = msg.orientation.yaw
            self.vx = msg.body_velocity.x
            self.vy = msg.body_velocity.y
            self.vz = msg.body_velocity.z

            # Draw security sphere
            self.draw_sphere(self.pos_x, self.pos_y, self.pos_z)

            # Objective distance
            self.obj_distance = sqrt((self.robot_waypoints[self.path_iterator][0] - self.pos_x)**2 + (self.robot_waypoints[self.path_iterator][1] - self.pos_y)**2 +
                                (self.robot_waypoints[self.path_iterator][2] - self.pos_z)**2)

            # If the robot has reached its waypoint while it is avoiding obstacles
            # increment iterator to avoid deadlock
            if self.active_service == None:
                self.position_check()

            # If collision avoidance is not enabled
            if not self.enable_CA:
                # Make robot navigate to its initial position
                if self.first_time:
                    self.first_time = False
                    self.robot_navigation_control(True, False, False, False)
                
                # Whether the robot has reached its final waypoint 
                # or it is waiting for the rest of the robots to reach their initial position
                elif self.active_service != "goto":

                    # Keep position
                    self.robot_navigation_control(False, False, True, False)
            
            # If collision avoidance is enabled
            else:
                
                # If there are critical robots then apply CA algorithm
                # Or if RVO2-3D is enabled just navigate with it          
                if len(self.critical_robots) != 0 or self.ALGOR == 2: 
                    self.obstacle_avoidance()                
                    
                # Otherwise continue navigating
                else:
                    if self.ALGOR == 1:
                        self.vx_obs = 0.0
                        self.vy_obs = 0.0
                        self.vz_obs = 0.0
                    
                    # If the robot was keeping position then shutdown the service
                    if self.active_service == "keep_position":
                        self.robot_navigation_control(False, False, False, True)

                    if self.active_service != "goto":
                        self.robot_navigation_control(True, False, False, False)
    
    def obstacle_avoidance(self):

        # If stop_goto service hasn't been triggered before and goto service is active then shutdown goto service
        if self.active_service == "goto":
            self.robot_navigation_control(False, True, False, False)

        # If Stop&Wait strategy is enabled and the priority is the highest then apply CA algorithm, otherwise apply CA algorithm anyway
        elif (self.STOP_AND_WAIT and self.SAW_PRIO < min(x for x in self.critical_priorities)) or not self.STOP_AND_WAIT:

            # If the robot was keeping position then shutdown the service
            if self.active_service == "keep_position":
                self.robot_navigation_control(False, False, False, True)

            # Publish new odometry to avoid obstacle
            self.pub_velocity.publish(self.set_vel_msg)
        
        # If the robot is not the one with the highest priority just keep position until it is
        elif self.active_service == None:
            self.robot_navigation_control(False, False, True, False)  

    def obstacle_detection(self, msg, robot_number):

        with self.detection_lock:

            if self.enable_CA:
            
                # Get Euclidean distance
                self.obs_distances[robot_number] = sqrt((msg.position.north - self.pos_x)**2 + (msg.position.east - self.pos_y)**2 +
                                                    (msg.position.depth - self.pos_z)**2) 

                if self.obs_distances[robot_number] <= self.CRIT_DIST:

                    self.critical_robots.add(robot_number)  # Add the robot name to the set to know when to apply CA algorithm
                    self.critical_priorities[robot_number] = self.robot_priorities[robot_number] # Add robot priority for the Stop&Wait strategy             

                    # If Stop&Wait strategy is enabled and the priority is the highest then apply CA algorithm, otherwise apply CA algorithm anyway                
                    if (self.STOP_AND_WAIT and self.SAW_PRIO < min(x for x in self.critical_priorities)) or not self.STOP_AND_WAIT:

                        # Apply potential fields
                        if self.ALGOR == 1:                    
                            self.potential_fields(self.obs_distances[robot_number], msg.position.north, msg.position.east, msg.position.depth)
                
                # If the robot was in the critical range and is now no longer
                elif robot_number in self.critical_robots:  

                    self.critical_robots.remove(robot_number)  # Remove the robot name from the set
                    self.critical_priorities[robot_number] = self.num_robots # Remove robot priority 

                # Apply RVO2-3D
                if self.ALGOR == 2:
                    self.reciprocal_velocity(msg.position.north, msg.position.east, msg.position.depth,
                                            msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z, robot_number)                                       

    def potential_fields(self, obs_distance, obs_pos_x, obs_pos_y, obs_pos_z):

        # Vi magnitude
        Vi = (self.CRIT_DIST - obs_distance)/self.CRIT_DIST

        # Vobstacle unit vector
        Vix = Vi*(self.pos_x - obs_pos_x)/obs_distance
        Viy = Vi*(self.pos_y - obs_pos_y)/obs_distance
        Viz = Vi*(self.pos_z - obs_pos_z)/obs_distance

        # Vobstacle summatory
        self.vx_obs += Vix
        self.vy_obs += Viy
        self.vz_obs += Viz

        # Vobjective unit vector
        vx_obj = (self.robot_waypoints[self.path_iterator][0] - self.pos_x)/self.obj_distance
        vy_obj = (self.robot_waypoints[self.path_iterator][1] - self.pos_y)/self.obj_distance
        vz_obj = (self.robot_waypoints[self.path_iterator][2] - self.pos_z)/self.obj_distance

        # Final vector
        vfx = self.W1*vx_obj + self.W2*self.vx_obs
        vfy = self.W1*vy_obj + self.W2*self.vy_obs
        vfz = self.W1*vz_obj + self.W2*self.vz_obs        

        # Azimuth and elevation of the final vector
        azimuth = atan2(vfy, vfx)
        elevation = atan2(vfz, sqrt(vfx**2 + vfy**2))

        robot_orientation = self.yaw
        robot_inclination = self.pitch

        # Azimuth and elevation of the final vector from the robot's pose
        azimuth = atan2(sin(azimuth - robot_orientation), cos(azimuth - robot_orientation))
        elevation = atan2(sin(elevation - robot_inclination), cos(elevation - robot_inclination))

        self.set_vel_msg.header.frame_id = rospy.get_namespace() + "base_link"
        self.set_vel_msg.header.stamp = rospy.Time.now()
        self.set_vel_msg.goal.priority = 40
        self.set_vel_msg.goal.requester = rospy.get_name() + "_velocity_req"

        self.set_vel_msg.disable_axis.x = False 
        self.set_vel_msg.disable_axis.y = True
        self.set_vel_msg.disable_axis.z = False
        self.set_vel_msg.disable_axis.roll = True
        self.set_vel_msg.disable_axis.pitch = True
        self.set_vel_msg.disable_axis.yaw = False

        # Check if azimuth error is within the defined threshold
        if abs(azimuth) <= self.AZIMUTH_ERROR:
            # Set linear x velocity only if azimuth error is small enough
            self.set_vel_msg.twist.linear.x = self.CA_VEL 
            # Set angular z velocity considering the azimuth error
            self.set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.CA_VEL*azimuth
        else:
            # Stop linear x velocity if azimuth error is too large
            self.set_vel_msg.twist.linear.x = 0.0
            # Set angular z velocity considering the azimuth error
            self.set_vel_msg.twist.angular.z = self.K_ROT_MAX*self.CA_VEL*azimuth

        # Check if elevation error is within the defined threshold
        if abs(elevation) <= self.ELEVATION_ERROR:
            # If elevation error is small enough, don't change z direction
            self.set_vel_msg.twist.linear.z = 0.0
        elif elevation > self.ELEVATION_ERROR:
            # If robot needs to go up, set positive z velocity
            self.set_vel_msg.twist.linear.z = self.CA_VEL
        else: # elevation < -self.ELEVATION_ERROR
            # If robot needs to go down, set negative z velocity
            self.set_vel_msg.twist.linear.z = -self.CA_VEL

        self.set_vel_msg.twist.linear.y = 0.0
        self.set_vel_msg.twist.angular.x = 0.0
        self.set_vel_msg.twist.angular.y = 0.0

        # self.new_odom = True
    
    def reciprocal_velocity(self, obs_pos_x, obs_pos_y, obs_pos_z, obs_vel_x, obs_vel_y, obs_vel_z, robot_number):

        # Get main agent position and velocity
        position = (self.pos_x, self.pos_y, self.pos_z)
        velocity = (self.vx, self.vy, self.vz)

        # Get main agent preferred velocity
        vx_obj = self.GOTO_VEL*(self.robot_waypoints[self.path_iterator][0] - self.pos_x)/self.obj_distance
        vy_obj = self.GOTO_VEL*(self.robot_waypoints[self.path_iterator][1] - self.pos_y)/self.obj_distance
        vz_obj = self.GOTO_VEL*(self.robot_waypoints[self.path_iterator][2] - self.pos_z)/self.obj_distance

        preferred_velocity = (vx_obj, vy_obj, vz_obj)

        # Get neighbor agent position and velocity
        neighbor_position = (obs_pos_x, obs_pos_y, obs_pos_z)
        neighbor_velocity = (obs_vel_x, obs_vel_y, obs_vel_z)

        # Get neighbor agent objective distance        
        self.neigh_obj_distance = sqrt((self.objectives[robot_number][0] - obs_pos_x)**2 + (self.objectives[robot_number][1] - obs_pos_y)**2 +
                                (self.objectives[robot_number][2] - obs_pos_z)**2)

        # Get neighbor agent preferred velocity
        neigh_vx_obj = self.GOTO_VEL*((self.objectives[robot_number][0] - obs_pos_x)/self.neigh_obj_distance)
        neigh_vy_obj = self.GOTO_VEL*((self.objectives[robot_number][1] - obs_pos_y)/self.neigh_obj_distance)
        neigh_vz_obj = self.GOTO_VEL*((self.objectives[robot_number][2] - obs_pos_z)/self.neigh_obj_distance)

        neigh_preferred_velocity = (neigh_vx_obj, neigh_vy_obj, neigh_vz_obj)        

        # Create simulation and add the main agent
        if self.start_RVO_simulation:
            self.sim = rvo_wrapper.PyRVOSimulator(self.time_step, self.CRIT_DIST, self.num_robots, self.time_horizon, self.sphere_radius, self.GOTO_VEL, velocity)
            self.agent = self.sim.addAgent(position, self.CRIT_DIST, self.num_robots, self.time_horizon, self.sphere_radius, self.GOTO_VEL, velocity)
            self.start_RVO_simulation = False

        # Check if the neighbor robot is already being simulated, otherwise add it to the simulation
        if robot_number not in self.neighbors_id:
            self.neighbors_id[robot_number] = robot_number
            self.neighbors[robot_number] = self.sim.addAgent(neighbor_position, self.CRIT_DIST, self.num_robots,
                                            self.time_horizon, self.sphere_radius, self.GOTO_VEL, neighbor_velocity)              

        # Update main agent and neighbor agent preferred velocities
        self.sim.setAgentPrefVelocity(self.agent, preferred_velocity)
        self.sim.setAgentPrefVelocity(self.neighbors[robot_number], neigh_preferred_velocity)

        # Update main agent and neighbor agent positions and velocities
        self.sim.setAgentPosition(self.agent, position)
        self.sim.setAgentVelocity(self.agent, velocity)
        self.sim.setAgentPosition(self.neighbors[robot_number], neighbor_position)
        self.sim.setAgentVelocity(self.neighbors[robot_number], neighbor_velocity) 
        
        # Advance one step in the simulation
        self.sim.doStep()

        # Get RVO2-3D resulting velocity
        velocity = self.sim.getAgentVelocity(self.agent)

        # Azimuth of the final vector
        azimuth = atan2(velocity[1], velocity[0])
        # Azimuth from the robot's pose
        azimuth = atan2(sin(azimuth - self.yaw), cos(azimuth - self.yaw))

        velocity_x = sqrt((velocity[0])**2 + (velocity[1])**2)

        self.set_vel_msg.header.frame_id = rospy.get_namespace() + "base_link"
        self.set_vel_msg.header.stamp = rospy.Time.now()
        self.set_vel_msg.goal.priority = 40
        self.set_vel_msg.goal.requester = rospy.get_name() + "_velocity_req"

        self.set_vel_msg.disable_axis.x = False
        self.set_vel_msg.disable_axis.y = True
        self.set_vel_msg.disable_axis.z = False
        self.set_vel_msg.disable_axis.roll = True
        self.set_vel_msg.disable_axis.pitch = True
        self.set_vel_msg.disable_axis.yaw = False

        # Check if azimuth error is within the defined threshold
        if abs(azimuth) <= self.AZIMUTH_ERROR:
            # Set linear x velocity only if azimuth error is small enough
            self.set_vel_msg.twist.linear.x = velocity_x 
            # Set angular z velocity considering the azimuth error
            self.set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.CA_VEL*azimuth
        else:
            # Stop linear x velocity if azimuth error is too large
            self.set_vel_msg.twist.linear.x = 0.0
            # Set angular z velocity considering the azimuth error
            self.set_vel_msg.twist.angular.z = self.K_ROT_MAX*self.CA_VEL*azimuth
        
        self.set_vel_msg.twist.linear.z = velocity[2]

        self.set_vel_msg.twist.linear.y = 0.0
        self.set_vel_msg.twist.angular.x = 0.0
        self.set_vel_msg.twist.angular.y = 0.0
        
    def draw_sphere(self, sphere_x, sphere_y, sphere_z):
        marker = Marker()
        marker.header.frame_id = "/world_ned"
        marker.header.stamp = rospy.Time.now()  
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.SPHERE    
        marker.action = Marker.ADD  
        marker.pose.position.x = sphere_x
        marker.pose.position.y = sphere_y
        marker.pose.position.z = sphere_z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.sphere_radius
        marker.scale.y = self.sphere_radius
        marker.scale.z = self.sphere_radius

        if len(self.critical_robots) != 0 and self.enable_CA:
            if any(item is not None and item <= self.sphere_radius for item in self.obs_distances):
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.25

            elif any(item is not None and item <= self.CRIT_DIST for item in self.obs_distances):
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.25
        
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.25

        marker.lifetime = rospy.Duration()
        self.pub_marker.publish(marker)
                  
if __name__ == '__main__':

    try:
        rospy.init_node('collision_avoidance')
        ca = CA(rospy.get_name(), rospy.get_param('~robot_name'))
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass