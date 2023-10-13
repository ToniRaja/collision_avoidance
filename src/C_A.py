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
from cola2_msgs.msg import WorldSectionAction, WorldSectionGoal,GoalDescriptor, WorldSectionGoal, WorldSectionActionResult
from cola2_msgs.msg import  NavSts, BodyVelocityReq, WorldWaypointActionGoal
from cola2_msgs.srv import Goto, GotoRequest
from collision_avoidance.msg import Results, Objective
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker
import time
import threading
import sys
import subprocess
import signal
import os
import roslaunch
# Import wrapper
sys.path.append('/home/arm792/MRS/src/MRS_stack/collision_avoidance/include/RVO2-3D/')
import rvo_wrapper

class CA:

    def __init__(self, name, robot_name):

        self.robot_name = robot_name

        self.first_time = True # To send the robot to its initial position
        self.initial_reached = False # To indicate the robot has reached its initial position
        self.final_reached = False # To indicate the robot has reached its final waypoint
        self.enable_CA = False # To enable collision avoidance after all the robots have reached their initial position
        self.start_RVO_simulation = True # To create a new RVO2-3D simulation
        self.second_time = True # To know when the CA navigation has started
        self.get_CA_time = False # To get collision avoidance time
        self.stuck = False # To know if the robot got stuck avoiding collision

        self.active_service = None # To know which service is active
        self.path_iterator = 0 # To iterate through path waypoints
        self.critical_robots = set() # To store those robots which are inside the critical distance
        self.robots_initial_reached = set() # To store those robots that have reached their initial position
        self.robots_wp_reached = 0 # To know how many robots have reached a waypoint
        self.obj_distance = 0.0 # Distance to the robot objective
        self.num_collisions_avoided = 0 # Number of collisions avoided successfully
        self.num_fatal_collisions = 0 # Number of fatal collisions
        self.distance_travelled = 0.0 # Distance travelled during the simulation
        self.CA_initial_time = 0 # Initial collision avoidance time
        self.CA_time = 0 # Collision avoidance time 
        self.wp_initial_time = 0 # To obtain the time spent travelling to the next waypoint

        # Potential fields repulsive vector
        self.vx_obs = 0.0
        self.vy_obs = 0.0
        self.vz_obs = 0.0

        # New velocity messsage
        self.set_vel_msg = BodyVelocityReq()
        
        # Get Simulation conditions        
        self.GOTO_VEL = rospy.get_param('~GOTO_VEL')
        self.ROT_VEL = rospy.get_param('~ROT_VEL')
        self.K_ROT_MAX = rospy.get_param('~K_ROT_MAX')
        self.K_ROT_MIN = rospy.get_param('~K_ROT_MIN')
        self.num_robots = rospy.get_param('~num_robots')
        self.time_coefficient = rospy.get_param('~time_coefficient') 
        self.sphere_radius = rospy.get_param('~sphere_radius')  
        self.SAW_PRIO = rospy.get_param('~SAW_PRIO')
        self.path = rospy.get_param('~path')   

        # Create global arrays
        self.robot_priorities = [0] * self.num_robots # To store all the robot priorities
        self.critical_priorities = [self.num_robots] * self.num_robots # To store the priorities of those robots which are inside the critical range
        self.neighbors = [[None] * 6 for _ in range(self.num_robots)] # Neighbors in the RVO2-3D simulation
        self.neighbors_id = [[None] * 6 for _ in range(self.num_robots)] # Neighbors id (robot number) of the RVO2-3D simulation
        self.obs_distances = [[None] * 6 for _ in range(self.num_robots)] # To store robot distances (obstacle distances)
        self.objectives = [[None for _ in range(3)] for _ in range(self.num_robots)] # To store all the robot objectives
        self.fatal_collision = [False] * self.num_robots # To know when an obstacle could not be avoided
        self.normal_collision = [False] * self.num_robots # To know when an obstacle is being avoided successfully

        # Load Robot path
        robot_paths = rospy.get_param("/robot_paths")

        for robot_path in robot_paths:
            if robot_path['name'] == self.path:
                self.robot_waypoints = robot_path['waypoints']
                break
            
        # Load most important parameter values
        parameters = rospy.get_param("/parameter_values")

        for parameter in parameters:
            if parameter['name'] == "ALGOR":
                self.ALGOR = parameter['value']

            elif parameter['name'] == "STOP_AND_WAIT":
                self.STOP_AND_WAIT = parameter['value']

            elif parameter['name'] == "AZIMUTH_ERROR":
                self.AZIMUTH_ERROR = parameter['value']

            elif parameter['name'] == "CRIT_DIST":
                self.CRIT_DIST = parameter['value']

            elif parameter['name'] == "W1":
                self.W1 = parameter['value']
            
            elif parameter['name'] == "W2":
                self.W2 = parameter['value']
            
            elif parameter['name'] == "time_step":
                self.time_step = parameter['value']
            
            elif parameter['name'] == "time_horizon":
                self.time_horizon = parameter['value']
            
            else:
                self.DES_VEL = parameter['value']
        
        # Crate Results message
        self.results_msg = Results()
        self.results_msg.robot_name = self.robot_name
        self.results_msg.ALGOR = self.ALGOR
        self.results_msg.Stop_AND_Wait = self.STOP_AND_WAIT
        self.results_msg.W1 = self.W1
        self.results_msg.W2 = self.W2
        self.results_msg.time_step = self.time_step
        self.results_msg.time_horizon = self.time_horizon
        self.results_msg.sphere_radius = self.sphere_radius
        self.results_msg.CRIT_DIST = self.CRIT_DIST
        self.results_msg.DES_VEL = self.DES_VEL
        self.results_msg.ROT_VEL = self.ROT_VEL
        self.results_msg.K_ROT_MAX = self.K_ROT_MAX
        self.results_msg.K_ROT_MIN = self.K_ROT_MIN
        self.results_msg.AZIMUTH_ERROR = self.AZIMUTH_ERROR

        # Services clients
        # Retry until service is available
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
        rospy.Subscriber(
            '/results',
            Results,
            self.restart_simulation,
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
                    '/robot' + str(robot) + '/objective',
                    Objective,
                    self.get_objectives,
                    callback_args=robot,
                    queue_size=1) 

        # Publishers
        self.pub_CA_marker = rospy.Publisher('/' + self.robot_name + '/CA_sphere', Marker, queue_size=1)
        self.pub_security_marker = rospy.Publisher('/' + self.robot_name + '/security_sphere', Marker, queue_size=1)
        self.pub_velocity = rospy.Publisher('/' + self.robot_name + '/controller/body_velocity_req', BodyVelocityReq, queue_size=1)
        self.pub_initial_position_reached = rospy.Publisher('/' + self.robot_name + '/initial_position_reached', String, queue_size=1)
        self.pub_priority = rospy.Publisher('/' + self.robot_name + '/priority', Int32, queue_size=1)     
        self.pub_results = rospy.Publisher('/results', Results, queue_size=1)
        self.pub_objective = rospy.Publisher('/' + self.robot_name + '/objective', Objective, queue_size=1)

    def get_initial_reached(self,msg):
        # Start navigating when all the robots have reached their initial position

        self.robots_initial_reached.add(msg.data) # Add those robots which have reached their initial position

        if len(self.robots_initial_reached) == self.num_robots - 1 and not self.enable_CA:
            self.enable_CA = True # Enable collision avoidance                       

    def get_robot_priority(self, msg, robot_number):
        # Store all robot priorities by robot number

        self.robot_priorities[robot_number] = msg.data

    def get_objectives(self, msg, robot_number):
        # Store all robot objectives by robot number

        self.objectives[robot_number][0] = msg.x
        self.objectives[robot_number][1] = msg.y
        self.objectives[robot_number][2] = msg.z
    
    def restart_simulation(self, msg):
        # Since we are comparing CA algorithms the simulation will end after every robot has reached its second waypoint
        # If we want the robots to navigate through more waypoints simply change pkill condition
        # But robots can get stuck so we cannot expect them to always get to its last waypoint

        self.robots_wp_reached += 1

        # When all robots have reached their last waypoint (the second one) restart the simulation
        if self.robots_wp_reached == self.num_robots and self.robot_name == "robot0":
            os.system('pkill ros')

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

        self.goto_srv(goto_req)
        rospy.sleep(1.0)

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
        
            self.pub_initial_position_reached.publish(ipr_msg)
            self.pub_priority.publish(priority_msg)
            
        else:  
            self.position_check()   

        obj_msg = Objective()
        obj_msg.x = self.robot_waypoints[self.path_iterator][0]
        obj_msg.y = self.robot_waypoints[self.path_iterator][1] 
        obj_msg.z = self.robot_waypoints[self.path_iterator][2] 

        self.pub_objective.publish(obj_msg)           
    
    def position_check(self):
        # Increase iterator to navigate through the robot path after reaching the waypoint
        if self.obj_distance <= self.sphere_radius:
            self.wp_initial_time = time.time()
            self.path_iterator += 1

        # If the robot has ended the robot path then stop avoiding obstacles
        if self.path_iterator >= len(self.robot_waypoints):
            final_time = time.time() - self.initial_time
            self.enable_CA = False
            self.final_reached = True
            self.path_iterator -= 1

            # Get collision avoidance time
            if self.get_CA_time:
                self.CA_time += time.time() - self.CA_initial_time

            # Check if the collision has been avoided when the robot is still considered an obstacle
            # and increment collision counters
            for item in self.fatal_collision:
                if item:
                    self.num_fatal_collisions += 1
            
            for item in self.normal_collision:
                if item:
                    self.num_collisions_avoided += 1

            if not self.stuck:
                # Publish results
                self.results_msg.robot_stuck = self.stuck
                self.results_msg.avoided_collisions = self.num_collisions_avoided
                self.results_msg.fatal_collisions = self.num_fatal_collisions
                self.results_msg.navigation_time = final_time
                self.results_msg.CA_time = self.CA_time
                self.results_msg.distance_travelled = self.distance_travelled
                self.results_msg.optimal_time = self.optimal_time
                self.results_msg.optimal_distance_travelled = self.optimal_distance_travelled

                self.pub_results.publish(self.results_msg)

            # Publish lowest priority, so other robots don't get stuck
            priority_msg = Int32()
            priority_msg.data = self.num_robots
            self.pub_priority.publish(priority_msg)     
    
    def robot_navigation_control(self, service):
        # Service controller

        if service == "goto":
            self.active_service = "goto" 
            self.send_goto_strategy(self.robot_waypoints[self.path_iterator][0], self.robot_waypoints[self.path_iterator][1],
                                    self.robot_waypoints[self.path_iterator][2], False, self.GOTO_VEL) 
                    
        elif service == "stop_goto":
            self.active_service = None
            self.stop_goto_srv()               
    
        elif service == "keep_position":
            self.active_service = "keep_position"
            self.keep_position_srv() 

        elif service == "stop_keep_position":
            self.active_service = None
            self.stop_keep_position_srv()

    def update_robot_position(self,msg):
        # Use this function to move the robot

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

        # Create CA and security spheres        
        self.create_sphere(self.pos_x, self.pos_y, self.pos_z, "CA sphere")
        self.create_sphere(self.pos_x, self.pos_y, self.pos_z, "security sphere")

        # Objective distance
        self.obj_distance = sqrt((self.robot_waypoints[self.path_iterator][0] - self.pos_x)**2 + (self.robot_waypoints[self.path_iterator][1] - self.pos_y)**2 +
                            (self.robot_waypoints[self.path_iterator][2] - self.pos_z)**2)

        # If the robot has reached its waypoint while it is avoiding obstacles
        # increment iterator to avoid deadlock
        if self.active_service == None and self.initial_reached:
            self.position_check()

        # If collision avoidance is not enabled
        if not self.enable_CA:
            # Make robot navigate to its initial position
            if self.first_time:
                self.first_time = False
                self.robot_navigation_control("goto")
            
            # or if it is waiting for the rest of the robots to reach their initial position
            elif self.active_service != "goto" and self.initial_reached:

                # Keep position
                self.robot_navigation_control("keep_position")
        
        # If collision avoidance is enabled
        elif self.initial_reached:

            # Get initial timestamps to obtain results
            if self.second_time:
                self.second_time = False
                self.initial_time = time.time()
                self.wp_initial_time = time.time()
                self.optimal_time = self.obj_distance/self.DES_VEL
                self.optimal_distance_travelled = self.obj_distance
                self.last_x = self.pos_x
                self.last_y = self.pos_y
                self.last_z = self.pos_z
            
            # Get distance travelled                
            self.distance_travelled += sqrt((self.pos_x - self.last_x)**2 + (self.pos_y - self.last_y)**2 + (self.pos_z - self.last_z)**2)
            self.last_x = self.pos_x
            self.last_y = self.pos_y
            self.last_z = self.pos_z 

            # If the robot has been navigating more than x times the optimal time consider it got stuck
            if time.time() - self.wp_initial_time > self.optimal_time*self.time_coefficient and not self.stuck:
                self.stuck = True
                
                # Publish results
                self.results_msg.robot_stuck = self.stuck
                self.results_msg.avoided_collisions = self.num_collisions_avoided
                self.results_msg.fatal_collisions = self.num_fatal_collisions
                self.results_msg.navigation_time = time.time() - self.initial_time
                if len(self.critical_robots) == 0:
                    self.results_msg.CA_time = self.CA_time
                else:
                    self.results_msg.CA_time = self.CA_time + time.time() - self.CA_initial_time
                self.results_msg.distance_travelled = self.distance_travelled
                self.results_msg.optimal_time = self.optimal_time
                self.results_msg.optimal_distance_travelled = self.optimal_distance_travelled

                self.pub_results.publish(self.results_msg)
            
            # Apply CA algorithm for autonomous navigation
            self.obstacle_avoidance()
    
    def obstacle_avoidance(self):

        # If stop_goto service hasn't been triggered before and goto service is active then shutdown goto service
        if self.active_service == "goto":
            self.robot_navigation_control("stop_goto")

        # If Stop&Wait strategy is enabled and the priority is the highest then apply CA algorithm, otherwise apply CA algorithm anyway
        if (self.STOP_AND_WAIT and self.SAW_PRIO < min(x for x in self.critical_priorities)) or not self.STOP_AND_WAIT:

            # If the robot was keeping position then shutdown the service
            if self.active_service == "keep_position":
                self.robot_navigation_control("stop_keep_position")

            # Publish new odometry to avoid obstacle
            self.pub_velocity.publish(self.set_vel_msg)
        
        # If the robot is not the one with the highest priority just keep position until it is
        elif self.active_service == None:
            self.robot_navigation_control("keep_position")  

    def obstacle_detection(self, msg, robot_number):
        # Function to detect other robots as obstacles

        if self.enable_CA and self.initial_reached:
        
            # Get Euclidean distance
            self.obs_distances[robot_number] = sqrt((msg.position.north - self.pos_x)**2 + (msg.position.east - self.pos_y)**2 +
                                                (msg.position.depth - self.pos_z)**2) 

            if self.obs_distances[robot_number] <= self.CRIT_DIST:
                
                # Get CA initial time
                if not self.get_CA_time:
                    self.get_CA_time = True
                    self.CA_initial_time = time.time()

                # Check if there is fatal collision
                if self.obs_distances[robot_number] <= self.sphere_radius:
                    self.fatal_collision[robot_number] = True
                    self.normal_collision[robot_number] = False
                
                elif not self.fatal_collision[robot_number]:                   
                    self.normal_collision[robot_number] = True

                self.critical_robots.add(robot_number)  # Add the robot name to the set to know when to apply CA algorithm
                self.critical_priorities[robot_number] = self.robot_priorities[robot_number] # Add robot priority for the Stop&Wait strategy              
            
            # If the robot was in the critical range and is now no longer
            elif self.obs_distances[robot_number] > self.CRIT_DIST and robot_number in self.critical_robots: 

                self.critical_robots.remove(robot_number)  # Remove the robot name from the set
                self.critical_priorities[robot_number] = self.num_robots # Remove robot priority (Store the lowest priority)

                # If the are no more obstacles get the time spent avoiding collision
                if len(self.critical_robots) == 0:
                    self.CA_time += time.time() - self.CA_initial_time
                    self.get_CA_time = False

                    # Reset potential feilds obstacle vector
                    if self.ALGOR == 1:
                        self.vx_obs = 0.0
                        self.vy_obs = 0.0
                        self.vz_obs = 0.0

                # Increment collision counters
                if self.fatal_collision[robot_number]:
                    self.fatal_collision[robot_number] = False
                    self.num_fatal_collisions += 1
                
                else:
                    self.normal_collision[robot_number] = False
                    self.num_collisions_avoided += 1                    
            
            # Apply potential fields
            if self.ALGOR == 1:                    
                self.potential_fields(self.obs_distances[robot_number], msg.position.north, msg.position.east, msg.position.depth)
            
            # Apply RVO2-3D
            elif self.ALGOR == 2:
                self.reciprocal_velocity(msg.position.north, msg.position.east, msg.position.depth,
                                        msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z, robot_number)

    def potential_fields(self, obs_distance, obs_pos_x, obs_pos_y, obs_pos_z):

        vf_behind = False

        # Vi magnitude
        if obs_distance <= self.CRIT_DIST:
            Vi = (self.CRIT_DIST - obs_distance)/self.CRIT_DIST
        
        else:
            Vi = 0

        # Vobstacle vector
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

        # Azimuth of the final vector
        azimuth = atan2(vfy, vfx)

        # Azimuth of the final vector from the robot's pose
        azimuth = atan2(sin(azimuth - self.yaw), cos(azimuth - self.yaw))

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

        # If vf points behind the robot recalculate azimuth angle to allow the robot to move backwards
        if len(self.critical_robots) != 0:
            if azimuth <= -pi/2:
                azimuth = pi + azimuth
                vf_behind = True
            
            elif azimuth >= pi/2:
                azimuth = azimuth - pi
                vf_behind = True
        
        # if azimuth is inside the threshold and vf points behind then move backwards while turning
        if abs(azimuth) <= self.AZIMUTH_ERROR and vf_behind:
            self.set_vel_msg.twist.linear.x = -min(sqrt(vfx**2 + vfy**2), self.DES_VEL)
            self.set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.ROT_VEL*azimuth
        
        # if azimuth is inside the threshold and vf points forward then move forward while turning
        elif abs(azimuth) <= self.AZIMUTH_ERROR and not vf_behind:
            self.set_vel_msg.twist.linear.x = min(sqrt(vfx**2 + vfy**2), self.DES_VEL)
            self.set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.ROT_VEL*azimuth
        
        # Otherwise stop and rotate until azimuth is inside the threshold           
        else:
            self.set_vel_msg.twist.linear.x = 0.0
            self.set_vel_msg.twist.angular.z = self.K_ROT_MAX*self.ROT_VEL*azimuth

        if vfz < 0:
            self.set_vel_msg.twist.linear.z = max(vfz, -self.DES_VEL)

        else:
            self.set_vel_msg.twist.linear.z = min(vfz, self.DES_VEL)

        self.set_vel_msg.twist.linear.y = 0.0
        self.set_vel_msg.twist.angular.x = 0.0
        self.set_vel_msg.twist.angular.y = 0.0
        
    def reciprocal_velocity(self, obs_pos_x, obs_pos_y, obs_pos_z, obs_vel_x, obs_vel_y, obs_vel_z, robot_number):

        vx_behind = False

        # Get main agent position and velocity
        position = (self.pos_x, self.pos_y, self.pos_z)
        velocity = (self.vx, self.vy, self.vz)

        # Get main agent preferred velocity
        vx_obj = self.DES_VEL*(self.robot_waypoints[self.path_iterator][0] - self.pos_x)/self.obj_distance
        vy_obj = self.DES_VEL*(self.robot_waypoints[self.path_iterator][1] - self.pos_y)/self.obj_distance
        vz_obj = self.DES_VEL*(self.robot_waypoints[self.path_iterator][2] - self.pos_z)/self.obj_distance

        preferred_velocity = (vx_obj, vy_obj, vz_obj)

        # Get neighbor agent position and velocity
        neighbor_position = (obs_pos_x, obs_pos_y, obs_pos_z)
        neighbor_velocity = (obs_vel_x, obs_vel_y, obs_vel_z)

        # Get neighbor agent objective distance        
        neigh_obj_distance = sqrt((self.objectives[robot_number][0] - obs_pos_x)**2 + (self.objectives[robot_number][1] - obs_pos_y)**2 +
                                (self.objectives[robot_number][2] - obs_pos_z)**2)

        # Get neighbor agent preferred velocity
        neigh_vx_obj = self.DES_VEL*((self.objectives[robot_number][0] - obs_pos_x)/neigh_obj_distance)
        neigh_vy_obj = self.DES_VEL*((self.objectives[robot_number][1] - obs_pos_y)/neigh_obj_distance)
        neigh_vz_obj = self.DES_VEL*((self.objectives[robot_number][2] - obs_pos_z)/neigh_obj_distance)

        neigh_preferred_velocity = (neigh_vx_obj, neigh_vy_obj, neigh_vz_obj)        

        # Create simulation and add the main agent
        if self.start_RVO_simulation:
            self.start_RVO_simulation = False
            self.sim = rvo_wrapper.PyRVOSimulator(self.time_step, self.CRIT_DIST, self.num_robots,
                        self.time_horizon, self.sphere_radius, self.DES_VEL, velocity)
            self.agent = self.sim.addAgent(position, self.CRIT_DIST, self.num_robots, self.time_horizon,
                        self.sphere_radius, self.DES_VEL, velocity)
            
        # Check if the neighbor robot is already being simulated, otherwise add it to the simulation
        if robot_number not in self.neighbors_id:
            self.neighbors_id[robot_number] = robot_number
            self.neighbors[robot_number] = self.sim.addAgent(neighbor_position, self.CRIT_DIST, self.num_robots,
                                            self.time_horizon, self.sphere_radius, self.DES_VEL, neighbor_velocity)

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

        # If vf points behind the robot recalculate azimuth angle to allow the robot to move backwards
        if len(self.critical_robots) != 0:
            if azimuth <= -pi/2:
                azimuth = pi + azimuth
                vx_behind = True
            
            elif azimuth >= pi/2:
                azimuth = azimuth - pi
                vx_behind = True
        
        # if azimuth is inside the threshold and vf points behind then move backwards while turning
        if abs(azimuth) <= self.AZIMUTH_ERROR and vx_behind:
            self.set_vel_msg.twist.linear.x = -velocity_x
            self.set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.ROT_VEL*azimuth
        
        # if azimuth is inside the threshold and vf points forward then move forward while turning
        elif abs(azimuth) <= self.AZIMUTH_ERROR and not vx_behind:
            self.set_vel_msg.twist.linear.x = velocity_x
            self.set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.ROT_VEL*azimuth
        
        # Otherwise stop and rotate until azimuth is inside the threshold           
        else:
            self.set_vel_msg.twist.linear.x = 0.0
            self.set_vel_msg.twist.angular.z = self.K_ROT_MAX*self.ROT_VEL*azimuth
        
        self.set_vel_msg.twist.linear.z = velocity[2]

        self.set_vel_msg.twist.linear.y = 0.0
        self.set_vel_msg.twist.angular.x = 0.0
        self.set_vel_msg.twist.angular.y = 0.0   

    def create_sphere(self, sphere_x, sphere_y, sphere_z, sphere):
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

        # Create CA sphere with CRIT_DIST radius
        if sphere == "CA sphere":
            marker.scale.x = self.CRIT_DIST
            marker.scale.y = self.CRIT_DIST
            marker.scale.z = self.CRIT_DIST

            if self.enable_CA and self.initial_reached:
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
            
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.25
            
            marker.lifetime = rospy.Duration()
            self.pub_CA_marker.publish(marker)
        
        # Create security sphere with sphere radius
        else:
            marker.scale.x = self.sphere_radius
            marker.scale.y = self.sphere_radius
            marker.scale.z = self.sphere_radius

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.25
        
            marker.lifetime = rospy.Duration()
            self.pub_security_marker.publish(marker)
                  
if __name__ == '__main__':

    try:
        rospy.init_node('collision_avoidance')
        ca = CA(rospy.get_name(), rospy.get_param('~robot_name'))
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass