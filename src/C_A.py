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
        self.enable_CA = False # To enable collision avoidance after every robot has reached its initial position
        self.enable_navigation = False # To enable navigation after every robot has reached its initial position
        self.navigate = False  # To navigate through the robot path (To increment path iterator)
        self.keep_position = False  # To indicate whenever the robot must keep position        
        self.stop_goto = False # To indicate whenever the robot must stop navigating
        self.final_reached = False # To indicate the robot has reached the last position of the robot path
        self.start_RVO_simulation = True # To create a new RVO2-3D simualtion
        self.add_neighbor = False # To add a neighbor agent to the RVO2-3D simulation
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vx_obs = 0.0
        self.vy_obs = 0.0
        self.vz_obs = 0.0
        self.path_iterator = 0
        self.critical_robots = set()
        self.robots_initial_reached = set()
        self.path_x = []
        self.path_y = []
        self.path_z = []
        self.obj_distance = 0.0
        self.new_velocity = []
        self.objectives = []
        
        # Get Params
        
        self.num_robots = rospy.get_param('~num_robots')
        self.CRIT_DIST = rospy.get_param('~CRIT_DIST')
        self.GOTO_VEL = rospy.get_param('~GOTO_VEL')
        self.CA_VEL = rospy.get_param('~CA_VEL')
        self.path = rospy.get_param('~path')
        self.initial_x = rospy.get_param('~initial_x')
        self.initial_y = rospy.get_param('~initial_y') 
        self.initial_z = rospy.get_param('~initial_z')
        self.ALGOR = rospy.get_param('~ALGOR')
        self.W1 = rospy.get_param('~W1')
        self.W2 = rospy.get_param('~W2')
        self.K_ROT_MAX = rospy.get_param('~K_ROT_MAX')
        self.K_ROT_MIN = rospy.get_param('~K_ROT_MIN')
        self.AZIMUTH_ERROR = rospy.get_param('~AZIMUTH_ERROR')
        self.ELEVATION_ERROR = rospy.get_param('~ELEVATION_ERROR')
        self.STOP_AND_WAIT = rospy.get_param('~STOP_AND_WAIT')
        self.SAW_PRIO = rospy.get_param('~SAW_PRIO')
        self.radius = rospy.get_param('~radius')
        self.time_step = rospy.get_param('~time_step')
        self.time_horizon = rospy.get_param('~time_horizon') 

        self.robot_priorities = [0] * self.num_robots
        self.detected_robots = [None] * self.num_robots
        self.neighbors = [[None] * 6 for _ in range(self.num_robots)]
        self.neighbors_id = [[None] * 6 for _ in range(self.num_robots)]
        self.obs_distances = [[None] * 6 for _ in range(self.num_robots)]
        self.objectives = [[None for _ in range(3)] for _ in range(self.num_robots)]

        time.sleep(self.num_robots*3)

        # Robot paths        

        if self.path == 'path0':
            self.path_x = [25]
            self.path_y = [50]
            self.path_z = [50]
        elif self.path == 'path1':
            self.path_x = [0]
            self.path_y = [25]
            self.path_z = [50]
        elif self.path == 'path2':
            self.path_x = [25]
            self.path_y = [0]
            self.path_z = [50]
        elif self.path == 'path3':
            self.path_x = [50]
            self.path_y = [25]
            self.path_z = [50]
        elif self.path == 'path4':
            self.path_x = [25]
            self.path_y = [50]
            self.path_z = [0]
        elif self.path == 'path5':
            self.path_x = [0]
            self.path_y = [25]
            self.path_z = [0]
        elif self.path == 'path6':
            self.path_x = [25]
            self.path_y = [0]
            self.path_z = [0]
        elif self.path == 'path7':
            self.path_x = [50]
            self.path_y = [25]
            self.path_z = [0]
        elif self.path == 'path8':
            self.path_x = [50]
            self.path_y = [25]
            self.path_z = [25]
        elif self.path == 'path9':
            self.path_x = [0]
            self.path_y = [25]
            self.path_z = [25]
        
        else:
            rospy.logerr('%s: invalid path parameter', self.robot_name)
            # Default path
            self.path_x = [0]
            self.path_y = [0]
            self.path_z = [0]

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

        # Publishers

        self.pub_marker = rospy.Publisher('/' + self.robot_name + '/sphere', Marker, queue_size=1)
        self.pub_velocity = rospy.Publisher('/' + self.robot_name + '/controller/body_velocity_req', BodyVelocityReq, queue_size=1)
        self.pub_initial_position_reached = rospy.Publisher('/' + self.robot_name + '/initial_position_reached', String, queue_size=1)
        self.pub_priority = rospy.Publisher('/' + self.robot_name + '/priority', Int32, queue_size=1)

        # Subscribers
        
        rospy.Subscriber(
            '/' + self.robot_name + '/navigator/navigation',
            NavSts,
            self.update_robot_position,
            queue_size=1)
        
        for robot in range(self.num_robots):
            rospy.Subscriber(
                    '/robot' + str(robot) + '/initial_position_reached',
                    String,
                    self.start_navigating,
                    queue_size=1)
            if 'robot' + str(robot) != self.robot_name:                                        
                rospy.Subscriber(
                    '/robot' + str(robot) + '/navigator/navigation',
                    NavSts,
                    self.obstacle_detection,
                    queue_size=1)
                rospy.Subscriber(
                    '/robot' + str(robot) + '/priority',
                    Int32,
                    self.get_robot_priority,
                    queue_size=1)
                rospy.Subscriber(
                    '/robot' + str(robot) + '/pilot/goal',
                    PointStamped,
                    self.get_objectives,
                    queue_size=1)                        
        
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

        def goto_thread(): # Service as a thread so the suscription doesnt get blocked (so we can get pose information in update_robot_position())

            self.goto_srv(goto_req)

            # Once the initial position has been reached signal to keep position (wait for the rest) and publish to indicate so
            if not self.initial_reached:
                self.initial_reached = True
                self.keep_position = True
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
                
            # Increase iterator to navigate through the robot path when it is allowed
            elif not self.stop_goto and not self.keep_position and self.obj_distance <= self.radius:
                self.path_iterator += 1
                self.navigate = True

            # If the robot has finished the robot path then stop navigating and keep position
            if self.path_iterator >= len(self.path_x):
                self.keep_position_srv()
                self.path_iterator -= 1
                self.navigate = False
                self.final_reached = True               

        goto_thread = threading.Thread(target=goto_thread)
        goto_thread.start()
        rospy.sleep(1.0)
    
    def start_navigating(self,msg): # Start navigating when all the robots have reached their initial position

        self.robots_initial_reached.add(msg.data) # Add those robots which have reached its initial position to the set
        if len(self.robots_initial_reached) == self.num_robots and not self.enable_CA:
            self.navigate = True # Enable navigation through robot paths
            self.enable_CA = True # Enable collision avoidance

    def update_robot_position(self,msg):

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

        # Draw security sphere for each robot
        self.draw_sphere(self.pos_x, self.pos_y, self.pos_z)

        # Objective distance
        self.obj_distance = sqrt((self.path_x[self.path_iterator] - self.pos_x)**2 + (self.path_y[self.path_iterator] - self.pos_y)**2 +
                            (self.path_z[self.path_iterator] - self.pos_z)**2)

        # Send every robot to its initial position
        if self.first_time:
            self.first_time = False
            self.send_goto_strategy(self.initial_x,self.initial_y,self.initial_z, False, self.GOTO_VEL)                             
        
        # Once the initial position has been reached, keep position and wait for the rest of the robots
        elif self.initial_reached and self.keep_position and not self.enable_navigation:
            self.keep_position = False
            self.keep_position_srv()            

        # When every robot has reached its initial position stop waiting, disable keep position
        elif self.enable_CA and not self.enable_navigation:
            self.enable_navigation = True
            self.stop_keep_position_srv()            

        # Navigate through the robot path if all the robots have reached their initial position and there is no obstacle          
        elif self.navigate and not self.stop_goto and not self.final_reached: 
            self.navigate = False
            self.send_goto_strategy(self.path_x[self.path_iterator],self.path_y[self.path_iterator],self.path_z[self.path_iterator],False, self.GOTO_VEL) 

    def get_robot_priority(self, msg): # Store all robot priorities by robot number

        # Parse the robot's number from the topic name
        robot_number = int(msg._connection_header['topic'].split('/')[1].replace('robot', ''))
        self.robot_priorities[robot_number] = msg.data

    def get_objectives(self, msg): # Store all robot objectives

        # Parse the robot's number from the topic name
        robot_number = int(msg._connection_header['topic'].split('/')[1].replace('robot', ''))
        self.objectives[robot_number][0] = msg.point.x
        self.objectives[robot_number][1] = msg.point.y
        self.objectives[robot_number][2] = msg.point.z          

    def obstacle_detection(self, msg):

        if self.enable_CA and not self.final_reached and not self.navigate:
            # Parse the robot's number from the topic name
            robot_number = int(msg._connection_header['topic'].split('/')[1].replace('robot', ''))
            # Calculate Euclidean distance
            self.obs_distances[robot_number] = sqrt((msg.position.north - self.pos_x)**2 + (msg.position.east - self.pos_y)**2 + (msg.position.depth - self.pos_z)**2)

            if self.obs_distances[robot_number] <= self.radius:
                print(self.robot_name + ': ' + str(self.obs_distances[robot_number]))

            # Check distance to detect the robot as an obstacle
            if self.obs_distances[robot_number] <= self.CRIT_DIST:

                # Check if the neighbor robot is already being simulated, otherwise add it to the simulation
                if self.ALGOR == 2 and robot_number not in self.neighbors_id:
                    self.add_neighbor = True

                self.critical_robots.add(robot_number)  # Add the robot name to the set to know when to apply CA algorithm
                self.detected_robots[robot_number] = self.robot_priorities[robot_number] # Add robot priority for the Stop&Wait strategy
                self.neighbors_id[robot_number] = robot_number # Add the robot as a neighbor agent for the RVO2-3D algorithm

                if not self.stop_goto:  # Shutdown goto service
                    self.stop_goto = True
                    self.stop_goto_srv()

                # If Stop&Wait strategy is enabled and the priority is the highest then apply CA algorithm, if it isnt enabled apply CA algorithm anyway                
                if (self.STOP_AND_WAIT and self.SAW_PRIO < min(x for x in self.detected_robots if x is not None)) or not self.STOP_AND_WAIT:

                    # Stop keeping position
                    if self.keep_position:
                        self.keep_position = False
                        self.stop_keep_position_srv()

                    # Apply potential fields
                    if self.ALGOR == 1:                    
                        self.potential_fields(self.obs_distances[robot_number], msg.position.north, msg.position.east, msg.position.depth)
                    
                    # Apply RVO2-3D
                    elif self.ALGOR == 2:
                        self.reciprocal_velocity(msg.position.north, msg.position.east, msg.position.depth,
                                                msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z, robot_number)
                
                # If the robot is not the one with the highest priority just keep position until it is
                elif not self.keep_position:
                    self.keep_position = True
                    self.keep_position_srv()                    

            elif robot_number in self.critical_robots:  # If the robot was in the critical range and is now no longer

                self.critical_robots.remove(robot_number)  # Remove the robot name from the set
                self.detected_robots[robot_number] = None # Remove robot priority
                self.obs_distances[robot_number] = None # Remove robot distance since it is no longer considered an obstacle
                # If a robot is no longer considered an obstacle then restart simulation to remove it
                # Those robots which are still considered obstacles will be automatically added to the simulation
                self.neighbors_id = [[None] * 6 for _ in range(self.num_robots)]
                self.start_RVO_simulation = True

                if len(self.critical_robots) == 0:  # If there are no robots in the critical range, allow navigation to resume
                    if self.keep_position: # Shutdown keep position service
                        self.keep_position = False
                        self.stop_keep_position_srv()                         
                        
                    self.stop_goto = False
                    self.navigate = True  # Resume navigation        

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
        vx_obj = (self.path_x[self.path_iterator] - self.pos_x)/self.obj_distance
        vy_obj = (self.path_y[self.path_iterator] - self.pos_y)/self.obj_distance
        vz_obj = (self.path_z[self.path_iterator] - self.pos_z)/self.obj_distance

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

        set_vel_msg = BodyVelocityReq()

        set_vel_msg.header.frame_id = rospy.get_namespace() + "base_link"
        set_vel_msg.header.stamp = rospy.Time.now()
        set_vel_msg.goal.priority = 40
        set_vel_msg.goal.requester = rospy.get_name() + "_velocity_req"

        set_vel_msg.disable_axis.x = False
        set_vel_msg.disable_axis.y = True
        set_vel_msg.disable_axis.z = False
        set_vel_msg.disable_axis.roll = True
        set_vel_msg.disable_axis.pitch = True
        set_vel_msg.disable_axis.yaw = False

        # Check if azimuth error is within the defined threshold
        if abs(azimuth) <= self.AZIMUTH_ERROR:
            # Set linear x velocity only if azimuth error is small enough
            set_vel_msg.twist.linear.x = self.CA_VEL 
            # Set angular z velocity considering the azimuth error
            set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.CA_VEL*azimuth
        else:
            # Stop linear x velocity if azimuth error is too large
            set_vel_msg.twist.linear.x = 0.0
            # Set angular z velocity considering the azimuth error
            set_vel_msg.twist.angular.z = self.K_ROT_MAX*self.CA_VEL*azimuth

        # Check if elevation error is within the defined threshold
        if abs(elevation) <= self.ELEVATION_ERROR:
            # If elevation error is small enough, don't change z direction
            set_vel_msg.twist.linear.z = 0.0
        elif elevation > self.ELEVATION_ERROR:
            # If robot needs to go up, set positive z velocity
            set_vel_msg.twist.linear.z = self.CA_VEL
        else: # elevation < -self.ELEVATION_ERROR
            # If robot needs to go down, set negative z velocity
            set_vel_msg.twist.linear.z = -self.CA_VEL

        set_vel_msg.twist.linear.y = 0.0
        set_vel_msg.twist.angular.x = 0.0
        set_vel_msg.twist.angular.y = 0.0
 
        self.pub_velocity.publish(set_vel_msg)
    
    def reciprocal_velocity(self, obs_pos_x, obs_pos_y, obs_pos_z, obs_vel_x, obs_vel_y, obs_vel_z, robot_number):

        # Get main agent position and velocity
        position = (self.pos_x, self.pos_y, self.pos_z)
        velocity = (self.vx, self.vy, self.vz)

        # Get main agent preferred velocity
        vx_obj = self.GOTO_VEL*(self.path_x[self.path_iterator] - self.pos_x)/self.obj_distance
        vy_obj = self.GOTO_VEL*(self.path_y[self.path_iterator] - self.pos_y)/self.obj_distance
        vz_obj = self.GOTO_VEL*(self.path_z[self.path_iterator] - self.pos_z)/self.obj_distance

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
            self.sim = rvo_wrapper.PyRVOSimulator(self.time_step, self.CRIT_DIST, self.num_robots - 1, self.time_horizon, self.radius, self.GOTO_VEL, velocity)
            self.agent = self.sim.addAgent(position, self.CRIT_DIST, self.num_robots - 1, self.time_horizon, self.radius, self.GOTO_VEL, velocity)
            self.start_RVO_simulation = False
        
        # Add neighbor agents to the simulation
        if self.add_neighbor:
            self.neighbors[robot_number] = self.sim.addAgent(neighbor_position, self.CRIT_DIST, self.num_robots - 1,
                                            self.time_horizon, self.radius, self.GOTO_VEL, neighbor_velocity)     
            self.add_neighbor = False

        # Update main agent and neighbor agent preferred velocities
        self.sim.setAgentPrefVelocity(self.agent, preferred_velocity)
        self.sim.setAgentPrefVelocity(self.neighbors[robot_number], neigh_preferred_velocity)

        # Update main agent and neighbor positions and velocities
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
        # Azimuth and elevation of the final vector from the robot's pose
        azimuth = atan2(sin(azimuth - self.yaw), cos(azimuth - self.yaw))

        new_velocity = sqrt((velocity[0])**2 + (velocity[1])**2)

        set_vel_msg = BodyVelocityReq()

        set_vel_msg.header.frame_id = rospy.get_namespace() + "base_link"
        set_vel_msg.header.stamp = rospy.Time.now()
        set_vel_msg.goal.priority = 40
        set_vel_msg.goal.requester = rospy.get_name() + "_velocity_req"

        set_vel_msg.disable_axis.x = False
        set_vel_msg.disable_axis.y = True
        set_vel_msg.disable_axis.z = False
        set_vel_msg.disable_axis.roll = True
        set_vel_msg.disable_axis.pitch = True
        set_vel_msg.disable_axis.yaw = False

        # Check if azimuth error is within the defined threshold
        if abs(azimuth) <= self.AZIMUTH_ERROR:
            set_vel_msg.twist.linear.x = new_velocity
            # Set angular z velocity considering the azimuth error
            set_vel_msg.twist.angular.z = self.K_ROT_MIN*self.CA_VEL*azimuth
        else:
            set_vel_msg.twist.linear.x = 0.0
            # Set angular z velocity considering the azimuth error
            set_vel_msg.twist.angular.z = self.K_ROT_MAX*self.CA_VEL*azimuth

        set_vel_msg.twist.linear.z = velocity[2]
        set_vel_msg.twist.linear.y = 0.0
        set_vel_msg.twist.angular.x = 0.0
        set_vel_msg.twist.angular.y = 0.0
 
        self.pub_velocity.publish(set_vel_msg)  
        
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
        marker.scale.x = self.radius
        marker.scale.y = self.radius
        marker.scale.z = self.radius        
        
        if self.stop_goto and any(item is not None and item < self.radius for item in self.obs_distances):
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.25

        elif self.stop_goto and any(item is not None and item < self.CRIT_DIST for item in self.obs_distances):
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