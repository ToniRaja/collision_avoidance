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
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker
import time
import threading

class CA:

    def __init__(self, name, robot_name):
        self.robot_name = robot_name
        self.first_time = True # To send the robot to its initial position
        self.initial_reached = False # To indicate the robot has reached its initial position
        self.enable_CA = False # To enable collision avoidance after every robot has reached its initial position
        self.enable_navigation = False # To enable navigation after every robot has reached its initial position
        self.navigate = False  # To navigate through the robot path
        self.keep_position = False  # To indicate whenever the robot must keep position        
        self.stop_goto = False # To indicate whenever the robot must stop navigating
        self.final_reached = False # To indicate the robot has reached the last position of the robot path
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vx_obs = 0.0
        self.vy_obs = 0.0
        self.vz_obs = 0.0
        self.path_iterator = 0
        self.ori = 0.0
        self.critical_robots = set()  # Initialize the set of critical robots
        self.robots_initial_reached = set()
        self.path_x = []
        self.path_y = []
        self.path_z = []
        self.obj_distance = 0.0
        
        # Get Params
        
        self.num_robots = rospy.get_param('~num_robots')
        self.CRIT_DIST = rospy.get_param('~CRIT_DIST')
        self.GOTO_VEL = rospy.get_param('~GOTO_VEL')
        self.CA_VEL = rospy.get_param('~CA_VEL')
        self.path = rospy.get_param('~path')
        self.initial_x = rospy.get_param('~initial_x')
        self.initial_y = rospy.get_param('~initial_y') 
        self.initial_z = rospy.get_param('~initial_z')
        self.W1 = rospy.get_param('~W1')
        self.W2 = rospy.get_param('~W2')
        self.K_ROT_MAX = rospy.get_param('~K_ROT_MAX')
        self.K_ROT_MIN = rospy.get_param('~K_ROT_MIN')
        self.AZIMUTH_ERROR = rospy.get_param('~AZIMUTH_ERROR')
        self.ELEVATION_ERROR = rospy.get_param('~ELEVATION_ERROR')
        self.STOP_AND_WAIT = rospy.get_param('~STOP_AND_WAIT')
        self.SAW_PRIO = rospy.get_param('~SAW_PRIO')

        self.robot_priorities = [0] * self.num_robots
        self.detected_robots = [None] * self.num_robots

        # Robot paths        

        if self.path == 'path0':
            self.path_x = [20]
            self.path_y = [0]
            self.path_z = [0]
        elif self.path == 'path1':
            self.path_x = [0]
            self.path_y = [0]
            self.path_z = [0]
        elif self.path == 'path2':
            self.path_x = [30]
            self.path_y = [0]
            self.path_z = [10]
        elif self.path == 'path3':
            self.path_x = [30]
            self.path_y = [50]
            self.path_z = [5]
        elif self.path == 'path4':
            self.path_x = [30]
            self.path_y = [50]
            self.path_z = [15]
        elif self.path == 'path5':
            self.path_x = [50]
            self.path_y = [0]
            self.path_z = [0]
        elif self.path == 'path6':
            self.path_x = [50]
            self.path_y = [0]
            self.path_z = [0]
        elif self.path == 'path7':
            self.path_x = [50]
            self.path_y = [0]
            self.path_z = [0]
        
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
                    self.update_robot_priority,
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
            elif not self.stop_goto and not self.keep_position and self.obj_distance <= self.CRIT_DIST:
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
        # Get pose information
        self.pos_x = msg.position.north
        self.pos_y = msg.position.east
        self.pos_z = msg.position.depth
        self.roll = msg.orientation.roll
        self.pitch = msg.orientation.pitch
        self.yaw = msg.orientation.yaw

        # Draw security sphere for each robot
        self.draw_sphere(self.pos_x, self.pos_y, self.pos_z)

        # Objective distance
        self.obj_distance = sqrt((self.path_x[self.path_iterator] - self.pos_x)**2 + (self.path_y[self.path_iterator] - self.pos_y)**2 + (self.path_z[self.path_iterator] - self.pos_z)**2)

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

    def update_robot_priority(self, msg): # Store all robot priorities by robot number
        # Parse the robot's number from the topic name
        robot_number = int(msg._connection_header['topic'].split('/')[1].replace('robot', ''))
        self.robot_priorities[robot_number] = msg.data           

    def obstacle_detection(self, msg):
        if self.enable_CA and not self.final_reached and not self.navigate:
            # Calculate Euclidean distance
            distance = sqrt((msg.position.north - self.pos_x) ** 2 + (msg.position.east - self.pos_y) ** 2 + (msg.position.depth - self.pos_z) ** 2)
            # Parse the robot's number from the topic name
            robot_number = int(msg._connection_header['topic'].split('/')[1].replace('robot', ''))
            # Check distance to detect the robot as an obstacle
            if distance <= self.CRIT_DIST:
                self.critical_robots.add(robot_number)  # Add the robot name to the set
                self.detected_robots[robot_number] = self.robot_priorities[robot_number] # Add robot priority
                if not self.stop_goto:  # Shutdown goto service
                    self.stop_goto = True
                    self.stop_goto_srv()

                # If Stop&Wait strategy is enabled and the priority is the highest then apply potential fields, if it isnt enabled apply potential fields anyway                
                if (self.STOP_AND_WAIT and self.SAW_PRIO < min(x for x in self.detected_robots if x is not None)) or not self.STOP_AND_WAIT:
                    if self.keep_position:
                        self.keep_position = False
                        self.stop_keep_position_srv()

                    self.potential_fields(distance, msg.position.north, msg.position.east, msg.position.depth)
                
                # If the robot is not the one with the highest priority just keep position until it is
                elif not self.keep_position:
                    self.keep_position = True
                    self.keep_position_srv()                    

            elif robot_number in self.critical_robots:  # If the robot was in the critical range and is now no longer
                self.critical_robots.remove(robot_number)  # Remove the robot name from the set
                self.detected_robots[robot_number] = None # Remove robot priority
                if len(self.critical_robots) == 0:  # If there are no robots in the critical range, allow navigation to resume
                    if self.keep_position: # Shutdown keep position service
                        self.keep_position = False
                        self.stop_keep_position_srv()                         
                        
                    self.stop_goto = False
                    self.navigate = True  # Resume navigation          

    def potential_fields(self, obs_distance, obs_posx, obs_posy, obs_posz):

        # Vi magnitude
        Vi = (self.CRIT_DIST - obs_distance)/self.CRIT_DIST

        # Vobstacle unit vector
        Vix = Vi*(self.pos_x - obs_posx)/obs_distance
        Viy = Vi*(self.pos_y - obs_posy)/obs_distance
        Viz = Vi*(self.pos_z - obs_posz)/obs_distance

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
        marker.scale.x = self.CRIT_DIST
        marker.scale.y = self.CRIT_DIST
        marker.scale.z = self.CRIT_DIST

        if not self.stop_goto:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.25
        
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.25

        marker.lifetime = rospy.Duration()
        self.pub_marker.publish(marker)
                  
if __name__ == '__main__':

    try:
        time.sleep(25)
        rospy.init_node('collision_avoidance')
        ca = CA(rospy.get_name(), rospy.get_param('~robot_name'))
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass