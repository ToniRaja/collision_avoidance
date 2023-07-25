#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dis import dis
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from math import *
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts, BodyVelocityReq
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
        self.name = name
        self.robot_name = robot_name
        self.navigate = True
        self.goal_reached = False
        self.enable_CA = False
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.goto_running = False 
        self.stop_goto = False
        self.vx_obs = 0.0
        self.vy_obs = 0.0
        self.path_iterator = 0
        self.ori = 0.0
        self.critical_robots = set()  # Initialize the set of critical robots
        
        # Get Params
        
        self.num_robots = rospy.get_param('~num_robots')
        self.CRIT_DIST = rospy.get_param('~CRIT_DIST')
        self.path = rospy.get_param('~path')
        if self.path == 'path0':
            self.path_x = [40,0]
            self.path_y = [0,40]
        elif self.path == 'path1':
            self.path_x = [0,40]
            self.path_y = [40,0]
        elif self.path == 'path2':
            self.path_x = [40,0,0,40,40,0,0,40,40,0]
            self.path_y = [40,40,30,30,20,20,10,10,0,0]
        else:
            rospy.logerr('%s: invalid path parameter', self.name)
            # Default path
            self.path_x = [0]
            self.path_y = [0]           

        # Publishers

        self.pub_marker = rospy.Publisher('/' + self.robot_name + '/sphere', Marker, queue_size=1)
        # self.pub_initial_position = rospy.Publisher('/' + self.robot_name + '/dynamics/odometry', Odometry, queue_size=1)
        self.pub_velocity = rospy.Publisher('/' + self.robot_name + '/controller/body_velocity_req', BodyVelocityReq, queue_size=1)

        # Subscribers
        
        rospy.Subscriber(
            '/' + self.robot_name + '/navigator/navigation',
            NavSts,
            self.update_robot_position,
            queue_size=1)
            
        for robot in range(self.num_robots):
            if 'robot' + str(robot) != self.robot_name:
                rospy.Subscriber(
                    '/robot' + str(robot) + '/navigator/navigation',
                    NavSts,
                    self.obstacle_detection,
                    queue_size=1)

        # Services clients
        try:
            rospy.wait_for_service('/' + self.robot_name + '/captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/' + self.robot_name + '/captain/enable_goto', Goto)
            rospy.wait_for_service('/' + self.robot_name + '/captain/disable_goto', 20)
            self.stop_goto_srv = rospy.ServiceProxy(
                        '/' + self.robot_name + '/captain/disable_goto', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')        

    def send_goto_strategy(self, position_x, position_y, keep_position, linear_velocity):
        """Goto to position x, y, z, at velocity vel."""
        # Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = linear_velocity
        goto_req.position.x = position_x
        goto_req.position.y = position_y
        goto_req.position.z = 0.0
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

        def goto_thread(): # Service as a thread so the suscription doesnt get blocked
            self.goto_srv(goto_req)
            if not self.stop_goto:
                self.path_iterator += 1
                self.navigate = True
            if self.path_iterator >= len(self.path_x):
                self.goal_reached = True
                self.navigate = False
            elif self.path_iterator == 1:
                self.enable_CA = True
                self.navigate = True

        goto_thread = threading.Thread(target=goto_thread)
        goto_thread.start()
        rospy.sleep(1.0)

    def update_robot_position(self,msg):
        self.pose_x = msg.position.north
        self.pose_y = msg.position.east
        self.pose_z = msg.position.depth
        self.ori = msg.orientation.yaw

        self.draw_sphere(self.pose_x, self.pose_y, self.pose_z)

        if self.navigate and not self.stop_goto:
            self.navigate = False
            self.send_goto_strategy(self.path_x[self.path_iterator],self.path_y[self.path_iterator],False, 0.5)

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

    def obstacle_detection(self, msg):
        if self.enable_CA:
            distance = sqrt((msg.position.north - self.pose_x) ** 2 + (msg.position.east - self.pose_y) ** 2)
            if distance <= self.CRIT_DIST:
                self.critical_robots.add(msg.header.frame_id)  # Add the robot name to the set
                if not self.stop_goto:  # Shutdown goto service
                    self.stop_goto = True
                    self.stop_goto_srv()

                self.potential_fields(distance, msg.position.north, msg.position.east)
            elif msg.header.frame_id in self.critical_robots:  # If the robot was in the critical range and is now no longer
                self.critical_robots.remove(msg.header.frame_id)  # Remove the robot name from the set
                if len(self.critical_robots) == 0:  # If there are no robots in the critical range, allow navigation to resume
                    self.stop_goto = False
                    self.navigate = True  # Resume navigation                 

    def potential_fields(self, distance, obs_posx, obs_posy):
        W1 = 1.0
        W2 = 3.0
        K_ROT_MAX = 0.3
        K_ROT_MIN = 0.1
        V_MAX = 0.5
        ORI_ERROR = 0.1
        
        Vi = (self.CRIT_DIST - distance)/self.CRIT_DIST

        Vix = Vi*(self.pose_x - obs_posx)/distance
        Viy = Vi*(self.pose_y - obs_posy)/distance

        self.vx_obs += Vix
        self.vy_obs += Viy

        vx_obj = cos(atan2(self.path_y[self.path_iterator] - self.pose_y, self.path_x[self.path_iterator] - self.pose_x))
        vy_obj = sin(atan2(self.path_y[self.path_iterator]- self.pose_y, self.path_x[self.path_iterator] - self.pose_x))

        vfx = W1*vx_obj + W2*self.vx_obs
        vfy = W1*vy_obj + W2*self.vy_obs

        robot_orientation =  self.ori
        
        direction_angle = atan2(vfy,vfx)
        direction_angle = atan2(sin(direction_angle - robot_orientation),cos(direction_angle - robot_orientation))

        set_vel_msg = BodyVelocityReq()

        set_vel_msg.header.frame_id = rospy.get_namespace() + "base_link"
        set_vel_msg.header.stamp = rospy.Time.now()
        set_vel_msg.goal.priority = 40
        set_vel_msg.goal.requester = rospy.get_name() + "_velocity_req"

        if(abs(direction_angle) > ORI_ERROR):
            set_vel_msg.disable_axis.x = True
            set_vel_msg.disable_axis.y = True
            set_vel_msg.disable_axis.z = True
            set_vel_msg.disable_axis.roll = True
            set_vel_msg.disable_axis.pitch = True
            set_vel_msg.disable_axis.yaw = False
            set_vel_msg.twist.linear.x = 0.0
            set_vel_msg.twist.linear.y = 0.0
            set_vel_msg.twist.linear.z = 0.0
            set_vel_msg.twist.angular.x = 0.0
            set_vel_msg.twist.angular.y = 0.0
            set_vel_msg.twist.angular.z = K_ROT_MAX*V_MAX*direction_angle

        else:
            set_vel_msg.disable_axis.x = False
            set_vel_msg.disable_axis.y = True
            set_vel_msg.disable_axis.z = True
            set_vel_msg.disable_axis.roll = True
            set_vel_msg.disable_axis.pitch = True
            set_vel_msg.disable_axis.yaw = False
            set_vel_msg.twist.linear.x = V_MAX
            set_vel_msg.twist.linear.y = 0.0
            set_vel_msg.twist.linear.z = 0.0
            set_vel_msg.twist.angular.x = 0.0
            set_vel_msg.twist.angular.y = 0.0
            set_vel_msg.twist.angular.z = K_ROT_MIN*V_MAX*direction_angle

        self.pub_velocity.publish(set_vel_msg)
                  
if __name__ == '__main__':

    try:
        time.sleep(4)
        rospy.init_node('collision_avoidance')
        ca = CA(rospy.get_name(), rospy.get_param('~robot_name'))
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass