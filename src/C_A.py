#!/usr/bin/env python
from dis import dis
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from math import *
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts
from cola2_msgs.srv import Goto, GotoRequest
from sensor_msgs.msg import BatteryState
import numpy as np
from std_srvs.srv import Empty, EmptyResponse

class CA:

    def __init__(self, name, robot_name):
        self.name = name
        self.robot_name = robot_name
        # self.tolerance = self.get_param('tolerance',2)
        # self.surge_velocity = self.get_param('surge_velocity',0.5)
        # self.battery_topic = self.get_param('~battery_topic','/turbot1/batteries/status')
        # self.section_action = self.get_param('~section_action','/xiroi/pilot/world_section_req') 
        # self.section_result = self.get_param('~section_result','/xiroi/pilot/world_section_req/result') 

        # self.navigation_depth = self.get_param('~navigation_depth',10)
        # self.robot_ID = self.get_param('~robot_ID',0)
        # self.robot_name = self.get_param('~robot_name','turbot1')
        # self.distance = []
        # self.travelled_distance = []
        # self.robots_travelled_distances = [0,0,0,0,0,0]
        # self.robot_alive = False
        # self.is_section_actionlib_running = False
        # self.battery_status = [0,0,0]
        # self.first_time = True
        # self.ns = rospy.get_namespace()
        self.first_time = True
        self.robot0_goal_reached = False
        self.robot1_goal_reached = False
        self.robot0_goal_x = 50
        self.robot0_goal_y = 0
        self.robot1_goal_x = 0
        self.robot1_goal_y = 0
        self.pose_x = 0
        self.pose_y = 0
        self.stop_goto_service = False

        # Subscriber
        
        rospy.Subscriber(
            '/' + self.robot_name + '/navigator/navigation',
            NavSts,
            self.update_robot_position,
            queue_size=1)
        
        rospy.Subscriber(
            '/' + self.robot_name + '/navigator/navigation',
            NavSts,
            self.get_pose,
            queue_size=1)
        
        if(self.robot_name == 'robot0'):
            rospy.Subscriber(
                '/robot1/navigator/navigation',
                NavSts,
                self.force_collision,
                queue_size=1)
        
        elif(self.robot_name == 'robot1'):
            rospy.Subscriber(
                '/robot0/navigator/navigation',
                NavSts,
                self.force_collision,
                queue_size=1)

        # Services clients
        try:
            rospy.wait_for_service('/' + self.robot_name + '/captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/' + self.robot_name + '/captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')

    def send_goto_strategy(self, position_x, position_y,keep_position):
        # self.disable_all_and_set_idle_srv()
        """Goto to position x, y, z, at velocity vel."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 0.5
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
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)
        rospy.sleep(1.0) 
    
    def update_robot_position(self,msg):
        # fill the robots_information array with the robots information received from the NavSts 
        # self.robots_information[robot_id][0] = msg.position.north
        # self.robots_information[robot_id][1] = msg.position.east
        # self.robots_information[robot_id][2] = msg.position.depth
        # self.robots_information[robot_id][3] = msg.altitude
        # self.robots_information[robot_id][4] = msg.global_position.latitude
        # self.robots_information[robot_id][5] = msg.global_position.longitude
        # self.robots_information[robot_id][6] = msg.body_velocity.x
        # self.robots_information[robot_id][7] = msg.body_velocity.y
        # self.robots_information[robot_id][8] = msg.body_velocity.z
        # self.robots_information[robot_id][9] = msg.orientation.roll
        # self.robots_information[robot_id][10] = msg.orientation.pitch
        # self.robots_information[robot_id][11] = msg.orientation.yaw

        if(self.first_time == True and self.robot_name == 'robot0'):
            self.first_time = False
            self.send_goto_strategy(self.robot0_goal_x,self.robot0_goal_y,False)
            self.robot0_goal_reached = True
            
        elif(self.first_time == True and self.robot_name == 'robot1'):
            self.first_time = False
            #self.send_goto_strategy(self.robot1_goal_x,self.robot1_goal_y,False)
            self.robot1_goal_reached = True 

        if(self.robot0_goal_reached and self.robot1_goal_reached and self.robot_name == 'robot0'):
            self.send_goto_strategy(self.robot1_goal_x,self.robot1_goal_y,False)
        
        elif(self.robot0_goal_reached and self.robot1_goal_reached and self.robot_name == 'robot1'):
            self.send_goto_strategy(self.robot0_goal_x,self.robot0_goal_y,False)
    
    def force_collision(self, msg):
        if self.robot_name == 'robot0' and self.robot0_goal_reached and not self.robot1_goal_reached:
            if msg.position.north <= self.robot1_goal_x + 5 and msg.position.north >= self.robot1_goal_x - 5 and msg.position.east <= self.robot1_goal_y + 5 and msg.position.east >= self.robot1_goal_y - 5:
                self.robot1_goal_reached = True

        elif self.robot_name == 'robot1' and self.robot1_goal_reached and not self.robot0_goal_reached:
            if msg.position.north <= self.robot0_goal_x + 5 and msg.position.north >= self.robot0_goal_x - 5 and msg.position.east <= self.robot0_goal_y + 5 and msg.position.east >= self.robot0_goal_y - 5:
                self.robot0_goal_reached = True

        if self.robot0_goal_reached and self.robot1_goal_reached and not self.stop_goto_service:
            self.update_goto_service_status(self.pose_x, self.pose_y, msg.position.north, msg.position.east)

    def update_goto_service_status(self, pose_x, pose_y, other_pose_x, other_pose_y):
        security_distance = 5

        distance = sqrt((other_pose_x - pose_x) ** 2 + (other_pose_y - pose_y) ** 2)

        if distance <= security_distance:
            self.stop_goto_service = True
            self.goto_srv.close()
            print('hola')

        else:
            self.stop_goto_service = False
    
    def get_pose(self, msg):
        self.pose_x = msg.position.north
        self.pose_y = msg.position.east
  
    # def get_param(self, param_name, default = None):
    #     if rospy.has_param(param_name):
    #         param_value = rospy.get_param(param_name)
    #         return param_value
    #     elif default is not None:
    #         return default
    #     else:
    #         rospy.logfatal('[%s]: invalid parameters for %s in param server!', self.name, param_name)
    #         rospy.logfatal('[%s]: shutdown due to invalid config parameters!', self.name)
    #         exit(0)
              
if __name__ == '__main__':
    try:
        rospy.init_node('collision_avoidance')
        ca = CA(rospy.get_name(), rospy.get_param('~robot_name', 'robot0'))
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass