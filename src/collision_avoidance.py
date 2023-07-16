#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cola2_msgs.msg import BodyVelocityReq

rospy.init_node('collision_avoidance')

self.robot_ID = 'robot0'
self.surge_velocity = 0.6



# Services clients
try:
    rospy.wait_for_service('/robot'+str(self.robot_ID)+'/captain/enable_goto', 20)
    self.goto_srv = rospy.ServiceProxy(
                '/robot'+str(self.robot_ID)+'/captain/enable_goto', Goto)
except rospy.exceptions.ROSException:
    rospy.logerr('%s: error creating client to goto service',
                    self.name)
    rospy.signal_shutdown('Error creating client to goto service')

# # Crea el mensaje BodyVelocityReq para modificar la velocidad del cuerpo
# body_velocity_msg = BodyVelocityReq()
# body_velocity_msg.goal.priority = 40
# body_velocity_msg.twist.linear.x = 10.0
# body_velocity_msg.twist.linear.y = 0.0
# body_velocity_msg.twist.linear.z = 0.0
# body_velocity_msg.twist.angular.x = 0.0  
# body_velocity_msg.twist.angular.y = 0.0  
# body_velocity_msg.twist.angular.z = 0.0  

# # Publica la velocidad del cuerpo constantemente
# pub = rospy.Publisher('/robot0/controller/body_velocity_req', BodyVelocityReq)
# rate = rospy.Rate(10)  # Frecuencia de publicación de 10 Hz


def send_goto_strategy(self, position_x, position_y,keep_position):
        # self.disable_all_and_set_idle_srv()
        """Goto to position x, y, z, at velocity vel."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = self.surge_velocity
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

while not rospy.is_shutdown():
    # Publica el mensaje BodyVelocityReq en el topic
    # pub.publish(body_velocity_msg)
    self.send_goto_strategy(10,10,False)

    rate.sleep()
