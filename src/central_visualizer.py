#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 3D animation of the robots path

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cola2_msgs.msg import NavSts
import matplotlib.animation as animation

trajectories = {}
start_tracing = False
robots_initial_reached = set()
already_plotted_robots = set()
lines = []
robot_lines = {}

def callback_position(msg, robot_id):
    if start_tracing:
        if robot_id not in trajectories:
            trajectories[robot_id] = []
        trajectories[robot_id].append((msg.position.north, msg.position.east, msg.position.depth))

def update_fig(num, trajectories, ax):
    global already_plotted_robots

    ax.legend_ = None

    if start_tracing:
        for robot in sorted(trajectories.keys()):
            positions = trajectories[robot]
            xs, ys, zs = zip(*positions)

            if robot in already_plotted_robots:
                robot_lines[robot].set_data(xs, ys)
                robot_lines[robot].set_3d_properties(zs)
            else: 
                line, = ax.plot(xs, ys, zs, label=robot)
                robot_lines[robot] = line
                already_plotted_robots.add(robot)

        ax.legend()
        plt.savefig("/home/arm792/Escritorio/trajectory.png")


def initial_reached(msg):
    global start_tracing
    robots_initial_reached.add(msg.data)
    if len(robots_initial_reached) == num_robots:
        start_tracing = True

def main():
    global num_robots
    rospy.init_node('central_visualizer')
    num_robots = rospy.get_param('~num_robots')

    for i in range(num_robots):
        rospy.Subscriber(
            '/robot' + str(i) + '/navigator/navigation',
            NavSts,
            callback_position,
            callback_args='robot' + str(i),
            queue_size=1)

        rospy.Subscriber(
            '/robot' + str(i) + '/initial_position_reached',
            String,
            initial_reached,
            queue_size=1)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-50, 50])
    ax.set_ylim([-50, 50])
    ax.set_zlim([0, 50])
    
    ani = animation.FuncAnimation(fig, update_fig, fargs=(trajectories, ax), interval=1000)
    
    plt.show()

if __name__ == '__main__':
    main()
