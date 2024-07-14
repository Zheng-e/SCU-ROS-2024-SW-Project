#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import turtlesim.srv
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from assignment_scu_39.msg import GroupidLeaderMessage
from std_msgs.msg import Empty
import math
import random

follower_pose = Pose()
leader_pose = Pose()
initial_pose = None
pub = None
in_position_pub = None

def update_follower_pose(data):
    global follower_pose
    follower_pose = data

def update_leader_pose(data):
    global leader_pose
    leader_pose = data

def move_to_position(target_x, target_y):
    global follower_pose
    vel_msg = Twist()
    distance = ((follower_pose.x - target_x)**2 + (follower_pose.y - target_y)**2) ** 0.5
    while distance >= 0.1 and not rospy.is_shutdown():
        vel_msg.linear.x = 1.5 * distance
        vel_msg.angular.z = 4 * (math.atan2(target_y - follower_pose.y, target_x - follower_pose.x) - follower_pose.theta)
        pub.publish(vel_msg)
        distance = ((follower_pose.x - target_x)**2 + (follower_pose.y - target_y)**2) ** 0.5
        rospy.sleep(0.1)
    # Stop the turtle after reaching the target
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)

def callback(data):
    rospy.wait_for_service('spawn')
    spawn_turtle = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    rospy.wait_for_service('kill')
    kill_turtle = rospy.ServiceProxy('kill', turtlesim.srv.Kill)
    if data.command == "return":
        rospy.loginfo("Returning to initial position")
        if initial_pose:
            # 杀死原来的乌龟
            try:
                kill_turtle('scuGroupidFollowerA')
            except rospy.ServiceException as e:
                rospy.logwarn("Failed to kill turtle: %s" % e)
            
            # 生成新乌龟
            spawn_turtle(initial_pose.x, initial_pose.y, initial_pose.theta, 'scuGroupidFollowerA')
            # 停止乌龟的运动
            stop_msg = Twist()
            pub.publish(stop_msg)
            in_position_pub.publish(Empty())
            rospy.loginfo("Follower has returned to the initial position.")
            rospy.signal_shutdown("Follower has returned to the initial position.")

def followerB():
    global initial_pose, pub, in_position_pub
    rospy.init_node('scuGroupidFollowerB', anonymous=True)
    pub = rospy.Publisher('/scuGroupidFollowerB/cmd_vel', Twist, queue_size=10)
    in_position_pub = rospy.Publisher('/scuGroupidFollowerB/in_position', Empty, queue_size=10)
    rospy.Subscriber('/scuGroupidLeader/info', GroupidLeaderMessage, callback)
    rospy.Subscriber('/scuGroupidFollowerB/pose', Pose, update_follower_pose)
    rospy.Subscriber('/scuGroupidLeader/pose', Pose, update_leader_pose)
    rate = rospy.Rate(10)

    rospy.wait_for_service('spawn')
    spawn_turtle = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    initial_pose = Pose()
    initial_pose.x = random.uniform(1.0, 10.0)
    initial_pose.y = random.uniform(1.0, 10.0)
    initial_pose.theta = random.uniform(0, 6.28)
    spawn_turtle(initial_pose.x, initial_pose.y, initial_pose.theta, 'scuGroupidFollowerB')

    rospy.sleep(1)

    while not rospy.is_shutdown():
        if leader_pose:
            target_x = leader_pose.x - math.cos(leader_pose.theta + 0.785)*(2**0.5)
            target_y = leader_pose.y - math.sin(leader_pose.theta + 0.785)*(2**0.5)
            follower_pose.theta = leader_pose.theta
            move_to_position(target_x, target_y)
            in_position_pub.publish(Empty())
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        followerB()
    except rospy.ROSInterruptException:
        pass

