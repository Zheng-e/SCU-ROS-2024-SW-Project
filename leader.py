#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import turtlesim.srv
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from assignment_scu_39.msg import GroupidLeaderMessage
from std_msgs.msg import Empty
import random

leader_pose = Pose()
in_position_a = False
in_position_b = False
initial_pose = None

def update_leader_pose(data):
    global leader_pose
    leader_pose = data

def followerA_in_position(data):
    global in_position_a
    in_position_a = True

def followerB_in_position(data):
    global in_position_b
    in_position_b = True

def check_followers_in_position():
    return in_position_a and in_position_b

def move_random(pub):
    vel_msg = Twist()
    vel_msg.linear.x = random.uniform(0.5, 1.0)
    vel_msg.angular.z = random.uniform(-1.0, 1.0)
    pub.publish(vel_msg)

def at_boundary():
    return leader_pose.x < 1.0 or leader_pose.x > 10.0 or leader_pose.y < 1.0 or leader_pose.y > 10.0

def teleport_to_initial_position():
    global initial_pose
    rospy.wait_for_service('/scuGroupidLeader/teleport_absolute')
    try:
        teleport_turtle = rospy.ServiceProxy('/scuGroupidLeader/teleport_absolute', turtlesim.srv.TeleportAbsolute)
        teleport_turtle(5.5, 5.5, 0)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def leader():
    global initial_pose
    rospy.init_node('scuGroupidLeader', anonymous=True)
    pub = rospy.Publisher('/scuGroupidLeader/cmd_vel', Twist, queue_size=10)
    info_pub = rospy.Publisher('/scuGroupidLeader/info', GroupidLeaderMessage, queue_size=10)
    rospy.Subscriber('/scuGroupidLeader/pose', Pose, update_leader_pose)
    rospy.Subscriber('/scuGroupidFollowerA/in_position', Empty, followerA_in_position)
    rospy.Subscriber('/scuGroupidFollowerB/in_position', Empty, followerB_in_position)
    rate = rospy.Rate(10)

    # Spawn the leader turtle
    rospy.wait_for_service('spawn')
    spawn_turtle = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    clear_turtle = rospy.ServiceProxy('kill', turtlesim.srv.Kill)

        # 清除已有的乌龟
    try:
        clear_turtle('scuGroupidLeader')
    except rospy.ServiceException:
        pass  # 如果乌龟不存在，忽略错误

    spawn_turtle(5.5, 5.5, 0, 'scuGroupidLeader')

    initial_pose = leader_pose

    rospy.loginfo("Waiting for followers to get into position...")
    while not rospy.is_shutdown():
        if check_followers_in_position():
            rospy.loginfo("Followers in position. Leader starts moving.")
            while not rospy.is_shutdown():
                if at_boundary():
                    rospy.loginfo("Leader is at the boundary. Teleporting to initial position.")
                    pub.publish(Twist())  # stop the turtle
                    info_pub.publish(GroupidLeaderMessage(command="return"))
                    rospy.sleep(1)  # give followers time to return
                    teleport_to_initial_position()
                    rospy.loginfo("Leader back to initial position. Waiting for followers...")
		            rospy.sleep(100) 
                    break
                move_random(pub)
                rate.sleep()
        rate.sleep()

if __name__ == '__main__':
    try:
        leader()
    except rospy.ROSInterruptException:
        pass
