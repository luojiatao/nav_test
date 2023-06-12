#!/usr/bin/env python

#可以直接用rosrun teleop_twist_keyboard teleop_twist_keyboard.py来使用键盘控制运动，由于后期导航需要用到move_base做路径规划，而move_base发布的cmd_vel话题是geometry_msgs.msg/Twist消息类型，keyboard_teleop.py里使用TK框显示的键盘控制用的是ackermann_msgs/AckermannDriveStamped 消息类型控制小车运动，因此要做一个消息类型的转换，该文件就是转换的代码文件。

import rospy
import std_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

import time 
import threading
pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)

def thread_job():
    rospy.spin()

def callback(data):
    speed = data.linear.x 
    turn = data.angular.z

    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link";

    msg.drive.speed = speed;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = turn
    msg.drive.steering_angle_velocity = 1

    pub.publish(msg)

def SubscribeAndPublish():
    rospy.init_node('nav_sim', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    rospy.spin()


if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass


