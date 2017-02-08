#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

g_is_dead_bot = True
g_twist = Twist() # default to 0

def dead_bot_switch(msg):
    global g_is_dead_bot

    if msg.buttons[2]==1:
        g_is_dead_bot = (g_is_dead_bot==False)

    print(g_is_dead_bot)

def got_twist(msg):
    global g_twist
    g_twist = msg

if __name__ == '__main__':
    joy_sub = rospy.Subscriber('joy', Joy, dead_bot_switch)
    cmd_vel_with_dead_bot_switch = rospy.Subscriber('cmd_vel_safe', Twist, got_twist)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('dead_bot_switch')

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if(g_is_dead_bot):
            cmd_vel_pub.publish(Twist()) # stop the robot
        else:
            cmd_vel_pub.publish(g_twist)
        rate.sleep()
