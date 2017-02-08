#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import time


def scan_callback(msg):
    global g_range_ahead, g_min_index
    g_range_ahead = min(msg.ranges)
    # range_ahead = msg.ranges[len(msg.ranges)/2]
    # print(msg.ranges)
    # print "range ahead: %0.1f" % range_ahead
    # g_range_ahead = range_ahead
    # print(type(g_range_ahead))

    depths = []
    for dist in msg.ranges:
        if not np.isnan(dist):
            depths.append(dist)

    if (len(depths) != 0):
        g_range_ahead = min(depths)
    # else:
    #   g_range_ahead = 0.5

    try:
        g_min_index = msg.ranges.index(g_range_ahead)
    except:
        g_min_index = 0
    # if(not np.isnan(g_range_ahead)):
    #   print("range:",g_range_ahead,"index:",g_min_index)

g_min_index = 320  # initial to midle point
g_last_twist = Twist()  # initial to 0
g_range_ahead = 1  # anything to start
g_turn_distance = 0.8
g_stop_distance = 0.55
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
start_turning = False
rate = rospy.Rate(10)
start_time = -1
end_time = -1
dead_end = False

while not rospy.is_shutdown():
    end_time = time.time()
    if start_time != -1:
        stop_time = end_time - start_time
        print (str(stop_time))
        if stop_time > 5:
            print ('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            state_change_time = rospy.Time.now() + rospy.Duration(random.randint(3,5))
            print (state_change_time)
            dead_end = True
            start_time = time.time()
            try:
                cmd_vel_pub.publish(twist)
                rate.sleep()
                print ('done!!!')
            except:
                print('can not make it')
            continue

    #print("range:", g_range_ahead, "index:", g_min_index)
    # test = random.uniform(-1,1)
    # print(test)

    if (rospy.Time.now() >= state_change_time):
        dead_end = False
        driving_forward = True
        start_turning = False
        if driving_forward:
        # BEGIN FORWARD
            if (g_range_ahead < g_turn_distance and g_range_ahead > g_stop_distance):
                start_turning = True
        # or rospy.Time.now() > state_change_time):
            if (g_range_ahead < g_stop_distance or np.isnan(g_range_ahead)):
                driving_forward = False
                start_turning = True

        #   state_change_time = rospy.Time.now() + rospy.Duration(random.uniform(0.8,3.0))
        # END FORWARD
        if not driving_forward:  # we're not driving_forward
        # BEGIN TURNING
        # if rospy.Time.now() > state_change_time:
            if (g_range_ahead > g_stop_distance):
                driving_forward = True  # we're done spinning, time to go forwards!
        #   state_change_time = rospy.Time.now() + rospy.Duration(5)

        # END TURNING
    twist = Twist()
    if dead_end == True:
        twist.angular.z = 1
        start_time = time.time()
    else:
        if (driving_forward) and (not start_turning):
            twist.linear.x = 3
            start_time = time.time()

        elif (start_turning) and (not driving_forward):
        # if(not np.isnan(g_range_ahead)):

            if g_min_index > 320:
                twist.angular.z = -0.5
            else:
                twist.angular.z = 0.5

        elif (driving_forward) & (start_turning):
            twist.linear.x = 0.2
            start_time = time.time()
            if g_min_index > 320:
                twist.angular.z = -0.8
            else:
                twist.angular.z = 0.8

    cmd_vel_pub.publish(twist)

    rate.sleep()
# END ALL
