#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math

def scan_callback(msg):
  global g_range_ahead_min, g_min_index
  g_range_ahead_min = min(msg.ranges)

  depths_ahead = []
  depths_all = []

  dist_idx = 0
  for dist in msg.ranges:
    if not np.isnan(dist):
      depths_all.append(dist)

      angle = (abs(dist_idx - 320) / 320.0 ) * 29
      #print("dist_idx:",dist_idx,"angle:",angle,"sin(angle):",math.sin(math.radians(angle)))
      dist_to_center_axis = dist * math.sin(math.radians(angle))
      if dist_to_center_axis < 0.28:
        depths_ahead.append(dist)
    dist_idx += 1

  if (len(depths_ahead)!=0):
    g_range_ahead_min = min(depths_ahead)

  if (len(depths_all)!=0):
    g_range_all_min = min(depths_all)

  try:
    g_min_index = msg.ranges.index(g_range_all_min)
  except:
    g_min_index = -1

  print("range:",g_range_ahead_min,"index:",g_min_index)

g_min_index = 320 # initial to midle point
g_last_twist = Twist() # initial to 0
g_range_ahead_min = 1 # anything to start
g_stop_distance = 1.2

driving_forward = True
start_turning = False
only_turning = False

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()

rate = rospy.Rate(10)

while not rospy.is_shutdown():
  # test = random.uniform(-1,1)
  # print(test)

  if driving_forward:
    # BEGIN FORWARD
    if (g_range_ahead_min < g_stop_distance or np.isnan(g_range_ahead_min)):# or rospy.Time.now() > state_change_time):

      driving_forward = False

    #   state_change_time = rospy.Time.now() + rospy.Duration(random.uniform(0.8,3.0))
    # END FORWARD
  else: # we're not driving_forward
    # BEGIN TURNING
    #if rospy.Time.now() > state_change_time:
    if g_range_ahead_min > g_stop_distance:
      driving_forward = True # we're done spinning, time to go forwards!
    #   state_change_time = rospy.Time.now() + rospy.Duration(5)

  if start_turning:
    if (g_range_ahead_min < 0.7 or np.isnan(g_range_ahead_min)):
      only_turning = True
  else:
    if g_range_ahead_min > 0.7:
      only_turning = False
      start_turning = False

  if driving_forward:
    g_last_twist.linear.x = 0.7
    g_last_twist.angular.z = 0
    start_turning = False
    only_turning = False
  elif not start_turning:
    g_last_twist.linear.x = 0.45
    start_turning = True
    # if(not np.isnan(g_range_ahead_min)):
    #   print("range:",g_range_ahead_min,"index:",g_min_index)

    if g_min_index > 320:
      g_last_twist.angular.z = -2
    elif g_min_index>=0:
      g_last_twist.angular.z = 2

    #g_last_twist = twist

  elif only_turning:
    g_last_twist.linear.x = 0
    if g_last_twist.angular.z == 0:
        g_last_twist.angular.z = random.uniform(-3.14, 3.14)

  cmd_vel_pub.publish(g_last_twist)

  rate.sleep()
# END ALL
