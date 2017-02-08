"""
A library for ramp motion.
"""
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def getTimeSafe():
    while True:
        # rospy may returns zero, so we loop until get a non-zero value.
        time = rospy.Time.now()
        if time != rospy.Time(0):
            return time


class Movement:
    def __init__(self):
        self.last = Twist()
        self.target = Twist()
        self.acce_linear = 0.3
        self.acce_angular = 2.0
        self.t_last = None
        self.t_now = None
        self.suspend = True

    # def target_x(self, x):
        # self.target.linear.x = x

    # def target_z(self, z):
        # self.target.angular.z = z

    def updateTarget(self, tw):
        self.target = tw

    def stopped(self):
        print self.last.linear.x, self.last.linear.z
        return self.last.linear.x == 0 and self.last.angular.z == 0

    def detach(self):
        self.suspend = True

    def start(self):
        # Run this everytime you start a movement, after a short pause, you
        # need to start again, otherwise the robot will encounter a sudden
        # move or sudden drop in speed.
        if self.suspend:
            self.t_last = getTimeSafe()
            self.suspend = False

        # while True:
            # # rospy may returns zero, so we loop until get a non-zero value.
            # self.t_last = rospy.Time.now()
            # if self.t_last != rospy.Time(0):
                # break

    def step(self):
        if not self.suspend:
            # self.t_now = rospy.Time.now()
            self.t_now = getTimeSafe()
            # print '\tnow=', self.t_now
            self.ramped_twist()
            self.t_last = self.t_now
        return self.last

    def ramped_vel(self, v, v_target, t, t_now, acceleration):
        # print '\t\t', t, t_now
        step = acceleration * (t_now - t).to_sec()
        if v_target > v:
            sign = 1.0
        else:
            sign = -1.0
        if math.fabs(v_target - v) < step:
            return v_target
        else:
            return v + step * sign

    def ramped_twist(self):
        self.last.angular.z = self.ramped_vel(
                self.last.angular.z, self.target.angular.z,
                self.t_last, self.t_now,
                self.acce_angular)
        self.last.linear.x = self.ramped_vel(
                self.last.linear.x, self.target.linear.x,
                self.t_last, self.t_now,
                self.acce_linear)

    def force_stop(self):
        self.last.angular.z = 0
        self.last.linear.x = 0
        self.detach()
