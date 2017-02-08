#!/usr/bin/env python

"""
The pursuit-bot node
"""

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ramp import Movement
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

br = CvBridge()
cascade = cv2.CascadeClassifier('kobuki_haar.xml')

frame = np.zeros([480, 640, 3], dtype=np.uint8)
depth_image = None
# frame_gray = None

show = True

# def equalize(img):
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    # img[:, :, 0] = cv2.equalizeHist(img[:, :, 0])
    # img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR)
    # return img


def grab_image_callback(msg):
    global frame
    # global frame_gray
    try:
        tmp = br.imgmsg_to_cv2(msg)
        frame[..., 0] = tmp[..., 2]
        frame[..., 1] = tmp[..., 1]
        frame[..., 2] = tmp[..., 0]
    except:
        frame = None
    # frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def grab_depth_image_callback(msg):
    global depth_image
    depth_image = br.imgmsg_to_cv2(msg)

if __name__ == '__main__':
    rospy.init_node('cascade_follower')
    grab_image = rospy.Subscriber('camera/rgb/image_raw', Image, grab_image_callback)
    grab_depth_image = rospy.Subscriber('/camera/depth/image', Image, grab_depth_image_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    # rate = rospy.Rate(10)
    prev_frame_gray = None
    corners = None
    mask = None
    prev_box = None
    box = None
    usebox = False
    dest_x = None
    dest_y = None
    prev_x = None
    prev_y = None
    move = Movement()
    tw = Twist()
    rate = rospy.Rate(10)
    decay = 0.15
    exist_last_tw = False
    # last_tw = Twist()
    locked_target = False


    feature_params = dict( maxCorners = 100,
            qualityLevel = 0.3,
            minDistance = 7,
            blockSize = 7 )

    lk_params = dict( winSize  = (15,15),
            maxLevel = 2,
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    while not rospy.is_shutdown():
        if frame != None:
            frame_bkp = frame.copy()
            # frame_in = equalize(frame)
            # curr_frame = frame.copy()
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            loc = cascade.detectMultiScale(frame, 1.3, 5)


            for box in loc:

                if prev_box != None and locked_target:
                    check_dist = 0.0
                    for i in range(4):
                        check_dist += (box[i] - prev_box[i]) ** 2
                    check_dist = math.sqrt(check_dist)
                    if check_dist > 50:
                        print 'ignore out of range target'
                        continue

                if prev_box != None:
                    for i in range(4):
                        # box[i] = int(round((prev_box[i] + box[i]) / 2.0))
                        box[i] = int(round(prev_box[i] * decay + box[i] * (1 - decay) ))

                x, y, w, h = box
                mid_x = x + w / 2
                mid_y = y + h / 2
                dest_x = mid_x
                dest_y = mid_y
                if mid_y < 100:
                    continue

                if show:
                    cv2.rectangle(frame_bkp, (x, y), (x+w, y+h), (255, 0, 0), 2)
                print 'found kubuki'

                mask = np.zeros_like(frame_gray, dtype=np.uint8)
                mask[y:y+h, x:x+w] = 1
                # mask[x:x+w, y:y+h] = 1

                prev_box = box
                usebox = True
                locked_target = True
                break

            if mask != None:
                corners = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                mask = None
                print 'new points generated'
                # if corners != None:
                    # corners = corners.astype(np.float32)
                    # corners = np.int0(corners)

            elif corners != None and len(corners.shape) == 3:
                if prev_frame_gray != None:
                    corners, st, err = cv2.calcOpticalFlowPyrLK(prev_frame_gray, frame_gray, corners, None, **lk_params)
                    # corners, st, err = cv2.calcOpticalFlowPyrLK(
                            # prev_frame_gray, frame_gray, corners, None,
                            # winSize=(15, 15),
                            # flags = cv2.OPTFLOW_USEINITIAL_FLOW,
                            # criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                                # 10, 0.03))

                    npt = np.sum(st==1)
                    print '%d points tracked'%(npt)
                    if npt == 0:
                        print 'loss all points'
                        corners = None
                    else:
                        corners = corners[st==1]
                        corners = corners[:, np.newaxis, :]

                        dest_x = corners[:, 0, 0].mean().astype(np.int32)
                        dest_y = corners[:, 0, 1].mean().astype(np.int32)


            if dest_x != None and dest_y != None and depth_image != None:
                if prev_x != None and prev_y != None:
                    dest_x = decay * prev_x + (1-decay) * dest_x
                    dest_y = decay * prev_y + (1-decay) * dest_y

                ###
                # Control motion:
                move.start()
                x_offset = dest_x - 320
                if abs(x_offset) > 10:
                    tw.angular.z = - x_offset * 0.01
                else:
                    tw.angular.z = 0

                # cmd_vel_pub.publish(tw)
                # print tw

                # Depth Image dist:
                # dist = np.nanmean(depth_image[y:y+h, x:x+w])
                dist = np.nanmean(depth_image[dest_y-5:dest_y+5, dest_x-5:dest_x+5])
                if math.isnan(dist):
                    dist = 0.8
                print 'sensor distance:', dist
                dist_offset = dist - 0.8

                # Camera dist:
                # print 'dest_y:', dest_y
                # dist_offset = (370 - dest_y) * 0.01
                print 'dist_error:', dist_offset


                # print 'dest:', dest_x, dest_y
                # print 'offset:', x_offset, dist_offset
                # if dist > 0:

                tw.linear.x = dist_offset * 1.5

                if tw.linear.x > 0.9:
                    tw.linear.x = 0.9
                print 'linear velocity:', tw.linear.x

                move.updateTarget(tw)
                cmd_vel_pub.publish(move.step())

                # last_tw.linear.x = tw.linear.x
                # last_tw.angular.z = tw.angular.z
                exist_last_tw = True

                if show:
                    ix = int(round(dest_x))
                    iy = int(round(dest_y))
                    cv2.circle(frame_bkp, (ix, iy), 5, (0, 0, 255), -1)

                prev_x = dest_x
                prev_y = dest_y
                dest_x = None
                dest_y = None

            else:
                print 'loss track'
                locked_target = False
                # if exist_last_tw or True:
                    # print '\tturn around'
                    # # print '\tpublish x:%d z:%d instead'%(last_tw.linear.x, last_tw.angular.z)
                    # if tw.angular.z > 0:
                        # tw.angular.z = 1
                    # elif tw.angular.z < 0:
                        # tw.angular.z = -1
                    # tw.linear.x = 0
                    # move.start()
                    # move.updateTarget(tw)
                    # cmd_vel_pub.publish(move.step())


            if show and corners != None:
                for i in corners.astype(np.int32):
                    cx, cy = i.ravel()
                    cv2.circle(frame_bkp, (cx, cy), 2, 255, -1)

            if show:
                cv2.imshow('frame', frame_bkp)

            prev_frame_gray = frame_gray


        if show:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        rate.sleep()
