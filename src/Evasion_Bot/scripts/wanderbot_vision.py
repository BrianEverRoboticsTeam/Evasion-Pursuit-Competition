#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# cascade = cv2.CascadeClassifier('kobuki.xml')
# cascade = cv2.CascadeClassifier('kobuki_2.xml')
# cascade = cv2.CascadeClassifier('kobuki_lbp.xml')

frame = np.zeros([480, 640, 3], dtype=np.uint8)
br = CvBridge()

feature_params = dict(maxCorners = 500, qualityLevel = 0.15,
                      minDistance = 7, blockSize = 7)

num_parts_x = 8
num_parts_y = 8
height = 480
width = 640

msg = 0

def grab_image_callback(msg):
    global frame
    # global frame_gray
    tmp = br.imgmsg_to_cv2(msg)
    frame[..., 0] = tmp[..., 2]
    frame[..., 1] = tmp[..., 1]
    frame[..., 2] = tmp[..., 0]
    # frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

if __name__ == '__main__':
    rospy.init_node('color_and_feature_tracker_node')
    grab_image = rospy.Subscriber('camera/rgb/image_raw', Image, grab_image_callback)
    publisher = rospy.Publisher('color_and_feature_tracker', String, queue_size=1)
    #cascade = cv2.CascadeClassifier('kobuki_haar3.xml')
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if frame != None:
            # local sum list
            #local_sum = [[0 for i in range(num_parts_x)] for j in range(num_parts_y)]
            local_sum = np.zeros((num_parts_y,num_parts_x))
            #print(local_sum)
            #print("")

            # Capture frame-by-frame
            #ret, frame = cap.read()

            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray,(11,11),0)

            # Convert BGR to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # define range of blue color in HSV
            lower_blue = np.array([70,0,0])
            upper_blue = np.array([179,255,30])
            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(frame,frame, mask= mask)

            # Set threshold and maxValue
            thresh = 0
            maxValue = 255

            # Basic threshold example
            #th, dst = cv2.threshold(gray, thresh, maxValue, cv2.THRESH_BINARY);
            th3 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

            kp = cv2.goodFeaturesToTrack(th3, mask = mask, **feature_params)
            #print(kp[0])
            if kp!= None:
                for i in range(len(kp)):
                    cv2.circle(frame, (kp[i][0][0], kp[i][0][1]), 5, (0, 0, 255), -1)
                    #print(int(kp[i][0][0]//(width/num_parts_x)))
                    #print(int(kp[i][0][1]//(height/num_parts_y)))
                    x = int(kp[i][0][0]//(width/num_parts_x))
                    y = int(kp[i][0][1]//(height/num_parts_y))
                    local_sum[y][x] += 1
            main_data = np.array(local_sum[3:6,:])
            main_data_max = np.sum(main_data, axis=0)
            #print(main_data)
            print("\n")
            print(np.sum(main_data, axis=0))
            if np.max(main_data_max)>2:
                msg = np.argmax(main_data_max) - 3.5
                print(np.argmax(main_data_max) - 3.5)
            else:
                msg = "N/A"
                print("N/A")

            for i in range(num_parts_x):
                for j in range(num_parts_y):
                    cv2.circle(frame,
                            (i*(width/num_parts_x)+(width/num_parts_x)/2,
                            j*(height/num_parts_y)+(height/num_parts_y)/2),
                            int((height/num_parts_y)/3),
                            (0, local_sum[j][i]*10, 0), -1)

            # Display the resulting frame
            # cv2.imshow('mask',mask)
            # cv2.imshow('res',res)
            # cv2.imshow('th3',th3)
            # cv2.imshow('frame',frame)
            #
            # if cv2.waitKey(1000/24) & 0xFF == ord('q'):
            #     break

        publisher.publish(msg)
        rate.sleep()
