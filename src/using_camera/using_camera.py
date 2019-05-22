#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

def isYellow(pixel):
    if pixel[1] > 200 and pixel[0] < 50:
        return True
    else:
        return False

def isWhite(pixel):
    if pixel[1] > 200 and pixel[0] > 50:
        return True
    else:
        return False

def findYellowLane(image):
    exist = False
    point1 = [-1,-1]
    point2 = [-1,-1]
    for i in range(0, 50):
        for j in range(0, 100):
            row = 599 - i * 10
            col = 499 - j * 5
            if isYellow(image[row][col][:]):
                point1[0] = row
                point1[1] = col
                index = i
                break
        if point1[0] != -1:
            break
    
    if point1[0] == -1:
        return exist, point1, point2


    for i in range(index + 10, 60):
        for j in range(0, 100):
            row = 599 - i * 10
            col = 499 - j * 5
            if isYellow(image[row][col][:]):
                point2[0] = row
                point2[1] = col
                exist = True
                break
        if point2[0] != -1:
            break

    return exist, point1, point2

def findWhiteLane(image):
    exist = False
    point1 = [-1, -1]
    point2 = [-1, -1]
    for i in range(0, 50):
        for j in range(0, 100):
            row = 599 - i * 10
            col = 499 + j * 5
            if isWhite(image[row][col][:]):
                point1[0] = row
                point1[1] = col
                index = i
                break
        if point1[0] != -1:
            break
    
    if point1[0] == -1:
        return exist, point1, point2


    for i in range(index + 10, 60):
        for j in range(0, 100):
            row = 599 - i * 10
            col = 499 + j * 5
            if isWhite(image[row][col][:]):
                point2[0] = row
                point2[1] = col
                exist = True
                break
        if point2[0] != -1:
            break

    return exist, point1, point2

def callback(x):
    pass

def PIDcontroll(l_exist, l_point1, l_point2, r_exist, r_point1, r_point2, trackingLane):
    l_point1[0] = 599 - l_point1[0]
    l_point2[0] = 599 - l_point2[0]
    r_point1[0] = 599 - r_point1[0]
    r_point2[0] = 599 - r_point2[0]

    # decide which lane to follow
    # if there is only one lane, follow that lane
    # else if there are two lanes and one lane decline more then some value, follow that lane
    # else follow previous follow lane
    if l_exist == True and r_exist == False:
        trackingLane = "left"
    elif l_exist == False and r_exist == True:
        trackingLane = "right"
    elif l_exist == True and r_exist == True:
        if l_point1[1] == l_point2[1]:
            l_point1[1] += 1
        if r_point1[1] == r_point2[1]:
            r_point1[1] += 1
        
        l_angle = (l_point1[0] - l_point2[0]) / (l_point1[1] - l_point2[1])
        r_angle = (r_point1[0] - r_point2[0]) / (r_point1[1] - r_point2[1])

        if l_angle < 3 and l_angle > 0:
            trackingLane = "left"
        elif r_angle > -3 and r_angle < 0:
            trackingLane = "right"
    #print trackingLane
    if trackingLane == "left":
        point1 = l_point1
        point2 = l_point2
    elif trackingLane == "right":
        point1 = r_point1
        point2 = r_point2

    # calculating the distance between lane and vehicle
    # then by using the distance, we can calculate desired_direction between lane and vehicle
    # lastly by using this angle, calculate desired_wheel_angle
    # in this process we use PID controll 2 times and I'll use only P controll to be simple
    
    # 1. distance
    point_robot = [-300, 500]
    if point1[1] == point2[1]: # in case of vertical to avoid INF angle
        point1[1] += 1

    # a*x + by + c = 0
    a = (point1[0] - point2[0]) / (point1[1] - point2[1])
    if a == 0:
        a = 0.000001
    b = -1
    c = -a * point1[1] + point1[0]

    distance = abs(a * point_robot[1] + b * point_robot[0] + c) / math.sqrt(a * a + b * b)

    # 2. desired_direction
    if trackingLane == "left":
        desired_direction = (distance - 450) * 0.25
    elif trackingLane == "right":
        desired_direction = -(distance - 450) * 0.25

    # 3. wheel_angle
    theta_current = -math.atan(1/(a * np.pi / 180))
    if theta_current < 0:
        theta_current = -(np.pi/2 + theta_current)
    else:
        theta_current = np.pi/2 - theta_current
    theta_current = theta_current * 180 / np.pi
    wheel_angle = (desired_direction + theta_current)  * 0.01

    print "a : ", a 
    print trackingLane, "distance : ", distance, "  desired_direction : ", desired_direction, "  thera_current : ", theta_current,   "    wheel_angle : ", wheel_angle
    return trackingLane, wheel_angle
class Driving():
    def __init__(self):

        self.trackbar = "off" # you can choose showing trackbar or not by "on", "off"
        self.showing_images = "on" # you can choose showing images or not by "on", "off"
        self._sub = rospy.Subscriber('/camera/image', Image, self.callback, queue_size=1)
        self._cv_bridge = CvBridge()
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty) # reset service
        self.end = False
        self.trackingLane = "right"
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.speed = 1
        self.reset_proxy()

    def callback(self, image_msg):

        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # adding Gaussian blur
        #cv_image = cv2.GaussianBlur(cv_image,(5,5),0)

        # copy original image to use in homography process
        cv_origin = np.copy(cv_image)

        # setting homography variables
        Top_w = 81
        Top_h = 0
        Bottom_w = 160
        Bottom_h = 100
        binary_threshold = 40


        if self.trackbar == "on":
            # creating trackbar
            cv2.namedWindow('trackbar')
            cv2.moveWindow('trackbar', 40,30)
            cv2.createTrackbar('Top_w','trackbar',Top_w, 120,callback)
            cv2.createTrackbar('Top_h','trackbar',Top_h, 120,callback)
            cv2.createTrackbar('Bottom_w','trackbar',Bottom_w, 160,callback)
            cv2.createTrackbar('Bottom_h','trackbar',Bottom_h, 160,callback)
            cv2.createTrackbar('binary_threshold', 'trackbar', binary_threshold, 255, callback)

            # getting homography variables from trackbar
            Top_w = cv2.getTrackbarPos('Top_w', 'trackbar')
            Top_h = cv2.getTrackbarPos('Top_h', 'trackbar')
            Bottom_w = cv2.getTrackbarPos('Bottom_w', 'trackbar')
            Bottom_h = cv2.getTrackbarPos('Bottom_h', 'trackbar')
            binary_threshold = cv2.getTrackbarPos('binary_threshold', 'trackbar')
            cv2.moveWindow('trackbar', 40,30)
        if self.showing_images == "on" and self.trackbar == 'on':
            # draw lines to help setting homography variables
            cv_image = cv2.line(cv_image, (160-Top_w, 120-Top_h), (160+Top_w, 120-Top_h), (0, 0, 255), 1)
            cv_image = cv2.line(cv_image, (160-Bottom_w, 120+Bottom_h), (160+Bottom_w, 120+Bottom_h), (0, 0, 255), 1)
            cv_image = cv2.line(cv_image, (160+Bottom_w, 120+Bottom_h), (160+Top_w, 120-Top_h), (0, 0, 255), 1)
            cv_image = cv2.line(cv_image, (160-Bottom_w, 120+Bottom_h), (160-Top_w, 120-Top_h), (0, 0, 255), 1)
            cv2.imshow('trackbar', cv_image), cv2.waitKey(1)
        ### homography transform process ###
        # selecting 4 points from the original image
        pts_src = np.array([[160-Top_w, 120-Top_h], [160+Top_w, 120-Top_h], [160+Bottom_w, 120+Bottom_h], [160-Bottom_w, 120+Bottom_h]])
        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])
        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)
        # homography process
        cv_Homography = cv2.warpPerspective(cv_origin, h, (1000, 600))
        
        # find and draw yellow lane
        Y_exist, Y_point1, Y_point2 = findYellowLane(cv_Homography)
        W_exist, W_point1, W_point2 = findWhiteLane(cv_Homography)
        if Y_exist:
            cv_Homography = cv2.line(cv_Homography, (Y_point1[1], Y_point1[0]), (Y_point2[1], Y_point2[0]), (255, 255, 0), 5)
        if W_exist:
            cv_Homography = cv2.line(cv_Homography, (W_point1[1], W_point1[0]), (W_point2[1], W_point2[0]), (0, 0, 255), 5)

        if Y_exist == False and W_exist == False:
            if self.trackingLane == "right":
                wheel_angle = -0.2
            elif self.trackingLane == "left":
                wheel_angle = 0.2
        else:
            self.trackingLane, wheel_angle = PIDcontroll(Y_exist, Y_point1, Y_point2, W_exist, W_point1, W_point2, self.trackingLane)
        
        vel = Twist()
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = wheel_angle * self.speed
        vel.linear.x = 0.1 * self.speed
        vel.linear.y = 0
        vel.linear.z = 0
        self._pub.publish(vel)

        # showing calibrated iamge and Bird's eye view image
        if self.showing_images == "on":
            
            cv2.namedWindow('Homography')
            cv2.moveWindow('Homography', 600,30)
            cv2.imshow('Homography', cv_Homography), cv2.waitKey(1)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('driving')
    node = Driving()
    node.main()
