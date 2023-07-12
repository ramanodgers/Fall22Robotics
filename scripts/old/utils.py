#!/usr/bin/env python3


from distutils.log import ERROR
import rospy, cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import numpy as np
import path_finder

COLOR_BOUNDS = {"pink": np.array([[130, 100, 100], [170, 255, 255]]),
                "green": np.array([[40, 100, 100], [75, 255, 255]]),
                "blue": np.array([[80, 100, 100], [110, 255, 255]])}

class Utils():
    def __init__(self, freq):
        # initial the node, command format, and publisher
        self.scan_res = np.array([0] * 360) # ranges from a scan is stored here
        self.move_cmd = Twist() # Twist command, avoiding re-initializing the object
        self.rate = rospy.Rate(freq) # Rate of publishing control commands
        self.freq = freq # Frequency (in Hz) of publishing control commands
        self.closest_dir = 0 # Direction (in deg) of closest object
        self.closest_dist = 0 # Distance to the closest object
        self.threshold = 5000 # The number of minimal pixels to find a color
        self.image = None # The most recent image feed from the camera

        # subscribe to scan messages and store the results in self.scan_res
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.record)

        # publish command messages for the turtlebot (itself, not the arm) to move
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

    # stop any motion, publish multiple times to make sure
    def stop(self):
        for i in range(10):
            self.move(0,0)
            self.rate.sleep()

    # publish control command
    def move(self, trans, rot):
            print("movevee")
            self.move_cmd.angular.z = rot
            self.move_cmd.linear.x = trans
            self.cmd_publisher.publish(self.move_cmd)

    # call back function of subscriber
    # store the scan result of ranges in scan_res
    def record(self, data):
        self.scan_res = np.array(data.ranges)

    def get_closest_dir(self):
        res = np.array(self.scan_res)
        res[res < 0.1] = 10
        # print error when dimension of scan result is not 360
        if res.size != 360:
            print('ERROR: scan range size not 360')
            return
        print(np.argmin(res))
        self.closest_dir = np.argmin(res)

    # return the distance to the closest obstacle detected
    # by the lidar
    def get_closest_dist(self):
        res = np.array(self.scan_res)
        res[res < 0.1] = 10
        if res.size != 360:
            print('ERROR: scan range size not 360')
            return -1
        self.closest_dist = min(res)

    # return the closest distance in the direction
    # from range to 360 - range
    def get_closest_infront(self, range):
        res = np.array(self.scan_res)
        res = np.concatenate((res[-range:-1], res[0: range]))
        res[res < 0.1] = 10
        return min(res)

    # converts the incoming ROS message and save it to self.image
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    # Given self.image, get the pixel coordinate of the center of the color on the image
    def get_color_center(self, color):

        # When not image is received
        if self.image is None:
            print("Image Not Received")
            return 0

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # get the upper and the lower bound of the color in self.color_looking_for
        bounds = COLOR_BOUNDS[color]

        lower_bound = bounds[0]
        upper_bound = bounds[1]

        # this erases all pixels that aren't the given color
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)

        print("The total number of pixels are {}".format(M['m00']))

        # pixels of the given color found
        if M['m00'] > self.threshold:
            # center of the yellow pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # a red circle is visualized in the debugging window to indicate
            # the center point of the yellow pixels
            # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
            cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)

            # save the result to a self.color_center
            self.color_center = cx

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        cv2.imshow("window", self.image)
        cv2.waitKey(3)

        # not enough pixels of the given color found
        if not M['m00'] > self.threshold:
            return 0
        # enough pixels of the given color found
        else:
            return 1


    # Return the difference (in deg) between two directions
    # The result is how many degrees is the current direction away from the target
    # negative if current direction is in the left of the target direction
    def deg_diff(self, target, curr):
        # print error when current degree is not between 0 and 359
        if not 0 <= curr <= 359:
            print('ERROR: the degree must be between 0 and 359')
            return 360
        # When target is in the right but the number (from 0 to 359) is larger
        elif target - curr > 180:
            err = curr + 360 - target
        # when target is in the left but the number (from 0 to 359) is smaller
        elif curr - target > 180:
            err = curr - (target + 360)
        # normal case
        else:
            err = curr - target
        return err

    # return the cropped proportional error between target and current direction
    # the returned value is a float between -1 and 1
    def err_proportional_dir(self, target, curr, range):
        err = self.deg_diff(target, curr)
        # error case of deg_diff
        if err == 360:
            return 360
        # need to crop the error
        if abs(err) > range:
            # when err / range > 1, return 1
            if err > 0:
                return 1
            # when err / range < -1, return -1
            else:
                return -1
        # normal case
        else:
            return err / range

    # return the cropped proportional error between target and current distance
    # the returned value is a float between -0.1 and 0.1
    def err_proportional_dist(self, target, curr, range):
        err = curr - target
        # need to crop the error
        if abs(err) > range:
            # when err / range > 0.1, return 0.1
            if err > 0:
                return 0.1
            # when err / range < -0.1, return -0.1
            else:
                return -0.1
        # normal case
        else:
            return err * 0.1 / range
