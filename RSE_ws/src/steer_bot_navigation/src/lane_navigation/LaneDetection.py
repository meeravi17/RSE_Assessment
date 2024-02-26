#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import ros
import math

class LaneController:
    def __init__(self):
        pass

    def average_slope_intercept(self, image, lines):
        left_lanes = []
        right_lanes = []
        
        boundary = 1 / 3
        left_region_boundary = image.shape[1] * (1 - boundary)
        right_region_boundary = image.shape[1] * boundary
        
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
    
            if x1 == x2:
                print('[Lane Detection] Rejecting straight lines')
                continue
    
            slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)
            
            if abs(slope) < 0.05:
                print('[Lane Detection] Rejecting Horizontal lines')
                continue
    
            print("slope", slope)
    
            if slope < 0 and x1 < left_region_boundary and x2 < left_region_boundary:
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red color for left lines
                left_lanes.append((slope, intercept))
                print(len(left_lanes))
    
            elif slope > 0 and x1 > right_region_boundary and x2 > right_region_boundary:
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green color for right lines
                right_lanes.append((slope, intercept))
    
        result = []
        if left_lanes:
            left_line_params_avg = np.average(left_lanes, axis=0)
            left_line = self.make_coordinates(image, left_line_params_avg)
            result.append(left_line)
    
        if right_lanes:
            right_line_params_avg = np.average(right_lanes, axis=0)
            right_line = self.make_coordinates(image, right_line_params_avg)
            result.append(right_line)
    
        return image, np.asarray(result)

    def make_coordinates(self, image , line_parameteres):
        slope , intercept = line_parameteres
        width = image.shape[1]
        y1 = image.shape[0]
        y2 = int(y1*1/2)
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

        return np.array([x1 , y1 , x2 , y2])

    def compute_steering_angles(self, image , lane_lines):
        if len(lane_lines)==0:
            print("[Lane Detection] No lane lines detected")
            return -90

        height , width , _ = image.shape
        #print(lane_lines)
        if len(lane_lines) == 1:
            x1 , y1 , x2 , y2 = lane_lines[0].reshape(4)
            xoffset = x2 - x1
        else:
            lx1 , ly1 , lx2 , ly2 = lane_lines[0].reshape(4)
            rx1 , ry1 , rx2 , ry2 = lane_lines[1].reshape(4)
            mid = int(width/2)
            xoffset = int(((lx2 + rx2)/2) - mid)

        yoffset = int(height/2)
        if (xoffset) == 0:
            print(0)
            return 0
        angle_to_mid_radian = math.atan(yoffset/xoffset)
        angle_to_mid_degree = int(angle_to_mid_radian*180/math.pi)
        if(angle_to_mid_degree > 0 ):
            steering_angle = angle_to_mid_degree - 90
        else:
            steering_angle = angle_to_mid_degree + 90
        print("[Lane Detection] steering angle", steering_angle)

        return steering_angle

    def show_steering_corection(self, image , error):
        correction_image = np.zeros_like(image)
        error_radian = (error*math.pi)/180
        diff_in_x = 300*math.tan(error_radian)
        correct_x_coord = int(600-diff_in_x)
        cv2.line(correction_image , (600 , correction_image.shape[0]) , (correct_x_coord , correction_image.shape[0]-300) , (0 , 0 , 255) , 5)

        return correction_image


class LaneDetectors:
    def __init__(self, image_topic, lane_controller):
        self.image_topic = image_topic
        self.lane_controller = lane_controller
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=1)
        self.kP = 0.0001
        self.kD = 0.00001
        self.kI = 0.000000
        self.last_error =0
        self.integralActivatingZone = 0
        self.prop = 0
        self.diff = 0
        self.integral = 0
        self.pub = 0
        self.tot_error = 0

        # Create a mask (e.g., a triangle)
        self.mask = np.zeros((480, 640), dtype=np.uint8)
        height, width = self.mask.shape
        polygon = np.array([[0, 280], [100, 210], [550, 220], [width, 270]], dtype=np.int32)
        cv2.fillPoly(self.mask, [polygon], 255)
        
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def give_steering_angle(self, error):
        if( error < self.integralActivatingZone and error != 0):
            self.tot_error += error
        else:
            self.tot_error = 0

        if error == 0:
            self.diff = 0
            self.prop = 0

        self.prop += error*self.kP
        self.diff += (error - self.last_error)*self.kD
        self.integral += self.tot_error *self.kI

        self.last_error = error

        steering_val_to_motors = self.prop + self.diff + self.integral
        self.publish_velocity(0.3, steering_val_to_motors)
   

    def detect_and_draw_lines(self, image):
        rospy.loginfo("[Lane Navigation] Starting Navigation ...")
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([10, 100, 20])  
        upper_green = np.array([80, 255, 255])

        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        green_mask = cv2.bitwise_and(green_mask, self.mask)

        green_regions = cv2.bitwise_and(image, image, mask=green_mask)
        green_gray = cv2.cvtColor(green_regions, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(green_gray, 50, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)

        result_image = image.copy()
        left_lane = ()
        combo_image = image.copy()

        if lines is not None:
            result_image, lane_sum_average = self.lane_controller.average_slope_intercept(image, lines)
            combo_image = cv2.addWeighted(image , 0.8 , result_image , 1 , 1)
            if len(lane_sum_average) > 0:
                steering_error = self.lane_controller.compute_steering_angles(image , lane_sum_average)
                correction_image = self.lane_controller.show_steering_corection(combo_image , steering_error)
                combo_image = cv2.addWeighted(combo_image , 0.9 , correction_image , 1 , 1)
                self.give_steering_angle(steering_error)

        return combo_image #, left_lane

    def image_callback(self, msg):
        try:
            left_lane = ()
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = self.detect_and_draw_lines(cv_image)
            cv2.line(img , (600 , cv_image.shape[0]) , (600 , cv_image.shape[0]-300) , (0 , 255 , 0) , 5 )

            cv2.imshow("Lane Detection", img)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(e)

    def publish_velocity(self, linear_velocity, steering_angle):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = steering_angle
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(0.005)


def main():
    rospy.init_node('green_lines_detector', anonymous=True)
    image_topic = "/steer_bot/camera/color/image_raw" 

    controller = LaneController()
    detector = LaneDetectors(image_topic, lane_controller=controller)


    rospy.spin()

if __name__ == '__main__':
    main()
