#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
import math

class MatchPercentage(object):
    def __init__(self):
        rospy.init_node('map_laser_filter')
        self.scan_pub = rospy.Publisher('/map_laser_match_scan',
                                   LaserScan,
                                   queue_size=10)
        self.percentage_pub = rospy.Publisher('/scan_percentage', 
				 				   Float32,
				   				   queue_size=10)
        self.listener = tf.TransformListener()
        self.map = None
        self.trans = None
        self.rot = None
        self.nieghbours=2
        self.laser_scans=LaserScan()
        self.scan_percentage=0.0
        self.laser_sub = rospy.Subscriber('/steer_bot/scan',
                                    LaserScan,
                                    self.laser_cb,
                                    queue_size=1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)

    # Map callback
    def map_cb(self, msg):
        self.map = msg

    # Method that returns the tf between map_frame and laser_frame
    def get_laser_frame(self, msg):
        now = msg.header.stamp
        map_frame = '/map'
        laser_frame = msg.header.frame_id
        self.listener.waitForTransform(map_frame, laser_frame, now, rospy.Duration(.15))
        return self.listener.lookupTransform(map_frame, laser_frame, now)

    # This method sees if an x-y coordinate is occupied
    def is_occupied(self, x, y):
        N = self.nieghbours
        for dx in range(-N, N + 1):
            for dy in range(-N, N + 1):
                index = (x + dx) + (y + dy) * self.map.info.width
                if index < 0 or index > len(self.map.data):
                    continue
                value = self.map.data[index]
                if value > 50:
                    return True
        return False

    # Laser callback
    def laser_cb(self, msg):
        self.laser_scans = msg

    # Publish data
    def publish_data(self):
        self.scan_percentage = 0.0
        matched_beams = 0.0
        total_beams = 0.0
        # Try to get map to laser tf
        try:
            (self.trans, self.rot) = self.get_laser_frame(self.laser_scans)
        except tf.Exception:
            self.percentage_pub.publish(self.scan_percentage)
            return

        if self.map is None:
            return

        yaw = euler_from_quaternion(self.rot)[2]
        nr = []

        for (i, d) in enumerate(self.laser_scans.ranges):
            if math.isnan(d) or d > self.laser_scans.range_max or d < self.laser_scans.range_min:
                nr.append(self.laser_scans.range_max + 1.0)
                continue

            # Convert to cartesian
            angle = yaw + self.laser_scans.angle_min + self.laser_scans.angle_increment * i
            dx = math.cos(angle) * d
            dy = math.sin(angle) * d
            # print("angle", dx, dy)
            map_x = self.trans[0] + dx
            map_y = self.trans[1] + dy
            
            if math.isinf(map_x): 
                map_x = 70.0
            if math.isinf(map_y):
                map_y = 70.0 

            grid_x = int((map_x - self.map.info.origin.position.x) / self.map.info.resolution)
            grid_y = int((map_y - self.map.info.origin.position.y) / self.map.info.resolution)

            # This means this beam is on the map
            if self.is_occupied(grid_x, grid_y):
                nr.append(self.laser_scans.range_max + 1.0)
                matched_beams = matched_beams + 1
            else:
                nr.append(d)

            total_beams = total_beams + 1

        # Convert to percentage
        self.scan_percentage = (matched_beams / total_beams) * 100
        self.laser_scans.ranges = nr
        self.percentage_pub.publish(self.scan_percentage)
        self.scan_pub.publish(self.laser_scans)

if __name__ == '__main__':
    match_percentage = MatchPercentage()
    match_percentage.publish_data()
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        match_percentage.publish_data()
        r.sleep()
