#!/usr/bin/env python

import rospy
import numpy as np

import time
from threading import Lock
import tf
from std_msgs.msg import Header, String, Float32MultiArray
from visualization_msgs.msg import Marker
import tf.transformations

# messages
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

# visualization packages
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import range_libc
import scipy.stats as stats

VAR_NO_EVAL_SENSOR_MODEL = 0
VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3
VAR_RADIAL_CDDT_OPTIMIZATIONS = 4


class CircularArray(object):

    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])


class Timer:

    def __init__(self, smoothing):
        self.arr = CircularArray(smoothing)
        self.last_time = time.time()

    def tick(self):
        t = time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    def fps(self):
        return self.arr.mean()


class ParticleFiler():

    def __init__(self):
        # parameters
        self.ANGLE_STEP = int(rospy.get_param("~angle_step"))
        self.MAX_PARTICLES = int(rospy.get_param("~max_particles"))
        self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles"))
        self.INV_SQUASH_FACTOR = 1.0 / float(rospy.get_param("~squash_factor"))
        self.MAX_RANGE_METERS = float(rospy.get_param("~max_range"))
        self.THETA_DISCRETIZATION = int(
            rospy.get_param("~theta_discretization"))
        self.WHICH_RM = rospy.get_param("~range_method", "cddt").lower()
        self.RANGELIB_VAR = int(rospy.get_param("~rangelib_variant", "3"))
        self.SHOW_FINE_TIMING = bool(rospy.get_param("~fine_timing", "0"))
        self.PUBLISH_ODOM = bool(rospy.get_param("~publish_odom", "1"))
        self.DO_VIZ = bool(rospy.get_param("~viz"))
        self.LOG_TAG = "[MCL - Offline]"

        # sensor model constants
        self.Z_SHORT = float(rospy.get_param("~z_short", 0.01))
        self.Z_MAX = float(rospy.get_param("~z_max", 0.07))
        self.Z_RAND = float(rospy.get_param("~z_rand", 0.12))
        self.Z_HIT = float(rospy.get_param("~z_hit", 0.75))
        self.SIGMA_HIT = float(rospy.get_param("~sigma_hit", 8.0))

        # motion model constants
        self.MOTION_DISPERSION_X = float(
            rospy.get_param("~motion_dispersion_x", 0.05))
        self.MOTION_DISPERSION_Y = float(
            rospy.get_param("~motion_dispersion_y", 0.025))
        self.MOTION_DISPERSION_THETA = float(
            rospy.get_param("~motion_dispersion_theta", 0.25))

        self.MAX_RANGE_PX = None
        self.odometry_data = np.array([0.0, 0.0, 0.0])
        self.laser = None
        self.iters = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.last_pose = None
        self.laser_angles = None
        self.downsampled_angles = None
        self.range_method = None
        self.last_time = None
        self.last_stamp = None
        self.first_sensor_update = True
        self.state_lock = Lock()

        self.local_deltas = np.zeros((self.MAX_PARTICLES, 3))

        self.queries = None
        self.ranges = None
        self.tiled_angles = None
        self.sensor_model_table = None

        self.inferred_pose = None
        self.particle_indices = np.arange(self.MAX_PARTICLES)
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        self.smoothing = CircularArray(10)
        self.timer = Timer(10)
        self.get_omap()
        self.precompute_sensor_model()
        self.initialize_global()

        # these topics are for visualization
        self.pose_pub = rospy.Publisher("/pf/viz/inferred_pose",
                                        PoseStamped,
                                        queue_size=1)
        self.particle_pub = rospy.Publisher("/pf/viz/particles",
                                            PoseArray,
                                            queue_size=1)
        self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan",
                                             LaserScan,
                                             queue_size=1)
        self.rect_pub = rospy.Publisher("/pf/viz/poly1",
                                        PolygonStamped,
                                        queue_size=1)

        if self.PUBLISH_ODOM:
            self.odom_pub = rospy.Publisher("/pf/pose/odom",
                                            Odometry,
                                            queue_size=1)

        # these topics are for coordinate space things
        self.pub_tf = tf.TransformBroadcaster()

        # these topics are to receive data from the racecar
        self.laser_sub = rospy.Subscriber(rospy.get_param(
            "~scan_topic", "/steer_bot/scan"),
                                          LaserScan,
                                          self.lidarCB,
                                          queue_size=1)
        self.odom_sub = rospy.Subscriber(rospy.get_param(
            "~odometry_topic",
            "/steer_bot/ackermann_steering_controller/odom"),
                                         Odometry,
                                         self.odomCB,
                                         queue_size=1)
        self.pose_sub = rospy.Subscriber("/initialpose",
                                         PoseWithCovarianceStamped,
                                         self.clicked_pose,
                                         queue_size=1)
        self.click_sub = rospy.Subscriber("/clicked_point",
                                          PointStamped,
                                          self.clicked_pose,
                                          queue_size=1)

    def get_omap(self):
        try:
            map_service_name = rospy.get_param("~static_map", "static_map")
            rospy.loginfo("%s Getting map from service: %s", self.LOG_TAG,
                          map_service_name)

            rospy.wait_for_service(map_service_name)

            map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

            oMap = range_libc.PyOMap(map_msg)
            self.map_info = map_msg.info
            self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS /
                                    self.map_info.resolution)

            self.initialize_range_method(oMap)

            array_255 = np.array(map_msg.data).reshape(
                (map_msg.info.height, map_msg.info.width))

            self.permissible_region = (array_255 == 0)
            self.map_initialized = True
            rospy.loginfo("%s Successfully loaded map", self.LOG_TAG)

        except rospy.ServiceException as e:
            rospy.logerr("%s Error while getting map: %s", self.LOG_TAG,
                         str(e))
            self.map_initialized = False

    def initialize_range_method(self, oMap):
        try:
            rospy.loginfo("%s Initializing range method: %s", self.LOG_TAG,
                          self.WHICH_RM)

            if self.WHICH_RM == "bl":
                self.range_method = range_libc.PyBresenhamsLine(
                    oMap, self.MAX_RANGE_PX)
            elif "cddt" in self.WHICH_RM:
                self.range_method = range_libc.PyCDDTCast(
                    oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
                if self.WHICH_RM == "pcddt":
                    rospy.loginfo("%s Pruning...", self.LOG_TAG)
                    self.range_method.prune()
            elif self.WHICH_RM == "rm":
                self.range_method = range_libc.PyRayMarching(
                    oMap, self.MAX_RANGE_PX)
            elif self.WHICH_RM == "rmgpu":
                self.range_method = range_libc.PyRayMarchingGPU(
                    oMap, self.MAX_RANGE_PX)
            elif self.WHICH_RM == "glt":
                self.range_method = range_libc.PyGiantLUTCast(
                    oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            else:
                rospy.logerr("%s Invalid range method specified: %s",
                             self.LOG_TAG, self.WHICH_RM)
                self.map_initialized = False

            rospy.loginfo("%s Done initializing range method", self.LOG_TAG)

        except Exception as e:
            rospy.logerr("%s Error while initializing range method: %s",
                         self.LOG_TAG, str(e))
            self.map_initialized = False

    def publish_tf(self, pose, stamp=None):
        try:
            if stamp is None:
                stamp = rospy.Time.now()

            # Publish TF transform
            self.pub_tf.sendTransform(
                (pose[0], pose[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, pose[2]), stamp,
                "/steer_bot/velodyne2", "map")

            if self.PUBLISH_ODOM:
                # Publish Odometry message
                odom = Odometry()
                odom.header = self.make_header("map", stamp)
                odom.pose.pose.position.x = pose[0]
                odom.pose.pose.position.y = pose[1]
                odom.pose.pose.orientation = self.angle_to_quaternion(pose[2])
                self.odom_pub.publish(odom)

            return True  # or any other value to indicate success

        except rospy.ROSException as e:
            rospy.logerr("%s Error while publishing TF/Odometry: %s",
                         self.LOG_TAG, str(e))
            return False  # or any other value to indicate failure
        except Exception as ex:
            rospy.logerr(
                "%s Unexpected error during TF/Odometry publishing: %s",
                self.LOG_TAG, str(ex))
            return False  # or any other value to indicate failure

    def visualize(self):
        try:
            if not self.DO_VIZ:
                return

            if self.pose_pub.get_num_connections() > 0 and isinstance(
                    self.inferred_pose, np.ndarray):
                # Publish the inferred pose for visualization
                ps = PoseStamped()
                ps.header = self.make_header("map")
                ps.pose.position.x = self.inferred_pose[0]
                ps.pose.position.y = self.inferred_pose[1]
                ps.pose.orientation = self.angle_to_quaternion(
                    self.inferred_pose[2])
                self.pose_pub.publish(ps)

            if self.particle_pub.get_num_connections() > 0:
                # publish a downsampled version of the particle distribution to avoid a lot of latency
                if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                    # randomly downsample particles
                    proposal_indices = np.random.choice(self.particle_indices,
                                                        self.MAX_VIZ_PARTICLES,
                                                        p=self.weights)
                    self.publish_particles(self.particles[proposal_indices, :])
                else:
                    self.publish_particles(self.particles)

            if self.pub_fake_scan.get_num_connections() > 0 and isinstance(
                    self.ranges, np.ndarray):
                # generate the scan from the point of view of the inferred position for visualization
                self.viz_queries[:, 0] = self.inferred_pose[0]
                self.viz_queries[:, 1] = self.inferred_pose[1]
                self.viz_queries[:,
                                 2] = self.downsampled_angles + self.inferred_pose[
                                     2]
                self.range_method.calc_range_many(self.viz_queries,
                                                  self.viz_ranges)
                self.publish_scan(self.downsampled_angles, self.viz_ranges)

        except rospy.ROSException as e:
            rospy.logerr("%s Error while visualizing: %s", self.LOG_TAG,
                         str(e))
        except Exception as ex:
            rospy.logerr("%s Unexpected error during visualization: %s",
                         self.LOG_TAG, str(ex))

    def publish_particles(self, particles):
        try:
            # Publish the given particles as a PoseArray object
            pa = PoseArray()
            pa.header = self.make_header("map")
            pa.poses = self.particles_to_poses(particles)
            self.particle_pub.publish(pa)
        except rospy.ROSException as e:
            rospy.logerr("%s Error while publishing particles: %s",
                         self.LOG_TAG, str(e))
        except Exception as ex:
            rospy.logerr("%s Unexpected error during particles publishing: %s",
                         self.LOG_TAG, str(ex))

    def publish_scan(self, angles, ranges):
        try:
            # Publish the given angles and ranges as a laser scan message
            ls = LaserScan()
            ls.header = self.make_header("/steer_bot/velodyne2",
                                         stamp=self.last_stamp)
            ls.angle_min = np.min(angles)
            ls.angle_max = np.max(angles)
            ls.angle_increment = np.abs(angles[0] - angles[1])
            ls.range_min = 0
            ls.range_max = np.max(ranges)
            ls.ranges = ranges
            self.pub_fake_scan.publish(ls)
        except rospy.ROSException as e:
            rospy.logerr("%s Error while publishing laser scan: %s",
                         self.LOG_TAG, str(e))
        except Exception as ex:
            rospy.logerr(
                "%s Unexpected error during laser scan publishing: %s",
                self.LOG_TAG, str(ex))

    def lidarCB(self, msg):
        try:
            # Initializes reused buffers and stores the relevant laser scanner data for later use
            if not isinstance(self.laser_angles, np.ndarray):
                rospy.loginfo("%s Received LIDAR scan", self.LOG_TAG)
                self.laser_angles = np.linspace(msg.angle_min, msg.angle_max,
                                                len(msg.ranges))
                self.downsampled_angles = np.copy(
                    self.laser_angles[0::self.ANGLE_STEP]).astype(np.float32)
                self.viz_queries = np.zeros(
                    (self.downsampled_angles.shape[0], 3), dtype=np.float32)
                self.viz_ranges = np.zeros(self.downsampled_angles.shape[0],
                                           dtype=np.float32)
                rospy.loginfo("%s %d", self.LOG_TAG,
                              self.downsampled_angles.shape[0])

            # store the necessary scanner information for later processing
            self.downsampled_ranges = np.array(msg.ranges[::self.ANGLE_STEP])
            self.lidar_initialized = True
        except rospy.ROSException as e:
            rospy.logerr("%s Error in LiDAR callback: %s", self.LOG_TAG,
                         str(e))
        except Exception as ex:
            rospy.logerr("%s Unexpected error in LiDAR callback: %s",
                         self.LOG_TAG, str(ex))

    def odomCB(self, msg):
        try:
            position = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y])
            orientation = self.quaternion_to_angle(msg.pose.pose.orientation)
            pose = np.array([position[0], position[1], orientation])

            if isinstance(self.last_pose, np.ndarray):
                # changes in x, y, theta in the local coordinate system of the car
                rot = self.rotation_matrix(-self.last_pose[2])
                delta = np.array([position - self.last_pose[0:2]]).transpose()
                local_delta = (rot * delta).transpose()

                self.odometry_data = np.array([
                    local_delta[0, 0], local_delta[0, 1],
                    orientation - self.last_pose[2]
                ])
                self.last_pose = pose
                self.last_stamp = msg.header.stamp
                self.odom_initialized = True
            else:
                rospy.loginfo("%s Received Odometry message", self.LOG_TAG)
                self.last_pose = pose

            # this topic is slower than lidar, so update every time we receive a message
            self.update()
        except rospy.ROSException as e:
            rospy.logerr("%s Error in Odometry callback: %s", self.LOG_TAG,
                         str(e))

    def clicked_pose(self, msg):
        try:
            if isinstance(msg, PointStamped):
                self.initialize_global()
            elif isinstance(msg, PoseWithCovarianceStamped):
                self.initialize_particles_pose(msg.pose.pose)
        except rospy.ROSException as e:
            rospy.logerr("%s Error in clicked_pose callback: %s", self.LOG_TAG,
                         str(e))
        except Exception as ex:
            rospy.logerr("%s Unexpected error in clicked_pose callback: %s",
                         self.LOG_TAG, str(ex))

    def initialize_particles_pose(self, pose):
        try:

            rospy.loginfo("%s Setting Pose", self.LOG_TAG)
            rospy.loginfo(pose)
            self.state_lock.acquire()
            self.weights = np.ones(self.MAX_PARTICLES) / float(
                self.MAX_PARTICLES)
            self.particles[:, 0] = pose.position.x + np.random.normal(
                loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
            self.particles[:, 1] = pose.position.y + np.random.normal(
                loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
            self.particles[:, 2] = self.quaternion_to_angle(
                pose.orientation) + np.random.normal(
                    loc=0.0, scale=0.4, size=self.MAX_PARTICLES)
            self.state_lock.release()
        except rospy.ROSException as e:
            rospy.logerr("%s Error in initialize_particles_pose: %s",
                         self.LOG_TAG, str(e))
        except Exception as ex:
            rospy.logerr(
                "%s Unexpected error in initialize_particles_pose: %s",
                self.LOG_TAG, str(ex))

    def initialize_global(self):
        try:
            '''
            Spread the particle distribution over the permissible region of the state space.
            '''
            rospy.loginfo("%s Global Initialization", self.LOG_TAG)
            # randomize over grid coordinate space
            self.state_lock.acquire()
            permissible_x, permissible_y = np.where(
                self.permissible_region == 1)
            indices = np.random.randint(0,
                                        len(permissible_x),
                                        size=self.MAX_PARTICLES)

            permissible_states = np.zeros((self.MAX_PARTICLES, 3))
            permissible_states[:, 0] = permissible_y[indices]
            permissible_states[:, 1] = permissible_x[indices]
            permissible_states[:, 2] = np.random.random(
                self.MAX_PARTICLES) * np.pi * 2.0

            self.map_to_world(permissible_states, self.map_info)
            self.particles = permissible_states
            self.weights[:] = 1.0 / self.MAX_PARTICLES
            self.state_lock.release()
        except rospy.ROSException as e:
            rospy.logerr("%s Error in initialize_global: %s", self.LOG_TAG,
                         str(e))
        except Exception as ex:
            rospy.logerr("%s Unexpected error in initialize_global: %s",
                         self.LOG_TAG, str(ex))

    def precompute_sensor_model(self):
        try:
            '''
            Generate and store a table which represents the sensor model. For each discrete computed
            range value, this provides the probability of measuring any (discrete) range.

            This table is indexed by the sensor model at runtime by discretizing the measurements
            and computed ranges from RangeLibc.
            '''
            rospy.loginfo("%s Precomputing sensor model", self.LOG_TAG)
            # sensor model constants
            z_short = self.Z_SHORT
            z_max = self.Z_MAX
            z_rand = self.Z_RAND
            z_hit = self.Z_HIT
            sigma_hit = self.SIGMA_HIT

            table_width = int(self.MAX_RANGE_PX) + 1
            self.sensor_model_table = np.zeros((table_width, table_width))

            t = time.time()
            # d is the computed range from RangeLibc
            for d in range(table_width):
                norm = 0.0
                sum_unknown = 0.0
                # r is the observed range from the lidar unit
                for r in range(table_width):
                    prob = 0.0
                    z = float(r - d)
                    # reflects from the intended object
                    prob += z_hit * np.exp(
                        -(z * z) /
                        (2.0 * sigma_hit * sigma_hit)) / (sigma_hit *
                                                          np.sqrt(2.0 * np.pi))

                    # observed range is less than the predicted range - short reading
                    if r < d:
                        prob += 2.0 * z_short * (d - r) / float(d)

                    # erroneous max range measurement
                    if int(r) == int(self.MAX_RANGE_PX):
                        prob += z_max

                    # random measurement
                    if r < int(self.MAX_RANGE_PX):
                        prob += z_rand * 1.0 / float(self.MAX_RANGE_PX)

                    norm += prob
                    self.sensor_model_table[int(r), int(d)] = prob

                # normalize
                self.sensor_model_table[:, int(d)] /= norm

            # upload the sensor model to RangeLib for ultra-fast resolution
            if self.RANGELIB_VAR > 0:
                self.range_method.set_sensor_model(self.sensor_model_table)
        except rospy.ROSException as e:
            rospy.logerr("%s Error in precompute_sensor_model: %s",
                         self.LOG_TAG, str(e))
        except Exception as ex:
            rospy.logerr("%s Unexpected error in precompute_sensor_model: %s",
                         self.LOG_TAG, str(ex))

    def motion_model(self, proposal_dist, action):
        try:
            cosines = np.cos(proposal_dist[:, 2])
            sines = np.sin(proposal_dist[:, 2])

            self.local_deltas[:, 0] = cosines * action[0] - sines * action[1]
            self.local_deltas[:, 1] = sines * action[0] + cosines * action[1]
            self.local_deltas[:, 2] = action[2]

            proposal_dist[:, :] += self.local_deltas
            proposal_dist[:, 0] += np.random.normal(
                loc=0.0,
                scale=self.MOTION_DISPERSION_X,
                size=self.MAX_PARTICLES)
            proposal_dist[:, 1] += np.random.normal(
                loc=0.0,
                scale=self.MOTION_DISPERSION_Y,
                size=self.MAX_PARTICLES)
            proposal_dist[:, 2] += np.random.normal(
                loc=0.0,
                scale=self.MOTION_DISPERSION_THETA,
                size=self.MAX_PARTICLES)

        except Exception as e:
            rospy.logerr(f"%s Motion Model Estimate Error: {str(e)}",
                         self.LOG_TAG)

    def sensor_model(self, proposal_dist, obs, weights):
        num_rays = self.downsampled_angles.shape[0]
        # only allocate buffers once to avoid slowness
        if self.first_sensor_update:
            if self.RANGELIB_VAR <= 1:
                self.queries = np.zeros((num_rays * self.MAX_PARTICLES, 3),
                                        dtype=np.float32)
            else:
                self.queries = np.zeros((self.MAX_PARTICLES, 3),
                                        dtype=np.float32)

            self.ranges = np.zeros(num_rays * self.MAX_PARTICLES,
                                   dtype=np.float32)
            self.tiled_angles = np.tile(self.downsampled_angles,
                                        self.MAX_PARTICLES)
            self.first_sensor_update = False

        if self.RANGELIB_VAR == VAR_RADIAL_CDDT_OPTIMIZATIONS:
            if "cddt" in self.WHICH_RM:
                self.queries[:, :] = proposal_dist[:, :]
                self.range_method.calc_range_many_radial_optimized(
                    num_rays, self.downsampled_angles[0],
                    self.downsampled_angles[-1], self.queries, self.ranges)

                # evaluate the sensor model
                self.range_method.eval_sensor_model(obs, self.ranges,
                                                    self.weights, num_rays,
                                                    self.MAX_PARTICLES)
                # apply the squash factor
                self.weights = np.power(self.weights, self.INV_SQUASH_FACTOR)
            else:
                rospy.loginfo(
                    " %s Cannot use radial optimizations with non-CDDT based methods, use rangelib_variant 2",
                    self.LOG_TAG)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_repeat_angles_eval_sensor_model(
                self.queries, self.downsampled_angles, obs, self.weights)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR:
            if self.SHOW_FINE_TIMING:
                t_start = time.time()
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            self.queries[:, :] = proposal_dist[:, :]
            if self.SHOW_FINE_TIMING:
                t_init = time.time()
            self.range_method.calc_range_repeat_angles(self.queries,
                                                       self.downsampled_angles,
                                                       self.ranges)
            if self.SHOW_FINE_TIMING:
                t_range = time.time()
            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights,
                                                num_rays, self.MAX_PARTICLES)
            if self.SHOW_FINE_TIMING:
                t_eval = time.time()
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
            if self.SHOW_FINE_TIMING:
                t_squash = time.time()
                t_total = (t_squash - t_start) / 100.0

            if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
                rospy.loginfo(" %s sensor_model: init: ", self.LOG_TAG, np.round((t_init-t_start)/t_total, 2), "range:", np.round((t_range-t_init)/t_total, 2), \
                      "eval:", np.round((t_eval-t_range)/t_total, 2), "squash:", np.round((t_squash-t_eval)/t_total, 2))
        elif self.RANGELIB_VAR == VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            # this part is inefficient since it requires a lot of effort to construct this redundant array
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays)
            self.queries[:, 2] += self.tiled_angles

            self.range_method.calc_range_many(self.queries, self.ranges)

            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights,
                                                num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_NO_EVAL_SENSOR_MODEL:
            # this version directly uses the sensor model in Python, at a significant computational cost
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays)
            self.queries[:, 2] += self.tiled_angles

            # compute the ranges for all the particles in a single functon call
            self.range_method.calc_range_many(self.queries, self.ranges)

            # resolve the sensor model by discretizing and indexing into the precomputed table
            obs /= float(self.map_info.resolution)
            ranges = self.ranges / float(self.map_info.resolution)
            obs[obs > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
            ranges[ranges > self.MAX_RANGE_PX] = self.MAX_RANGE_PX

            intobs = np.rint(obs).astype(np.uint16)
            intrng = np.rint(ranges).astype(np.uint16)

            # compute the weight for each particle
            for i in range(self.MAX_PARTICLES):
                weight = np.product(
                    self.sensor_model_table[intobs,
                                            intrng[i * num_rays:(i + 1) *
                                                   num_rays]])
                weight = np.power(weight, self.INV_SQUASH_FACTOR)
                weights[i] = weight
        else:
            rospy.loginfo(" %s Range-Lib variant not set, Set the variant",
                          self.LOG_TAG)

    def MCL(self, a, o):

        if self.SHOW_FINE_TIMING:
            t = time.time()
        # draw the proposal distribution from the old particles
        proposal_indices = np.random.choice(self.particle_indices,
                                            self.MAX_PARTICLES,
                                            p=self.weights)
        proposal_distribution = self.particles[proposal_indices, :]
        if self.SHOW_FINE_TIMING:
            t_propose = time.time()

        # compute the motion model to update the proposal distribution
        self.motion_model(proposal_distribution, a)
        if self.SHOW_FINE_TIMING:
            t_motion = time.time()

        # compute the sensor model
        self.sensor_model(proposal_distribution, o, self.weights)
        if self.SHOW_FINE_TIMING:
            t_sensor = time.time()

        # normalize importance weights
        self.weights /= np.sum(self.weights)
        if self.SHOW_FINE_TIMING:
            t_norm = time.time()
            t_total = (t_norm - t) / 100.0

        if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
            rospy.loginfo(" %s MCL: propose: ",self.LOG_TAG,  np.round((t_propose-t)/t_total, 2), "motion:", np.round((t_motion-t_propose)/t_total, 2), \
                  "sensor:", np.round((t_sensor-t_motion)/t_total, 2), "norm:", np.round((t_norm-t_sensor)/t_total, 2))

        # save the particles
        self.particles = proposal_distribution

    def expected_pose(self):
        # returns the expected value of the pose given the particle distribution
        return np.dot(self.particles.transpose(), self.weights)

    def update(self):
        if self.lidar_initialized and self.odom_initialized and self.map_initialized:
            if self.state_lock.locked():
                print("Concurrency error avoided")
            else:
                self.state_lock.acquire()
                self.timer.tick()
                self.iters += 1

                t1 = time.time()
                observation = np.copy(self.downsampled_ranges).astype(
                    np.float32)
                action = np.copy(self.odometry_data)
                self.odometry_data = np.zeros(3)
                

                # run the MCL update algorithm
                self.MCL(action, observation)

                # compute the expected value of the robot pose
                self.inferred_pose = self.expected_pose()
                self.state_lock.release()
                t2 = time.time()
                # publish transformation frame based on inferred pose
                self.publish_tf(self.inferred_pose, self.last_stamp)

                # this is for tracking particle filter speed
                ips = 1.0 / (t2 - t1)
                self.smoothing.append(ips)
                if self.iters % 10 == 0:
                    rospy.loginfo_throttle(
                        1, "%s MCL Iteration loop rate: %d possible: %d",
                        self.LOG_TAG, int(self.timer.fps()),
                        int(self.smoothing.mean()))
                self.visualize()

    def angle_to_quaternion(self, angle):
        return Quaternion(
            *tf.transformations.quaternion_from_euler(0, 0, angle))

    def quaternion_to_angle(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (x, y, z, w))
        return yaw

    def rotation_matrix(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.matrix([[c, -s], [s, c]])

    def particle_to_pose(self, particle):
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.orientation = self.angle_to_quaternion(particle[2])
        return pose

    def particles_to_poses(self, particles):
        return map(self.particle_to_pose, particles)

    def make_header(self, frame_id, stamp=None):
        if stamp == None:
            stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def map_to_world_slow(self, x, y, t, map_info):
        scale = map_info.resolution
        angle = self.quaternion_to_angle(map_info.origin.orientation)
        rot = self.rotation_matrix(angle)
        trans = np.array([[map_info.origin.position.x],
                          [map_info.origin.position.y]])

        map_c = np.array([[x], [y]])
        world = (rot * map_c) * scale + trans

        return world[0, 0], world[1, 0], t + angle

    def map_to_world(self, poses, map_info):

        scale = map_info.resolution
        angle = self.quaternion_to_angle(map_info.origin.orientation)

        # rotation
        c, s = np.cos(angle), np.sin(angle)
        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(poses[:, 0])
        poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
        poses[:, 1] = s * temp + c * poses[:, 1]

        # scale
        poses[:, :2] *= float(scale)

        # translate
        poses[:, 0] += map_info.origin.position.x
        poses[:, 1] += map_info.origin.position.y
        poses[:, 2] += angle

    def world_to_map(self, poses, map_info):
        scale = map_info.resolution
        angle = -self.quaternion_to_angle(map_info.origin.orientation)

        # translation
        poses[:, 0] -= map_info.origin.position.x
        poses[:, 1] -= map_info.origin.position.y

        # scale
        poses[:, :2] *= (1.0 / float(scale))

        # rotation
        c, s = np.cos(angle), np.sin(angle)
        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(poses[:, 0])
        poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
        poses[:, 1] = s * temp + c * poses[:, 1]
        poses[:, 2] += angle

    def world_to_map_slow(self, x, y, t, map_info):
        scale = map_info.resolution
        angle = self.quaternion_to_angle(map_info.origin.orientation)
        rot = self.rotation_matrix(-angle)
        trans = np.array([[map_info.origin.position.x],
                          [map_info.origin.position.y]])

        world = np.array([[x], [y]])
        map_c = rot * ((world - trans) / float(scale))
        return map_c[0, 0], map_c[1, 0], t - angle


if __name__ == "__main__":
    rospy.init_node("particle_filter_localization")

    pf = ParticleFiler()
    rospy.spin()
