# Import necessary modules
import sys
from utilities import Logger
from rclpy.time import Time
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom
from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter
from rclpy import init, spin, spin_once
import numpy as np
import message_filters

# Define constants
rawSensors = 0
kalmanFilter = 1
odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)

# Define the localization class
class localization(Node):
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_v","kf_w","kf_x", "kf_y","stamp"]):
        super().__init__("localizer")
        self.loc_logger = Logger(loggerName, loggerHeaders)
        self.pose = None
        
        # Initialize based on the type of localization
        if type == rawSensors:
            self.initRawSensors()
        elif type == kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return

    def initRawSensors(self):
        # Create a subscription to the "/odom" topic and set the callback function
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        # Set up the quantities for the Extended Kalman Filter (EKF)
        x = np.zeros((6,)) # initial state assumed 0
        Q = 0.5 * np.eye(4) # process noise
        R = 0.4 * np.eye(6) # measurement noise
        P = np.eye(6) # initial covariance
        
        # Create an instance of the Kalman Filter class
        self.kf = kalman_filter(P, Q, R, x, dt)
        
        # Use the odometry and IMU data for the EKF
        self.imu_sub = message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        self.odom_sub = message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        
        # Synchronize the odometry and IMU messages using approximate time
        time_syncher = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        # Use the EKF to perform state estimation
        
        # Take the measurements
        # The measurements are the linear velocity and angular velocity from the odometry message
        # and the linear acceleration in x and y from the IMU message
        z = np.array([odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z, imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y])
        
        # Implement the two steps for estimation: predict and update
        self.kf.predict()
        self.kf.update(z)
        
        # Get the estimate
        xhat = self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        self.pose = [xhat[0], xhat[1], xhat[2], odom_msg.header.stamp]
        x, y, th, w, v, vdot = xhat
        
        # Log the data
        self.loc_logger.log_values([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, vdot, w*v, v, w, x, y, Time.from_msg(odom_msg.header.stamp).nanoseconds])
      
    def odom_callback(self, pose_msg):
        # Update the pose based on the odometry message
        self.pose = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, euler_from_quaternion(pose_msg.pose.pose.orientation), pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__ == "__main__":
    # Initialize the ROS client library
    init()
    
    # Create an instance of the localization class
    LOCALIZER = localization()
    
    # Start spinning the node
    spin(LOCALIZER)
