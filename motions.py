# Imports
import rclpy

from math import exp 

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed:
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):

    def __init__(self, motion_type=0):

        super().__init__("motion_types")

        self.type=motion_type

        self.radius_=0.0

        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False

        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)

        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "stamp"])

        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)
        self.imu_initialized=True

        # ENCODER subscription
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)
        self.odom_initialized=True

        # LaserScan subscription
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile=qos)
        self.laser_initialized=True

        self.create_timer(0.1, self.timer_callback)

        if self.imu_initialized and self.odom_initialized and self.laser_initialized:
        	self.successful_init=False

    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        # Grab imu values
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        stamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1000000000

        # Populate values_list with imu msg values
        values_list = [acc_x, acc_y, angular_z, stamp]

        # Log imu msgs
        self.imu_logger.log_values(values_list)

    def odom_callback(self, odom_msg: Odometry):
        # Get odometery valus
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        quat_x = odom_msg.pose.pose.orientation.x
        quat_y = odom_msg.pose.pose.orientation.y
        quat_z = odom_msg.pose.pose.orientation.z
        quat_w = odom_msg.pose.pose.orientation.w
        quat = [quat_x, quat_y, quat_z, quat_w]
        # Convert quaternion into euler angle
        th = euler_from_quaternion(quat)
        
        stamp = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec / 1000000000

        # Populate values_list with odom msg values
        values_list = [x, y, th, stamp]

        # Log odom msgs
        self.odom_logger.log_values(values_list)

    def laser_callback(self, laser_msg: LaserScan):

        # Grab laser range values
        ranges = laser_msg.ranges
        stamp = laser_msg.header.stamp.sec + laser_msg.header.stamp.nanosec / 1000000000

        # Populate values_list with laser msg values
        values_list = [ranges, stamp]

        # Log laser msgs with position msg at that time
        self.laser_logger.log_values(values_list)

    def timer_callback(self):

        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True

        if not self.successful_init:
            return

        cmd_vel_msg=Twist()

        if self.type==CIRCLE:
        	cmd_vel_msg=self.make_circular_twist()

        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()

        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()

        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit

        self.vel_publisher.publish(cmd_vel_msg)

    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        # Fill up the twist msg for circular motion.
        msg=Twist()
        # A constant linear and angular velocity creates a circle.
        msg.linear.x = 0.2
        msg.angular.z = 0.4
        return msg

    def make_spiral_twist(self):
        # Fill up the twist msg for spiral motion
        msg=Twist()
        seconds = self.get_clock().now().nanoseconds/1000_000_000
        # A constant linear and with a decreasing angular velocity with time creates a spiral.
        msg.linear.x = 0.2
        msg.angular.z = 0.1*exp(seconds/-4) + 0.1
        return msg

    def make_acc_line_twist(self):
        # Fill up the twist msg for line motion
        msg=Twist()
        # 0 angular velocity and constant linear velocity creates a straight line.
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        return msg

import argparse

if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")

    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
