import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin, shutdown
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        #Currently in real robot config
        odom_qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.create_subscription(odom, "/odom", self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg: odom):
        
        # TODO Part 3: Read x,y, theta, and record the stamp
        quat_x = pose_msg.pose.pose.orientation.x
        quat_y = pose_msg.pose.pose.orientation.y
        quat_z = pose_msg.pose.pose.orientation.z
        quat_w = pose_msg.pose.pose.orientation.w

        stamp = pose_msg.header.stamp

        self.pose = [ pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, euler_from_quaternion([quat_x, quat_y, quat_z, quat_w]), stamp]
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
if __name__ == '__main__':
    init() 
    # Create a localization node with the localization type set to rawSensor
    localization_node = localization(localizationType=rawSensor) 
    try:
        # Start the localization node
        spin(localization_node)
    except KeyboardInterrupt:
        pass

    # Destroy the localization node explicitly 
    localization_node.destroy_node()
    shutdown()
    
