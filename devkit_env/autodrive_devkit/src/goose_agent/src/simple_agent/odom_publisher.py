import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_inverse
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) 

        # Subscribers
        self.create_subscription(
            Imu, 
            "/autodrive/roboracer_1/imu", 
            self.imu_callback, 
            10
        )
        self.create_subscription(
            JointState, 
            "/autodrive/roboracer_1/left_encoder", 
            self.lencoder_callback, 
            10
        )
        self.create_subscription(
            JointState, 
            "/autodrive/roboracer_1/right_encoder", 
            self.rencoder_callback, 
            10
        )

        # Publisher
        self.odometry_pub = self.create_publisher(
            Odometry, 
            "/odom", 
            10
        )

        # Timer (control loop)
        timer_period = 1.0 / 20.0
        self.create_timer(timer_period, self.get_current_position)
        self.prev_time = self.get_clock().now()

        # IMU / encoder state
        self.quaternions = None
        self.yaw = 0.0
        self.prev_yaw = None

        # Encoder angles (previous values)
        self.l_rotation = None
        self.r_rotation = None
        self.prev_l_rot = None
        self.prev_r_rot = None

        # Robot geometry
        self.wheel_radius = 0.0590
        self.track_width = 0.2360
        self.q = Quaternion
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # World quaternion
        self.initial_world_quat = None


    def convert_to_local_quat(self, quat: list):
        """
        Convert world->imu quaternion to robot-local yaw.
        """

        # 1. Save the initial world orientation as reference
        if self.initial_world_quat is None:
            self.initial_world_quat = quat
            return 0.0, 0.0, 0.0, 0.0  # yaw_local = 0 at start

        # 2. Compute relative rotation:
        #    q_local = q_initial^-1 * q_current
        q_initial_inv = quaternion_inverse(self.initial_world_quat)
        q_local = quaternion_multiply(q_initial_inv, quat)

        return q_local


    def imu_callback(self, msg: Imu):
        # store quaternion and derived yaw
        self.quaternions = msg.orientation
        _local_quat = self.convert_to_local_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Reassign the quaternions
        self.quaternions.x = _local_quat[0]
        self.quaternions.y = _local_quat[1]
        self.quaternions.z = _local_quat[2]
        self.quaternions.w = _local_quat[3]

        # Assign the yaw
        roll, pitch, yaw = euler_from_quaternion(_local_quat)
        self.yaw = yaw

    def tick_to_angular(self, tick: float):
        PPR = 16
        CR = 120
        return 2 * math.pi * tick / (PPR*CR)

    def lencoder_callback(self, msg: JointState):
        cur_l_rot = self.tick_to_angular(msg.position[0])
        if not self.prev_l_rot:
            self.l_rotation = 0.0
        else:
            self.l_rotation = cur_l_rot - self.prev_l_rot
        self.prev_l_rot = cur_l_rot

    def rencoder_callback(self, msg: JointState):
        cur_r_rot = self.tick_to_angular(msg.position[0])
        if not self.prev_r_rot:
            self.r_rotation = 0.0
        else:
            self.r_rotation = cur_r_rot - self.prev_r_rot
        self.prev_r_rot = cur_r_rot

    def get_current_position(self):
        # require quaternion/yaw present
        if self.l_rotation is None or self.r_rotation is None or self.quaternions is None:
            return
        # convert encoder angle deltas to wheel linear displacements
        s_l = 350 * self.wheel_radius * self.l_rotation
        s_r = 350 * self.wheel_radius * self.r_rotation

        print("Wheel rotation", s_l, s_r)

        # center (COM) forward displacement
        s = 0.5 * (s_r + s_l)
        angular_displacement = (s_r - s_l)/self.track_width

        self.x = self.x + s*math.cos(self.yaw + angular_displacement)
        self.y = self.y + s*math.sin(self.yaw + angular_displacement)
        self.yaw += angular_displacement

        print("Pos", self.x, self.y)

        # Calculate speed from odom
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9 
        self.prev_time = current_time
        linear_x = s / dt
        angular_z = angular_displacement / dt

        # Calculate the quaternion
        self.q = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        # Build odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        odom.pose.pose.orientation.x = self.q[0]
        odom.pose.pose.orientation.y = self.q[1]
        odom.pose.pose.orientation.z = self.q[2]
        odom.pose.pose.orientation.w = self.q[3]

        # twist
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = 0.0 
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_z

        # Covariance
        odom.twist.covariance = [float(v) for v in [
            0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0, 0.05
        ]]

        # publish
        self.odometry_pub.publish(odom)

        # Broadcast 
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        t.transform.rotation.x = self.q[0]
        t.transform.rotation.y = self.q[1]
        t.transform.rotation.z = self.q[2]
        t.transform.rotation.w = self.q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()