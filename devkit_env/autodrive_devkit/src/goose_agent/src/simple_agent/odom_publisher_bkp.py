#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Imu, "/autodrive/roboracer_1/imu", self.imu_callback, 10)
        self.create_subscription(JointState, "/autodrive/roboracer_1/left_encoder", self.lencoder_callback, 10)
        self.create_subscription(JointState, "/autodrive/roboracer_1/right_encoder", self.rencoder_callback, 10)

        # Publisher
        self.odometry_pub = self.create_publisher(Odometry, "/odom", 10)

        # Timer (control loop)
        timer_period = 1.0 / 20.0
        self.create_timer(timer_period, self.get_current_position)

        # IMU / encoder state
        self.quaternions = None
        self.yaw = 0.0
        self.prev_yaw = None
        self.angular_vel_imu = None
        self.linear_accel_imu = None

        # Encoder angles (previous values)
        self.l_prev = None
        self.r_prev = None
        # Delta angles (accumulated between integrations)
        self.delta_l = 0.0
        self.delta_r = 0.0

        # Robot geometry
        self.wheel_radius = 0.0590
        self.track_width = 0.2360

        # Pose state (global)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Timing for velocity computation
        self.last_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        # store quaternion and derived yaw
        self.quaternions = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        self.prev_yaw = self.yaw if self.prev_yaw is None else self.prev_yaw
        self.yaw = yaw
        self.angular_vel_imu = msg.angular_velocity
        self.linear_accel_imu = msg.linear_acceleration

    def lencoder_callback(self, msg: JointState):
        # guard empty
        if not msg.position:
            return
        current = msg.position[0]
        if self.l_prev is None:
            # first reading: just store
            self.l_prev = current
            return
        # compute delta since last encoder msg and accumulate until integration
        self.delta_l += (current - self.l_prev)
        self.l_prev = current

    def rencoder_callback(self, msg: JointState):
        # guard empty
        if not msg.position:
            return
        current = msg.position[0]
        if self.r_prev is None:
            self.r_prev = current
            return
        self.delta_r += (current - self.r_prev)
        self.r_prev = current

    def get_current_position(self):
        # require quaternion/yaw present
        if self.quaternions is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            # protect divide by zero and negative dt
            dt = None

        # convert encoder angle deltas to wheel linear displacements
        s_l = self.wheel_radius * self.delta_l
        s_r = self.wheel_radius * self.delta_r

        # reset deltas immediately so we don't double count
        self.delta_l = 0.0
        self.delta_r = 0.0

        # center (COM) forward displacement
        s = 0.5 * (s_r + s_l)

        # integrate position using IMU yaw as heading
        # using midpoint heading is better when using wheel-based dtheta,
        # but here we rely on IMU yaw (assumed up-to-date)
        self.x += s * math.cos(self.yaw)
        self.y += s * math.sin(self.yaw)

        # compute velocity estimates if dt available
        linear_velocity = 0.0
        angular_velocity = 0.0
        if dt is not None and dt > 0.0:
            linear_velocity = s / dt
            # angular velocity: prefer IMU angular z if available (more direct)
            if self.angular_vel_imu is not None:
                angular_velocity = self.angular_vel_imu.z
            else:
                # fallback: numerical derivative of yaw
                # be careful with wrap-around
                if self.prev_yaw is not None:
                    dyaw = math.atan2(math.sin(self.yaw - self.prev_yaw), math.cos(self.yaw - self.prev_yaw))
                    angular_velocity = dyaw / dt
            self.last_time = now
            self.prev_yaw = self.yaw

        # build odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = 'base_link'

        # pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        odom.pose.pose.orientation.x = self.quaternions.x
        odom.pose.pose.orientation.y = self.quaternions.y
        odom.pose.pose.orientation.z = self.quaternions.z
        odom.pose.pose.orientation.w = self.quaternions.w

        # twist (publish linear.x and angular.z)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        # publish
        self.odometry_pub.publish(odom)

        # build the transform broadcaster
        t = TransformStamped()

        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # translation and rotation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        t.transform.rotation.x  = self.quaternions.x
        t.transform.rotation.y  = self.quaternions.y
        t.transform.rotation.z  = self.quaternions.z
        t.transform.rotation.w  = self.quaternions.w

        # send the transformations
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()