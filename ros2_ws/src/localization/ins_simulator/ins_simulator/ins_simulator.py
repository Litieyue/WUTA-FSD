import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _write_yaw_quaternion(orientation, yaw):
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)


def _finite_or(value, fallback=0.0):
    if isinstance(value, (int, float)) and math.isfinite(value):
        return float(value)
    return fallback


class INSSimulator(Node):
    def __init__(self):
        super().__init__("ins_simulator")

        self.declare_parameter("input_topic", "/sim/ground_truth")
        self.declare_parameter("output_topic", "/cg410/odometry")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("position_noise_std", 0.05)
        self.declare_parameter("z_noise_std", 0.0)
        self.declare_parameter("yaw_noise_std_deg", 0.5)
        self.declare_parameter("velocity_noise_std", 0.02)
        self.declare_parameter("angular_velocity_noise_std", 0.0)
        self.declare_parameter("seed", 42)
        self.declare_parameter("use_ground_truth_stamp", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.position_noise_std = float(
            self.get_parameter("position_noise_std").value
        )
        self.z_noise_std = float(self.get_parameter("z_noise_std").value)
        yaw_noise_deg = float(self.get_parameter("yaw_noise_std_deg").value)
        self.yaw_noise_std = math.radians(yaw_noise_deg)
        self.velocity_noise_std = float(
            self.get_parameter("velocity_noise_std").value
        )
        self.angular_velocity_noise_std = float(
            self.get_parameter("angular_velocity_noise_std").value
        )
        self.use_ground_truth_stamp = bool(
            self.get_parameter("use_ground_truth_stamp").value
        )
        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed)

        self.latest_ground_truth = None
        self.warned_waiting = False

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self._on_ground_truth, 50
        )
        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._publish_ins)

        self.get_logger().info(
            "ins_simulator started: %s -> %s at %.1f Hz, seed=%d"
            % (self.input_topic, self.output_topic, self.publish_rate, seed)
        )

    def _noise(self, stddev):
        if stddev <= 0.0:
            return 0.0
        return self.rng.gauss(0.0, stddev)

    def _on_ground_truth(self, msg):
        self.latest_ground_truth = msg
        self.warned_waiting = False

    def _publish_ins(self):
        gt = self.latest_ground_truth
        if gt is None:
            if not self.warned_waiting:
                self.get_logger().warn(
                    "waiting for ground truth on %s" % self.input_topic
                )
                self.warned_waiting = True
            return

        ins = Odometry()
        if self.use_ground_truth_stamp:
            ins.header.stamp = gt.header.stamp
        else:
            ins.header.stamp = self.get_clock().now().to_msg()
        ins.header.frame_id = self.frame_id
        ins.child_frame_id = self.child_frame_id

        pos = gt.pose.pose.position
        ins.pose.pose.position.x = _finite_or(pos.x) + self._noise(
            self.position_noise_std
        )
        ins.pose.pose.position.y = _finite_or(pos.y) + self._noise(
            self.position_noise_std
        )
        ins.pose.pose.position.z = _finite_or(pos.z) + self._noise(
            self.z_noise_std
        )

        yaw = _yaw_from_quaternion(gt.pose.pose.orientation)
        _write_yaw_quaternion(
            ins.pose.pose.orientation,
            yaw + self._noise(self.yaw_noise_std),
        )

        linear = gt.twist.twist.linear
        angular = gt.twist.twist.angular
        ins.twist.twist.linear.x = _finite_or(linear.x) + self._noise(
            self.velocity_noise_std
        )
        ins.twist.twist.linear.y = _finite_or(linear.y)
        ins.twist.twist.linear.z = _finite_or(linear.z)
        ins.twist.twist.angular.x = _finite_or(angular.x)
        ins.twist.twist.angular.y = _finite_or(angular.y)
        ins.twist.twist.angular.z = _finite_or(angular.z) + self._noise(
            self.angular_velocity_noise_std
        )

        self._fill_covariance(ins)
        self.publisher.publish(ins)

    def _fill_covariance(self, msg):
        pos_var = self.position_noise_std * self.position_noise_std
        z_var = max(self.z_noise_std * self.z_noise_std, 1e-6)
        yaw_var = self.yaw_noise_std * self.yaw_noise_std
        vel_var = self.velocity_noise_std * self.velocity_noise_std
        yaw_rate_var = max(
            self.angular_velocity_noise_std
            * self.angular_velocity_noise_std,
            1e-6,
        )

        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = pos_var
        msg.pose.covariance[7] = pos_var
        msg.pose.covariance[14] = z_var
        msg.pose.covariance[21] = 1e-6
        msg.pose.covariance[28] = 1e-6
        msg.pose.covariance[35] = yaw_var

        msg.twist.covariance = [0.0] * 36
        msg.twist.covariance[0] = vel_var
        msg.twist.covariance[7] = vel_var
        msg.twist.covariance[14] = vel_var
        msg.twist.covariance[21] = 1e-6
        msg.twist.covariance[28] = 1e-6
        msg.twist.covariance[35] = yaw_rate_var


def main(args=None):
    rclpy.init(args=args)
    node = INSSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ins_simulator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
