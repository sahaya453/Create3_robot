#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class MazeController(Node):
    def __init__(self):
        super().__init__('maze_controller')

        # TurtleBot 4 / Create 3 requires cmd_vel_stamped
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/robot_1/cmd_vel_stamped',
            10
        )

        # LiDAR uses BEST_EFFORT QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos
        )

        # Movement parameters
        self.linear_speed = 0.05
        self.turn_speed = 0.15
        self.safe_distance = 0.30

        # Regions
        self.regions = {
            "front": float('inf'),
            "left": float('inf'),
            "right": float('inf')
        }

        self.get_logger().info("MazeController running with REAL LiDAR input.")

    def lidar_callback(self, msg):
        self.get_logger().info("LiDAR callback triggered")

        ranges = [r if r > 0.01 else float('inf') for r in msg.ranges]

        # Segment LiDAR into regions
        self.regions["front"] = min(min(ranges[0:20]), min(ranges[-20:]))
        self.regions["left"] = min(ranges[60:100])
        self.regions["right"] = min(ranges[-100:-60])

        self.navigate()

    def navigate(self):
        cmd = TwistStamped()

        front = self.regions["front"]
        left = self.regions["left"]
        right = self.regions["right"]

        if front < self.safe_distance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.turn_speed
            self.get_logger().info("Obstacle ahead → turning left")

        elif left < self.safe_distance:
            cmd.twist.linear.x = self.linear_speed
            cmd.twist.angular.z = -0.25
            self.get_logger().info("Wall on left → steering right")

        elif right < self.safe_distance:
            cmd.twist.linear.x = self.linear_speed
            cmd.twist.angular.z = 0.25
            self.get_logger().info("Wall on right → steering left")

        else:
            cmd.twist.linear.x = self.linear_speed
            cmd.twist.angular.z = 0.0
            self.get_logger().info("Path clear → moving forward")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MazeController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
