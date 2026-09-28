import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RobotMover(Node):

    def __init__(self):
        super().__init__('robot_mover')

        # Publisher for movement
        self.publisher = self.create_publisher(
            Twist,
            '/robot_1/cmd_vel',
            10
        )

        # IR sensor QoS
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscriber for IR sensors
        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/robot_1/ir_intensity',
            self.ir_callback,
            qos_profile
        )

        self.obstacle_detected = False
        self.turn_direction = 0.5

        # Check movement every 0.1 seconds
        self.timer = self.create_timer(
            0.1,
            self.move
        )

    def ir_callback(self, msg):

        # Get sensor values
        readings = {
            reading.header.frame_id: reading.value
            for reading in msg.readings
        }
        print("IR readings",readings)
        # Get front sensor values
        front_left = 0
        front_right = 0
        front_center_left = 0
        front_center_right = 0

        for frame, value in readings.items():

            if 'front_left' in frame:
                front_left = value

            elif 'front_right' in frame:
                front_right = value

            elif 'front_center_left' in frame:
                front_center_left = value

            elif 'front_center_right' in frame:
                front_center_right = value

        # Obstacle threshold
        threshold = 300

        # Detect obstacle in front
        if (
            front_left > threshold
            or front_right > threshold
            or front_center_left > threshold
            or front_center_right > threshold
        ):

            self.obstacle_detected = True

            # Turn away from the stronger side
            if front_left > front_right:
                self.turn_direction = -0.5
            else:
                self.turn_direction = 0.5

        else:
            self.obstacle_detected = False

    def move(self):

        msg = Twist()

        if self.obstacle_detected:

            # Obstacle detected → turn
            msg.linear.x = 0.0
            msg.angular.z = self.turn_direction

        else:

            # No obstacle → move forward
            msg.linear.x = 0.1
            msg.angular.z = 0.0

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = RobotMover()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
