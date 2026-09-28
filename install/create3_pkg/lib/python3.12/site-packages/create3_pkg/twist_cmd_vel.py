import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotMover(Node):

    def __init__(self):
        super().__init__('robot_mover')

        self.publisher = self.create_publisher(
            Twist,
            '/robot_1/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.move)

        self.counter = 0

    def move(self):
        msg = Twist()

  #      if self.counter < 30:
            # Move forward
        msg.linear.x = 0.2
        msg.angular.z = 0.0
   #     else:
            # Stop
    #        msg.linear.x = 0.0
     #       msg.angular.z = 0.0

        self.publisher.publish(msg)
      #  self.counter += 1


def main():
    rclpy.init()

    node = RobotMover()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
