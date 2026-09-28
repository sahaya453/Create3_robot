#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data


class MazeController(Node):

    def __init__(self):
        super().__init__('maze_controller')

        # FSM state
        self.current_state = 'FOLLOW_WALL'
        self.threshold = 50
        # Latest sensor values
        self.ir_values = {}
        self.ir_ready = False
        # Publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot_1/cmd_vel',
            10
        )

        self.ir_subscription = self.create_subscription(IrIntensityVector,
                                                        '/robot_1/ir_intensity',
                                                        self.ir_callback,
                                                        qos_profile=qos_profile_sensor_data )

        # Control loop
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )
    

    #def follow_wall(self):
        #d_desired = 0.2
        #kp = 2
        #u_ref = 0.20
        #error = self.left_distance - d_desired
        #w_ref = kp * error
        #return u_ref, w_ref

    def ir_callback(self, msg):
        for reading in msg.readings:
            sensor_name = reading.header.frame_id
            sensor_value = reading.value
            self.ir_values[sensor_name] = sensor_value
        self.ir_ready = True

    def timer_callback(self):
        if not self.ir_ready:
            self.stop_robot()
            return
        side_left_value = self.ir_values.get("ir_intensity_side_left", 0)
        left_value = self.ir_values.get("ir_intensity_left", 0)
        front_left_value = self.ir_values.get("ir_intensity_front_left", 0)
        front_center_value = self.ir_values.get("ir_intensity_front_center_left", 0)
        front_right_value = self.ir_values.get("ir_intensity_front_center_right", 0)
        right_value = self.ir_values.get("ir_intensity_front_right", 0)
        side_right_value = self.ir_values.get("ir_intensity_right", 0)


        front_block = front_center_value > self.threshold
        front_left_block = front_left_value > self.threshold
        left_wall_present = side_left_value > self.threshold or left_value > self.threshold
        left_open = not left_wall_present
      


        if self.current_state == 'FOLLOW_WALL':
            u_ref = 0.5
            w_ref = 0.0
            if front_block:
                self.current_state = 'TURN_RIGHT'
            elif left_open:
                self.current_state = 'TURN_LEFT'

        elif self.current_state == 'TURN_RIGHT':
            u_ref = 0.0
            w_ref = -0.20
            if not (front_left_block and front_block) :
                self.current_state = "FOLLOW_WALL"

        elif self.current_state == 'TURN_LEFT':
            u_ref = 0.05
            w_ref = 0.02
            if left_wall_present:
                self.current_state = "FOLLOW_WALL"
                

        elif self.current_state == 'STOP':
            u_ref = 0.0
            w_ref = 0.0
            
        self.publish_velocity(u_ref, w_ref)
        

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def publish_velocity(self, u_ref, w_ref):
        msg = Twist()
        msg.linear.x = u_ref
        msg.angular.z = w_ref
        self.cmd_vel_publisher.publish(msg)

def main(args=None):

    rclpy.init(args=args)

    controller = MazeController()

    rclpy.spin(controller)

    #controller.destroy_node()

    #rclpy.shutdown()

if __name__ == '__main__':
    main()
