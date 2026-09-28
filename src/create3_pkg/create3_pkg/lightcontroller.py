#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from rclpy.qos import ReliabilityPolicy, QoSProfile

class lightController(Node):

    def __init__(self):
        super().__init__("lightController")
        
        #Publish to the cmd_lightring topic, which uses messages with type LightringLeds
        self.lightringPublisher  = self.create_publisher(LightringLeds,"cmd_lightring",10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        #Initilaize message to correct message type
        msg = LightringLeds()
        msg.override_system = True #To override the default lightring settings
        blue = LedColor()
        blue.red = 0
        blue.green = 0
        blue.blue = 255

        msg.leds = [blue, blue, blue, blue, blue, blue]

        self.lightringPublisher.publish(msg)

        print("Publishing...")
      

def main():
    rclpy.init()
    controller = lightController()
    rclpy.spin(controller)

if __name__ == '__main__':
    main()