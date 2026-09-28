import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds, LedColor
from irobot_create_msgs.action import LedAnimation
from rclpy.qos import ReliabilityPolicy, QoSProfile

class animationController(Node):
    def __init__(self):
        super().__init__("animationController")

        self.buttonSubscriber = self.create_subscription(InterfaceButtons, "/robot_1/interface_buttons", self.button_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.action_client = ActionClient(self, LedAnimation, "/robot_1/led_animation")


    def send_goal(self):
        animationGoal = LedAnimation.Goal()
        animationGoal.animation_type = 1
        animationGoal.max_runtime.sec = 15
        animationGoal.lightring = LightringLeds()
        animationGoal.lightring.override_system = True
        blueLed = LedColor(red = 0, green = 0, blue = 255)
        greenLed = LedColor(red = 0, green = 255, blue = 0)
        redLed = LedColor(red = 255, green = 0, blue = 0)
        offLed = LedColor()
        animationGoal.lightring.leds = [blueLed, blueLed, blueLed, blueLed, blueLed, blueLed]
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(animationGoal)

    def button_callback(self, msg):
            print("button message received")
            if msg.button_1.is_pressed:
                print("button 1 is pressed")
                self.send_goal()

def main():
    rclpy.init()
    controller = animationController()
    rclpy.spin(controller)

if __name__ == "__main__":
    main()
