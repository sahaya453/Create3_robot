#!/usr/bin/env python3
from statistics import mean

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
#from irobot_create_msgs.msg import IrIntensityVector
import math
import numpy as np
from geometry_msgs.msg import Twist


class MazeController(Node):

    def __init__(self):
        super().__init__('maze_controller')





        self.sensor_test_mode = False
        self.turn_test_mode = False
        self.straight_test_mode = False
        self.wall_test_mode = False
        self.junction_test_mode = False
        self.print_counter = 0




        # FSM state
        self.current_state = 'FOLLOW_WALL'
        self.threshold = 50

        # Latest sensor values
        self.ir_values = {}
        self.ir_ready = False

        # Lidar values
        self.lidar_block_limit = 0.30
        self.lidar_open_limit = 0.60
        self.open_ratio_limit = 0.40
        self.lidar_ready = False
        self.front_lidar_distance = None
        self.left_lidar_distance = None
        self.right_lidar_distance = None
        self.left_open_ratio = None
        self.right_open_ratio = None
      
        self.branch_angle_tolerance = math.radians(15)


        # Publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot_1/cmd_vel',
            10
        )

        #subscriber
        #self.ir_subscription = self.create_subscription(IrIntensityVector,
                                                        #'/robot_1/ir_intensity',
                                                        #self.ir_callback,
                                                        #qos_profile=qos_profile_sensor_data)

        self.lidar_subscription = self.create_subscription(LaserScan, "/scan",
                                                       self.lidar_callback,
                                                       qos_profile=qos_profile_sensor_data)

        self.odom_subscription = self.create_subscription(Odometry, "/robot_1/odom", self.odom_callback, qos_profile=qos_profile_sensor_data)

        # Control loop
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.move_start_x = None
        self.move_start_y = None
        self.junction_clear_distance = 0.30

        self.junctions = []
        self.junction_position_tolerance = 0.20

        self.target_yaw = None
        self.turn_angle_tolerance = math.radians(5)

    def go_straight(self):
        u_ref = 0.05
        w_ref = 0.0
        return u_ref, w_ref
    def turn_left(self):
        u_ref = 0.0
        w_ref = 0.2
        return u_ref, w_ref
    def turn_right(self):
        u_ref = 0.0
        w_ref = -0.2
        return u_ref, w_ref
    def turn_around(self):
        u_ref = 0.0
        w_ref = 0.2
        return u_ref, w_ref
    def stop(self):
        u_ref = 0.0
        w_ref = 0.0
        return u_ref, w_ref



    def ir_callback(self, msg):
        for reading in msg.readings:
            sensor_name = reading.header.frame_id
            sensor_value = reading.value
            self.ir_values[sensor_name] = sensor_value
        self.ir_ready = True

    

    def lidar_callback(self, msg):

        self.front_lidar_distance = self.get_lidar_region(msg, math.radians(-15), math.radians(15), use_min= True)
        self.left_open_ratio = self.get_side_open_ratio(msg, math.radians(30), math.radians(80))
        self.right_open_ratio = self.get_side_open_ratio(msg, math.radians(-80), math.radians(-30))
        ## for follow wall, we need to get the left distance to the wall
        self.left_lidar_distance = self.get_lidar_region(msg, math.radians(80), math.radians(100), use_min= False)
        self.right_lidar_distance = self.get_lidar_region(msg, math.radians(-100), math.radians(-80), use_min= False)

        self.lidar_ready = True

    def get_lidar_region(self, msg, start_angle, end_angle, use_min = True):
            
            values = []
            for i, distance in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                if start_angle <= angle <= end_angle:
                    if msg.range_min <= distance <= msg.range_max:
                        values.append(distance)
            if len(values) == 0:
                return None

            if use_min:
                return float(min(values))
            else:
                return float(np.median(values))

    def get_side_open_ratio(self, msg, start_angle, end_angle):

        total = 0
        open_count = 0
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if start_angle <= angle <= end_angle:
                if math.isnan(distance):
                    continue
                if math.isinf(distance):
                    total += 1
                    open_count += 1
                    continue
                if msg.range_min <= distance <= msg.range_max:
                    total += 1
                    if distance > self.lidar_open_limit:
                        open_count += 1
        if total == 0:
            return None 

        return open_count / total
                        
                        

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(sin_yaw, cos_yaw)

        self.odom_ready = True

    def normalize_angle(self, angle):

        return math.atan2(math.sin(angle), math.cos(angle))


    def get_open_branches(self, front_open, left_open, right_open):

        branches = []

        if front_open:
            straight_heading = self.normalize_angle(self.yaw)
            branches.append(straight_heading)

        if left_open:
            left_heading = self.normalize_angle(self.yaw + math.pi / 2)
            branches.append(left_heading)

        if right_open:
            right_heading = self.normalize_angle(self.yaw - math.pi / 2)
            branches.append(right_heading)

        back_heading = self.normalize_angle(self.yaw + math.pi)
        branches.append(back_heading)

        return branches


    def count_matching_branches(self, junction, detected_branches):

        matches = 0

        for detected_heading in detected_branches:

            for known_heading in junction["branches"]:

                angle_difference = self.normalize_angle(
                    detected_heading - known_heading
                )

                if abs(angle_difference) < self.branch_angle_tolerance:
                    matches += 1
                    break

        return matches

    
    def find_known_junction(self, detected_branches):

        for junction in self.junctions:

            dx = self.x - junction["x"]
            dy = self.y - junction["y"]

            distance = math.sqrt(dx**2 + dy**2)
            if distance < self.junction_position_tolerance:
                matches = self.count_matching_branches(junction, detected_branches)    
                if matches >= 2:
                    return junction
                
        return None

    def is_branch_visited(self, junction, branch_heading):

        for visited_heading in junction["visited"]:
            angle_difference = self.normalize_angle(branch_heading - visited_heading)
            if abs(angle_difference) < self.branch_angle_tolerance:
                return True
        return False


    def get_unvisited_branches(self, junction):

        unvisited_branches = []
        for branch_heading in junction["branches"]:
            if not self.is_branch_visited(junction, branch_heading):
                unvisited_branches.append(branch_heading)
        return unvisited_branches

    def choose_unvisited_branch(self, junction):

        unvisited_branches = self.get_unvisited_branches(junction)
        
        if len(unvisited_branches) == 0:
            return None

        chosen_branch = unvisited_branches[0]

        junction["visited"].append(chosen_branch)

        return chosen_branch

    def branch_to_state(self, chosen_branch):

        self.target_yaw = self.normalize_angle(chosen_branch)

        angle_difference = self.normalize_angle(chosen_branch - self.yaw)

        if abs(angle_difference) < math.radians(30):
            return "GO_STRAIGHT"
        elif abs(angle_difference) > math.radians(150):
            return "TURN_AROUND"
        elif angle_difference > 0:
            return "TURN_LEFT"
        else:
            return "TURN_RIGHT"

    def target_heading_reached(self):

        if self.target_yaw is None:
            return False

        angle_error = self.normalize_angle(self.target_yaw - self.yaw)
        return abs(angle_error) < self.turn_angle_tolerance
    
# check whether all the branches of a junction are known, if not, add the new branch to the junction's branches list
    def is_branch_known(self, junction, branch_heading):

        for known_heading in junction["branches"]:
            angle_difference = self.normalize_angle(branch_heading - known_heading)
            if abs(angle_difference) < self.branch_angle_tolerance:
                return True 
        return False

    def update_junction_branches(self, junction, branches):

        for branch_heading in branches:
            if not self.is_branch_known(junction, branch_heading):
                junction["branches"].append(branch_heading)
###############

    def follow_wall(self, left_distance):

        desired_distance = 0.35
        kp = 1.5
        u_ref = 0.08
        error = left_distance - desired_distance
        w_ref = error * kp
        w_ref = max(-0.3, min(0.3, w_ref))
        print(
        "FOLLOW_WALL:",
        "left =", left_distance,
        "error =", error,
        "w =", w_ref
            )

        return u_ref, w_ref

    def follow_corridor(self, left_distance, right_distance):

        kp = 1.0
        u_ref = 0.08

        error = left_distance - right_distance

        w_ref = kp * error

        w_ref = max(-0.2, min(0.2, w_ref))

        return u_ref, w_ref
        
    
    def timer_callback(self):


        if self.junction_test_mode:

            self.publish_velocity(0.0, 0.0)

            if not self.lidar_ready:
                print("Waiting for lidar")
                return

            if (self.front_lidar_distance is None or
                self.left_open_ratio is None or
                self.right_open_ratio is None or
                self.right_lidar_distance is None or
                self.left_lidar_distance is None):
                print("Invalid lidar values")
                return

            front_block = (
                self.front_lidar_distance < self.lidar_block_limit
            )

            front_open = not front_block

            left_open = (
                self.left_open_ratio > self.open_ratio_limit
            )

            right_open = (
                self.right_open_ratio > self.open_ratio_limit
            )

            junction_detected = (
                (front_open and left_open)
                or
                (front_open and right_open)
                or
                (left_open and right_open)
            )

            dead_end = (
                front_block
                and not left_open
                and not right_open
            )

            self.print_counter += 1

            if self.print_counter >= 10:

                print(
                    "front =", round(self.front_lidar_distance, 3),
                    "| left_ratio =", round(self.left_open_ratio, 2),
                    "| right_ratio =", round(self.right_open_ratio, 2),
                    "| front_block =", front_block,
                    "| left_open =", left_open,
                    "| right_open =", right_open,
                    "| junction =", junction_detected,
                    "| dead_end =", dead_end
                )

                self.print_counter = 0

            return



        









        if not self.lidar_ready or not self.odom_ready:
            self.publish_velocity(0.0, 0.0)
            return 
        
        if (self.right_open_ratio is None or
            self.left_open_ratio is None or
            self.front_lidar_distance is None or
            self.left_lidar_distance is None):

            self.publish_velocity(0.0, 0.0)
            return

        front_block = self.front_lidar_distance < self.lidar_block_limit
        front_open = not front_block
        left_open = self.left_open_ratio > self.open_ratio_limit
        right_open = self.right_open_ratio > self.open_ratio_limit
        junction_detected = (front_open and left_open) or (front_open and right_open) or (left_open and right_open)
        dead_end = front_block and not left_open and not right_open


        
        if self.current_state == 'FOLLOW_WALL':

            if dead_end:
            
                print("DEAD END -> TURN_AROUND")

                self.target_yaw = self.normalize_angle(self.yaw + math.pi)
                self.current_state = "TURN_AROUND"
                u_ref, w_ref = self.stop()


            elif junction_detected:
                self.current_state = "JUNCTION"
                u_ref, w_ref = self.stop()


            elif right_open:
                self.target_yaw = self.normalize_angle(self.yaw - math.pi / 2)
                self.current_state = 'TURN_RIGHT'
                u_ref, w_ref = self.stop()


            elif left_open:
                self.target_yaw = self.normalize_angle(self.yaw + math.pi / 2)
                self.current_state = 'TURN_LEFT'
                u_ref, w_ref = self.stop()

            else:

                u_ref, w_ref = self.follow_corridor(self.left_lidar_distance, self.right_lidar_distance)

        elif self.current_state == "JUNCTION":  ## for junction_detected
            u_ref, w_ref = self.stop()
            branches = self.get_open_branches(front_open, left_open, right_open)
            print("JUNCTION detected")
            print("branches =", branches)
            back_heading = self.normalize_angle(self.yaw + math.pi)

            junction = self.find_known_junction(branches)
            if junction is None:
                new_junction = {"x": self.x, "y": self.y, "branches": branches, "visited": [back_heading]}
                self.junctions.append(new_junction)
                junction = new_junction
                print("NEW junction")
            else:
                   
                print("KNOWN junction")
   
                self.update_junction_branches(junction, branches)

                if not self.is_branch_visited(junction, back_heading):
                    junction["visited"].append(back_heading)


            chosen_branch = self.choose_unvisited_branch(junction)
            print("chosen branch =", chosen_branch)

            if chosen_branch is not None:
                    self.current_state = self.branch_to_state(chosen_branch)
                    print(
                    "next state =",
                    self.current_state
                )
            else:
                    self.current_state = "BACKTRACK"
                    print("no unvisited branch -> BACKTRACK")
            

            
        elif self.current_state == 'TURN_RIGHT':
            u_ref, w_ref = self.turn_right()

            if self.target_heading_reached():

                
                self.current_state = "GO_STRAIGHT"

        elif self.current_state == 'TURN_LEFT':
            u_ref, w_ref = self.turn_left()

            if self.target_heading_reached():

                
                self.current_state = "GO_STRAIGHT"

        elif self.current_state == "TURN_AROUND":
            u_ref, w_ref = self.turn_around()

            if self.target_heading_reached():

              
                self.current_state = "GO_STRAIGHT"

        elif self.current_state == "BACKTRACK":
            u_ref, w_ref = self.stop()
            self.target_yaw = self.normalize_angle(self.yaw + math.pi)
            self.current_state = "TURN_AROUND"

        elif self.current_state == "GO_STRAIGHT":
            if self.move_start_x is None:

                self.move_start_x = self.x
                self.move_start_y = self.y
                
            u_ref = 0.05
            if self.target_yaw is not None:
                heading_error = self.normalize_angle(
                self.target_yaw - self.yaw
                ) 
                kp_heading = 1.0
                w_ref = kp_heading * heading_error
                w_ref = max(-0.2, min(0.2, w_ref))
            else:
                w_ref = 0.0

            dx = self.x - self.move_start_x
            dy = self.y - self.move_start_y
            distance_moved = math.sqrt(dx**2 + dy**2)

            if dead_end:
                print("GO_STRAIGHT -> DEAD_END")
                self.move_start_x = None
                self.move_start_y = None
                self.target_yaw = self.normalize_angle(self.yaw + math.pi)
                self.current_state = "TURN_AROUND"

            elif front_block and (left_open or right_open):

                print("GO_STRAIGHT -> JUNCTION")
                self.move_start_x = None
                self.move_start_y = None
                self.target_yaw = None
                self.current_state = "JUNCTION"

            if (distance_moved >= self.junction_clear_distance and not junction_detected):
                print(
                "GO_STRAIGHT -> FOLLOW_WALL",
                "| left =", self.left_lidar_distance,
                "| yaw =", self.yaw,
                "| target_yaw =", self.target_yaw
                )
                self.target_yaw = None
                self.move_start_x = None
                self.move_start_y = None    
                self.current_state = "FOLLOW_WALL"
                



        elif self.current_state == 'STOP':
            u_ref, w_ref = self.stop()


        self.publish_velocity(u_ref, w_ref)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def publish_velocity(self, u_ref, w_ref):
        msg = Twist()
        msg.linear.x = float(u_ref)
        msg.angular.z = float(w_ref)
        self.cmd_vel_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    controller = MazeController()

    try:
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            controller.stop_robot()

        controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
