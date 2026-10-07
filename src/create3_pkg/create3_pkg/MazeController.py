#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import IrIntensityVector


class MazeController(Node):

    def __init__(self):

        super().__init__("maze_controller")

        self.current_state = "FOLLOW_PATH"

        # IR_sensor
        self.ir_values = {}
        self.ir_ready = False
        self.ir_threshold = 120
        self.ir_avoid_speed = 0.04
        self.ir_steer_correction = 0.15
        self.ir_front_sensors = ("ir_intensity_front_center_left")
        self.ir_front_left_sensors = ("ir_intensity_front_left", "ir_intensity_left")
        self.ir_front_right_sensors = ("ir_intensity_front_center_right", "ir_intensity_front_right")
 
        #LIDAR VALUES
        self.lidar_ready = False
        self.front_lidar_distance = math.inf
        self.left_lidar_distance = math.inf
        self.right_lidar_distance = math.inf
        self.lidar_block_limit = 0.20
        self.front_block_confirm_scans = 2
        self.front_block_counter = 0

        #ADAPTIVE SIDE-WALL TRACKING
        self.side_open_factor = 1.4
        self.wall_reappear_factor = 0.5
        self.side_change_confirm_scans = 3
        self.wall_update_alpha = 0.05
        self.wall_ref_update_factor = 1.15
        self.initial_two_wall_ratio = 1.8
        self.side_trackers = {"left": self.make_side_tracker(), "right": self.make_side_tracker()}
        self.side_tracking_initialized = False

        #JUNCTION MEMORY
        self.branch_angle_tolerance = math.radians(15)
        self.junctions = []
        self.junction_position_tolerance = 0.20

        #ODOMETRY
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.path_heading = None
        self.target_yaw = None
        self.turn_angle_tolerance = math.radians(5)

        #LEAVING A JUNCTION
        self.move_start_x = None
        self.move_start_y = None
        self.branch_entry_distance = 0.15

        #GOAL POSITION
        self.goal_x = None
        self.goal_y = None
        self.goal_tolerance = 0.05

        #ROS PUBLISHER
        self.cmd_vel_publisher = self.create_publisher(Twist, "/robot_1/cmd_vel", 10)

        #ROS SUBSCRIBERS
        self.ir_subscription = self.create_subscription(IrIntensityVector, "/robot_1/ir_intensity", self.ir_callback, qos_profile=qos_profile_sensor_data)
        self.lidar_subscription = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile=qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(Odometry, "/robot_1/odom", self.odom_callback, qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)

    #BASIC MOVEMENT COMMANDS
    def turn_left(self):
        return 0.0, 0.2

    def turn_right(self):
        return 0.0, -0.2

    def turn_around(self):
        return 0.0, 0.2

    def stop(self):
        return 0.0, 0.0
    
    # IR CALLBACK
    def ir_callback(self, msg):

        for reading in msg.readings:
            sensor_name = reading.header.frame_id
            sensor_value = reading.value
            self.ir_values[sensor_name] = sensor_value

        self.ir_ready = True

    def get_ir_group_max(self, sensor_names):

        values = [self.ir_values[name] for name in sensor_names if name in self.ir_values]
        if len(values) == 0:
            return 0

        return max(values)

    def get_ir_obstacle_status(self):

        if not self.ir_ready:
            return False, False, False, 0, 0, 0
        front_value = self.get_ir_group_max(self.ir_front_sensors)
        front_left_value = self.get_ir_group_max(self.ir_front_left_sensors)
        front_right_value = self.get_ir_group_max(self.ir_front_right_sensors)
        front_left_blocked = (front_left_value >= self.ir_threshold)
        front_right_blocked = (front_right_value >= self.ir_threshold)
        front_blocked = (front_value >= self.ir_threshold or (front_left_blocked and front_right_blocked))

        return (front_blocked, front_left_blocked, front_right_blocked, front_value, front_left_value, front_right_value,)

    def apply_ir_forward_steering(self, u_ref, w_ref):

        if not self.ir_ready or u_ref <= 0.0:
            return u_ref, w_ref

        (front_blocked, front_left_blocked, front_right_blocked, front_value, front_left_value, front_right_value) = self.get_ir_obstacle_status()

        if front_blocked:
            return 0.0, 0.0

        if front_left_blocked and not front_right_blocked:
            return min(u_ref, self.ir_avoid_speed), -self.ir_steer_correction

        if front_right_blocked and not front_left_blocked:
            return min(u_ref, self.ir_avoid_speed), self.ir_steer_correction

        return u_ref, w_ref

    # LIDAR
    def lidar_callback(self, msg):
        self.front_lidar_distance = self.get_lidar_region(msg, math.radians(-15), math.radians(15), use_min=True)
        self.left_lidar_distance = self.get_lidar_region(msg, math.radians(87), math.radians(93), use_min=False)
        self.right_lidar_distance = self.get_lidar_region(msg, math.radians(-93), math.radians(-87), use_min=False)

        self.lidar_ready = True

    def get_lidar_region(self, msg, start_angle, end_angle, use_min=True):

        values = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if start_angle <= angle <= end_angle:

                if msg.range_min <= distance <= msg.range_max:
                    values.append(distance)

        if len(values) == 0:
            return math.inf

        if use_min:
            return float(min(values))

        else:
            return float(np.median(values))

    # FRONT-BLOCK CONFIRMATION
    def front_is_blocked(self):

        raw_block = self.front_lidar_distance < self.lidar_block_limit

        if raw_block:
            self.front_block_counter += 1
        else:
            self.front_block_counter = 0

        return self.front_block_counter >= self.front_block_confirm_scans

    # SIDE-STATE TRACKING
    def make_side_tracker(self):

        return {"state": "UNKNOWN", "wall_ref": None, "open_ref": None, "candidate": None, "candidate_count": 0}

    def reset_side_tracking(self):

        self.side_trackers["left"] = self.make_side_tracker()
        self.side_trackers["right"] = self.make_side_tracker()
        self.side_tracking_initialized = False

    def set_side_wall(self, side, distance):

        tracker = self.side_trackers[side]

        tracker["state"] = "WALL"
        tracker["wall_ref"] = distance
        tracker["open_ref"] = None
        tracker["candidate"] = None
        tracker["candidate_count"] = 0

    def set_side_open(self, side, distance):

        tracker = self.side_trackers[side]

        tracker["state"] = "OPEN"
        tracker["wall_ref"] = None
        tracker["open_ref"] = distance
        tracker["candidate"] = None
        tracker["candidate_count"] = 0

    def initialize_side_tracking(self):

        left = self.left_lidar_distance
        right = self.right_lidar_distance


        near_distance = min(left, right)
        far_distance = max(left, right)

        if far_distance <= near_distance * self.initial_two_wall_ratio:

            self.set_side_wall("left", left)
            self.set_side_wall("right", right)

        elif left < right:

            self.set_side_wall("left", left)
            self.set_side_open("right", right)

        else:
                
            self.set_side_open("left", left)
            self.set_side_wall("right", right)

        self.side_tracking_initialized = True

    def confirm_candidate(self, tracker, candidate):

        if tracker["candidate"] == candidate:
            tracker["candidate_count"] += 1

        else:
            tracker["candidate"] = candidate
            tracker["candidate_count"] = 1

        return tracker["candidate_count"] >= self.side_change_confirm_scans

    def clear_candidate(self, tracker):

        tracker["candidate"] = None
        tracker["candidate_count"] = 0

    def update_side_tracker(self, side, distance):

        tracker = self.side_trackers[side]
        opening_started = False
        wall_started = False

        if tracker["state"] == "WALL":

            wall_ref = tracker["wall_ref"]

            looks_open = distance > wall_ref * self.side_open_factor
        
            if looks_open:

                if self.confirm_candidate(tracker, "OPEN"):

                    self.set_side_open(side, distance)
                    opening_started = True

            else:

                self.clear_candidate(tracker)

                if distance < wall_ref * 0.70:

                    tracker["wall_ref"] = distance

                elif distance <= wall_ref * self.wall_ref_update_factor:

                    alpha = self.wall_update_alpha
                    tracker["wall_ref"] = ((1.0 - alpha) * wall_ref + alpha * distance)

        elif tracker["state"] == "OPEN":

            open_ref = tracker["open_ref"]

            if open_ref is None:

                tracker["open_ref"] = distance
                self.clear_candidate(tracker)

                return opening_started, wall_started

            looks_like_wall = distance < open_ref * self.wall_reappear_factor

            if looks_like_wall:

                if self.confirm_candidate(tracker, "WALL"):

                    self.set_side_wall(side, distance)
                    wall_started = True

            else:

                self.clear_candidate(tracker)
                tracker["open_ref"] = max(distance, open_ref * 0.995)

        return opening_started, wall_started

    def update_side_tracking(self):


        if not self.side_tracking_initialized:
            self.initialize_side_tracking()

            return False, False

        left_opening_started, _ = self.update_side_tracker("left", self.left_lidar_distance,)
        right_opening_started, _ = self.update_side_tracker("right", self.right_lidar_distance,)

        return left_opening_started, right_opening_started

    def side_is_open(self, side):

        return self.side_trackers[side]["state"] == "OPEN"

    def side_is_wall(self, side):

        return self.side_trackers[side]["state"] == "WALL"

    #ODOMETRY
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(sin_yaw, cos_yaw)

        self.odom_ready = True

    def normalize_angle(self, angle):

        return math.atan2(math.sin(angle), math.cos(angle))

    def get_goal_heading(self):
        if self.goal_x is None or self.goal_y is None:
            return None
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        return math.atan2(dy, dx)

    #GOAL DETECTION
    def goal_reached(self):

        if self.goal_x is None or self.goal_y is None:
            return False

        dx = self.x - self.goal_x
        dy = self.y - self.goal_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        return distance_to_goal <= self.goal_tolerance

    #JUNCTION

    def get_open_branches(self, front_open, left_open, right_open):

        branches = []
        if front_open:
            branches.append(self.normalize_angle(self.yaw))

        if left_open:
            branches.append(self.normalize_angle(self.yaw + math.pi / 2))

        if right_open:
            branches.append(self.normalize_angle(self.yaw - math.pi / 2))

        branches.append(self.normalize_angle(self.yaw + math.pi))

        return branches

    def count_matching_branches(self, junction, detected_branches):

        matches = 0
        for detected_heading in detected_branches:
            for known_heading in junction["branches"]:
                angle_difference = self.normalize_angle(detected_heading - known_heading)

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
        goal_heading = self.get_goal_heading()
        if goal_heading is None:
            chosen_branch = unvisited_branches[0]
        else:
            chosen_branch = min(unvisited_branches, key=lambda branch: abs(self.normalize_angle(branch - goal_heading)))

        junction["visited"].append(chosen_branch)
        return chosen_branch

    def branch_to_state(self, chosen_branch):

        self.target_yaw = self.normalize_angle(chosen_branch)
        angle_difference = self.normalize_angle(chosen_branch - self.yaw)

        if abs(angle_difference) < math.radians(30):
            return "GO_STRAIGHT"

        if abs(angle_difference) > math.radians(150):
            return "TURN_AROUND"

        if angle_difference > 0:
            return "TURN_LEFT"

        return "TURN_RIGHT"

    def target_heading_reached(self):

        if self.target_yaw is None:
            return False

        angle_error = self.normalize_angle(self.target_yaw - self.yaw)

        return abs(angle_error) < self.turn_angle_tolerance

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

    #FORWARD
    def hold_heading(self, heading, speed=0.05):

        if heading is None:
            heading = self.yaw

        heading_error = self.normalize_angle(heading - self.yaw)
        kp_heading = 1.5
        w_ref = kp_heading * heading_error
        w_ref = max(-0.2, min(0.2, w_ref))

        return speed, w_ref

    def follow_path(self):

        left_tracker = self.side_trackers["left"]
        right_tracker = self.side_trackers["right"]
        left_wall_usable = (left_tracker["state"] == "WALL" and left_tracker["wall_ref"] is not None and left_tracker["candidate"] != "OPEN")
        right_wall_usable = (right_tracker["state"] == "WALL" and right_tracker["wall_ref"] is not None and right_tracker["candidate"] != "OPEN")

        if left_wall_usable and right_wall_usable:
            kp = 1.0
            u_ref = 0.08
            error = (self.left_lidar_distance - self.right_lidar_distance)
            w_ref = kp * error
            w_ref = max(-0.2, min(0.2, w_ref))

            return u_ref, w_ref

        if left_wall_usable:
            kp = 1.0
            u_ref = 0.06
            desired_left = left_tracker["wall_ref"]
            error = self.left_lidar_distance - desired_left
            w_ref = kp * error
            w_ref = max(-0.2, min(0.2, w_ref))

            return u_ref, w_ref
        
        if right_wall_usable:
            kp = 1.0
            u_ref = 0.06
            desired_right = right_tracker["wall_ref"]
            error = desired_right - self.right_lidar_distance
            w_ref = kp * error
            w_ref = max(-0.2, min(0.2, w_ref))

            return u_ref, w_ref

        goal_heading = self.get_goal_heading()
        if goal_heading is not None:
            return self.hold_heading(goal_heading, speed=0.05)

        return self.hold_heading(self.path_heading, speed=0.05)

    # BRANCH-ENTRY HELPER

    def prepare_branch_entry(self, reset_sides=True):

        self.move_start_x = None
        self.move_start_y = None
        if self.target_yaw is not None:
            self.path_heading = self.target_yaw
        else:
            self.path_heading = self.yaw

        if reset_sides:
            self.reset_side_tracking()

        self.current_state = "GO_STRAIGHT"

    #FSM
    def timer_callback(self):

        if not self.lidar_ready or not self.odom_ready:
            self.publish_velocity(0.0, 0.0)
            return

        if self.goal_reached():
            print("GOAL REACHED -> STOP")
            self.current_state = "STOP"

        lidar_front_block = self.front_is_blocked()

        (ir_front_block, *_) = self.get_ir_obstacle_status()

        front_block = lidar_front_block or ir_front_block
        front_open = not front_block

        # FOLLOW_PATH
        if self.current_state == "FOLLOW_PATH":


            if not self.side_tracking_initialized:
                self.initialize_side_tracking()

            if self.path_heading is None:
                self.path_heading = self.yaw

            (left_opening_started, right_opening_started) = self.update_side_tracking()

            left_open = self.side_is_open("left")
            right_open = self.side_is_open("right")
            left_wall = self.side_is_wall("left")
            right_wall = self.side_is_wall("right")

            if front_block:

                if left_open and right_open:
                    print("FRONT BLOCK + BOTH SIDES OPEN -> JUNCTION")
                    self.current_state = "JUNCTION"
                    u_ref, w_ref = self.stop()

                elif left_open and right_wall:
                    print("FRONT BLOCK -> TURN_LEFT")
                    self.target_yaw = self.normalize_angle(self.yaw + math.pi / 2)
                    self.current_state = "TURN_LEFT"
                    u_ref, w_ref = self.stop()

                elif right_open and left_wall:
                    print("FRONT BLOCK -> TURN_RIGHT")
                    self.target_yaw = self.normalize_angle(self.yaw - math.pi / 2)
                    self.current_state = "TURN_RIGHT"
                    u_ref, w_ref = self.stop()

                elif left_wall and right_wall:
                    print("DEAD END -> TURN_AROUND")
                    self.target_yaw = self.normalize_angle(self.yaw + math.pi)
                    self.current_state = "TURN_AROUND"
                    u_ref, w_ref = self.stop()

                else:
                    print("FRONT BLOCKED but side state uncertain -> STOP")
                    u_ref, w_ref = self.stop()

            #FRONT OPEN
            else:

                if left_opening_started or right_opening_started:
                    print("NEW SIDE OPENING -> JUNCTION")
                    self.current_state = "JUNCTION"
                    u_ref, w_ref = self.stop()

                else:
                    u_ref, w_ref = self.follow_path()


        #JUNCTION
        elif self.current_state == "JUNCTION":

            u_ref, w_ref = self.stop()
            left_open = self.side_is_open("left")
            right_open = self.side_is_open("right")
            branches = self.get_open_branches(front_open, left_open, right_open,)

            print("JUNCTION branches =", branches)
            back_heading = self.normalize_angle(self.yaw + math.pi)
            junction = self.find_known_junction(branches)
            if junction is None:
                new_junction = {"x": self.x, "y": self.y, "branches": branches, "visited": [back_heading]}
                self.junctions.append(new_junction)
                junction = new_junction
                print("NEW junction stored")

            else:
                print("KNOWN junction")
                self.update_junction_branches(junction, branches)

                if not self.is_branch_visited(junction, back_heading):
                    junction["visited"].append(back_heading)

            chosen_branch = self.choose_unvisited_branch(junction)
            print("chosen branch =", chosen_branch)

            if chosen_branch is not None:
                next_state = self.branch_to_state(chosen_branch)
                print("next state =", next_state)
                if next_state == "GO_STRAIGHT":
                    self.prepare_branch_entry(reset_sides=False)

                else:
                    self.current_state = next_state
            else:
                print("no unvisited branch -> BACKTRACK")
                self.current_state = "BACKTRACK"

        #TURN_RIGHT
        elif self.current_state == "TURN_RIGHT":

            u_ref, w_ref = self.turn_right()
            if self.target_heading_reached():
                print("TURN_RIGHT complete")
                self.prepare_branch_entry()
                u_ref, w_ref = self.stop()

        #TURN_LEFT
        elif self.current_state == "TURN_LEFT":

            u_ref, w_ref = self.turn_left()
            if self.target_heading_reached():
                print("TURN_LEFT complete")
                self.prepare_branch_entry()
                u_ref, w_ref = self.stop()

        #TURN_AROUND
        elif self.current_state == "TURN_AROUND":

            u_ref, w_ref = self.turn_around()
            if self.target_heading_reached():
                print("TURN_AROUND complete")
                self.prepare_branch_entry()
                u_ref, w_ref = self.stop()

        #BACKTRACK
        elif self.current_state == "BACKTRACK":

            u_ref, w_ref = self.stop()
            self.target_yaw = self.normalize_angle(self.yaw + math.pi)
            self.current_state = "TURN_AROUND"

        #GO_STRAIGHT
        elif self.current_state == "GO_STRAIGHT":

            if self.move_start_x is None:
                self.move_start_x = self.x
                self.move_start_y = self.y

            if self.side_tracking_initialized:
                self.update_side_tracking()
            u_ref, w_ref = self.hold_heading(self.path_heading,speed=0.05)

            dx = self.x - self.move_start_x
            dy = self.y - self.move_start_y
            distance_moved = math.sqrt(dx**2 + dy**2)

            if front_block:

                print("FRONT BLOCK during branch entry -> FOLLOW_PATH")

                if not self.side_tracking_initialized:
                    self.initialize_side_tracking()

                self.move_start_x = None
                self.move_start_y = None
                self.target_yaw = None
                self.current_state = "FOLLOW_PATH"
                u_ref, w_ref = self.stop()

            elif distance_moved >= self.branch_entry_distance:

                print("branch entry complete -> FOLLOW_PATH")

                if not self.side_tracking_initialized:
                    self.initialize_side_tracking()

                self.move_start_x = None
                self.move_start_y = None
                self.target_yaw = None
                self.current_state = "FOLLOW_PATH"

        #STOP
        elif self.current_state == "STOP":
            u_ref, w_ref = self.stop()
        else:
            print("UNKNOWN STATE:", self.current_state)
            u_ref, w_ref = self.stop()

        u_ref, w_ref = self.apply_ir_forward_steering(u_ref,w_ref)

        self.publish_velocity(u_ref, w_ref)

    # ================================================================

    # PUBLISHING / SHUTDOWN

    # ================================================================

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

if __name__ == "__main__":

    main()
