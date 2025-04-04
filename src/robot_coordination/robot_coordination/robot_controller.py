#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
import random
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Parameters
        self.num_robots = 4
        self.simulation_width = 11.0
        self.simulation_height = 11.0
        self.robot_poses = {}
        self.robot_statuses = {}
        self.robot_goals = {}
        self.robot_publishers = {}
        self.robot_subscribers = {}
        self.obstacles = {}  # Store obstacles as a dictionary with more details
        self.obstacle_removal_threshold = 0.5  # Reduced threshold for easier collision
        self.current_task = None

        # Service clients
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        # Subscribe to gesture commands
        self.gesture_subscription = self.create_subscription(
            String,
            '/gesture_commands',
            self.gesture_callback,
            10)

        # Initialize turtles
        self.get_logger().info('Initializing turtles...')
        self.init_turtles()
        self.create_obstacles()

        # Start coordination timer
        self.coordination_timer = self.create_timer(0.5, self.coordinate_robots)

        self.get_logger().info('Robot controller initialized')

    def init_turtles(self):
        # Wait for the spawn service
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # Kill default turtle
        kill_req = Kill.Request()
        kill_req.name = 'turtle1'
        self.kill_client.call_async(kill_req)

        # Spawn robots
        for i in range(self.num_robots):
            x = random.uniform(1.0, self.simulation_width - 1.0)
            y = random.uniform(1.0, self.simulation_height - 1.0)
            theta = random.uniform(0, 2 * math.pi)
            name = f'turtle{i+1}'

            spawn_req = Spawn.Request()
            spawn_req.x = x
            spawn_req.y = y
            spawn_req.theta = theta
            spawn_req.name = name

            future = self.spawn_client.call_async(spawn_req)
            rclpy.spin_until_future_complete(self, future)

            # Setup publishers & subscribers
            self.robot_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.robot_subscribers[name] = self.create_subscription(Pose, f'/{name}/pose',
                                                                    lambda msg, n=name: self.pose_callback(msg, n), 10)
            self.robot_statuses[name] = 'idle'
            self.robot_poses[name] = None
            self.robot_goals[name] = None

            self.get_logger().info(f'Spawned {name} at ({x}, {y}, {theta})')

    def create_obstacles(self):
        # Spawn stationary turtles as obstacles
        for i in range(2, 4):  # Adjust to match obstacle2, obstacle3
            x = random.uniform(2.0, self.simulation_width - 2.0)
            y = random.uniform(2.0, self.simulation_height - 2.0)
            name = f'obstacle{i}'

            spawn_req = Spawn.Request()
            spawn_req.x = x
            spawn_req.y = y
            spawn_req.theta = 0.0
            spawn_req.name = name

            future = self.spawn_client.call_async(spawn_req)
            rclpy.spin_until_future_complete(self, future)

            # Store more detailed obstacle information
            self.obstacles[name] = {
                'x': x, 
                'y': y, 
                'status': 'active'
            }
            self.get_logger().info(f'Created obstacle {name} at ({x}, {y})')

            self.set_pen_color(name, 255, 0, 0)  # Red color for obstacles

    def set_pen_color(self, turtle_name, r, g, b):
        client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {turtle_name}/set_pen service...')

        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = 3
        req.off = 0

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def pose_callback(self, msg, turtle_name):
        self.robot_poses[turtle_name] = msg

    def gesture_callback(self, msg):
        gesture = msg.data
        self.get_logger().info(f'Received gesture command: {gesture}')

        if gesture == 'cleaning':
            self.current_task = 'cleaning'
            self.start_cleaning()
        elif gesture == 'stop_reset':
            self.current_task = None
            self.stop_all_robots()

    def start_cleaning(self):
        self.get_logger().info('Starting Automated Cleaning mission')

        # Distribute robots to move across different areas
        section_width = self.simulation_width / self.num_robots
        
        for i, name in enumerate(self.robot_publishers.keys()):
            if name.startswith('turtle'):
                # Calculate section for this robot
                section_start_x = i * section_width
                section_end_x = (i + 1) * section_width
                
                x = random.uniform(section_start_x, section_end_x)
                y = random.uniform(1.0, self.simulation_height - 1.0)

                self.robot_goals[name] = (x, y)
                self.robot_statuses[name] = 'cleaning'

                self.get_logger().info(f'Assigned {name} to move to ({x}, {y})')

    def stop_all_robots(self):
        self.get_logger().info('Stopping all robots')

        for name, publisher in self.robot_publishers.items():
            twist = Twist()
            publisher.publish(twist)

            self.robot_statuses[name] = 'idle'
            self.robot_goals[name] = None

            self.get_logger().info(f'Stopped {name}')

    def remove_obstacle(self, obstacle_name):
        # Check if obstacle exists
        if obstacle_name not in self.obstacles:
            return False

        self.get_logger().info(f'Removing obstacle: {obstacle_name}')

        # Remove turtle representing the obstacle
        kill_req = Kill.Request()
        kill_req.name = obstacle_name
        future = self.kill_client.call_async(kill_req)
        rclpy.spin_until_future_complete(self, future)

        # Remove from obstacles dictionary
        del self.obstacles[obstacle_name]
        
        self.get_logger().info(f'Obstacle {obstacle_name} successfully removed.')
        return True

    def coordinate_robots(self):
        if self.current_task != 'cleaning':
            return

        for name, publisher in self.robot_publishers.items():
            if not name.startswith('turtle') or self.robot_poses.get(name) is None:
                continue

            pose = self.robot_poses[name]
            goal = self.robot_goals.get(name)

            if goal is None:
                continue

            # Check for collision with stationary obstacles
            for obs_name, obs_data in list(self.obstacles.items()):
                obs_dx = obs_data['x'] - pose.x
                obs_dy = obs_data['y'] - pose.y
                obs_distance = math.sqrt(obs_dx * obs_dx + obs_dy * obs_dy)

                # If moving turtle is very close to stationary obstacle, remove the obstacle
                if obs_distance < self.obstacle_removal_threshold:
                    self.remove_obstacle(obs_name)
                    break

            # Navigate towards goal
            dx = goal[0] - pose.x
            dy = goal[1] - pose.y
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_diff = angle_to_goal - pose.theta

            # Normalize angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            twist = Twist()

            if distance > 0.1:
                # Move towards goal
                if abs(angle_diff) > 0.1:
                    twist.angular.z = 0.6 * angle_diff
                else:
                    twist.linear.x = 0.5
            else:
                # Goal reached
                self.robot_goals[name] = None
                self.robot_statuses[name] = 'idle'

            publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()