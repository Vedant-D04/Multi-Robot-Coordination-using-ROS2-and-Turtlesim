from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start the Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Start the gesture recognition node
        Node(
            package='gesture_recognition',
            executable='gesture_detector',
            name='gesture_detector',
            output='screen'
        ),
        
        # Start the robot controller node
        Node(
            package='robot_coordination',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),
    ])