from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg_desc = get_package_share_directory('igvc_description')
    urdf_file = os.path.join(pkg_desc, 'urdf', 'robot.xacro')

    # Proper xacro processing (NO os.popen)
    robot_desc = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        # Start Gazebo Harmonic
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'igvc_bot',
                '-z', '0.0'
            ],
            output='screen'
        ),

        # Publish joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            output='screen'
        )
    ])

