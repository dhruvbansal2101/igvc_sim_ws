from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    # Get robot description
    pkg_desc = get_package_share_directory('igvc_description')
    urdf_file = os.path.join(pkg_desc, 'urdf', 'robot.xacro')
    robot_desc = xacro.process_file(urdf_file).toxml()

    # Get custom world file
    world_file = os.path.join(
        get_package_share_directory('igvc_gazebo'),
        'worlds',
        'igvc.world'
    )

    return LaunchDescription([

        # Start Gazebo with custom world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # Delay robot spawn so world fully loads
        TimerAction(
            period=3.0,
            actions=[

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[{'robot_description': robot_desc}]
                ),

                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher'
                ),

                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-topic', 'robot_description',
                        '-name', 'igvc_bot',
                        '-allow_renaming', 'false',
                        '-z', '0.0'
                    ],
                    output='screen'
                ),

                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                        '/model/igvc_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
                    ],
                    remappings=[
                        ('/model/igvc_bot/tf', '/tf')
                    ],
                    output='screen'
                )

            ]
        )
    ])
