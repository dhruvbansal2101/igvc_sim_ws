from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_igvc_desc = get_package_share_directory('igvc_description')
    pkg_igvc_gazebo = get_package_share_directory('igvc_gazebo')

    urdf_path = os.path.join(
        pkg_igvc_desc,
        'urdf',
        'robot.xacro'
    )

    world_path = os.path.join(
        pkg_igvc_gazebo,
        'worlds',
        'stable.world'
    )

    # 1️⃣ Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': os.popen(f'xacro {urdf_path}').read()
        }]
    )

    # 2️⃣ Start Gazebo Sim WITH explicit stable world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items(),
    )

    # 3️⃣ Spawn robot AFTER Gazebo is ready
    spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-world', 'default',          # 🔴 THIS WAS MISSING
        '-name', 'igvc_bot',
        '-topic', '/robot_description',
        '-x', '0',
        '-y', '0',
        '-z', '0.3'
    ],
    output='screen'
)


    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        )
    ])

