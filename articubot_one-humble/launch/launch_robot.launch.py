import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME
    joy_package_name='joy2twist' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(joy_package_name),'launch','gamepad_controller.launch.py'
                )])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')],
            respawn=True,
            respawn_delay=1.0
        )

    zlac_run = Node(
            package="zlac8015d",
            executable="zlac_run",
            name="zlac_run",
            output="screen",
            respawn=True,
            respawn_delay=1.0
        )

    rplidar_s1 = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 256000,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            respawn=True,
            respawn_delay=1.0
        )

    odometry_calculator = Node(
            package='odometry_calculator',
            executable='odometry_calculator',
            name='odometry_calculator',
            output='screen',
            respawn=True,
            respawn_delay=1.0
        )

    online_async = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        zlac_run,
        rplidar_s1,
        odometry_calculator,
        online_async
    ])
