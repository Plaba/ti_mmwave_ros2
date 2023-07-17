from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Input arguments
    sensor_ns = LaunchConfiguration('namespace', default='ti_mmwave')
    radar_type = LaunchConfiguration('radar_type', default='1843AOP')
    radar_frame = LaunchConfiguration('radar_frame', default='ti_mmwave')

    launch_tf = LaunchConfiguration('launch_tf', default='true')
    launch_bag = LaunchConfiguration('launch_bag', default='false')
    launch_rviz = LaunchConfiguration('launch_rviz', default='false')
    launch_dca = LaunchConfiguration('launch_dca', default='true')
    launch_xwr_uart = LaunchConfiguration('launch_xwr_uart', default='true')

    return LaunchDescription([
        # Input arguments
        DeclareLaunchArgument('namespace', default_value='ti_mmwave'),
        DeclareLaunchArgument('radar_type', default_value='1843AOP', description='[1843AOP, 6843AOP]'),
        DeclareLaunchArgument('radar_frame', default_value='ti_mmwave'),
        DeclareLaunchArgument('launch_dca', default_value="True"),

        # Quick config node
        Node(
            package='xwr_config',
            executable='quick_config',
            name='quick_config',
            namespace=sensor_ns,
            parameters=[
                { 'cfg_path':  PathJoinSubstitution([ get_package_share_directory("xwr_config"),
                    'cfg', radar_type, PythonExpression(['"',radar_type,'_shortRange_30fps.cfg"'])])},
                { 'override_enable_lvds': launch_dca },
            ],
        ),
        Node(
            package='xwr_config',
            executable='xwr_config_srv',
            name='xwr_config_srv',
            namespace=sensor_ns,
            parameters=[
                {'command_port': '/dev/ttyACM0'},
                {'command_rate': 115200},
            ],
        ),
        # XWR UART reader node
        Node(
            package='xwr_data',
            executable='xwr_uart_reader',
            name='xwr_uart_reader',
            namespace=sensor_ns,
            condition=IfCondition(launch_xwr_uart),
            parameters=[
                {'data_port': '/dev/ttyACM1'},
                {'data_rate': 921600},
                {'frame_id': radar_frame},
            ],
        ),
        # DCA1000 node
        Node(
            package='dca1000',
            executable='dca1000',
            name='dca1000',
            namespace=sensor_ns,
            condition=IfCondition(launch_dca),
            parameters=[
                {'frame_id': radar_frame},
            ],
        ),
        # Bag record node
        Node(
            package='rosbag2',
            executable='record',
            name='rosbag2',
            namespace=sensor_ns,
            condition=IfCondition(launch_bag),
            arguments=[
                '-e', '(.*)tf(.*)|(.*)ti_mmwave/radar_scan_pcl(.*)|(.*)/camera/(.*)',
                '--compression-mode', 'file',
                '--compression-format', 'zstd',
                '--output', 'rosbag2'
            ]
        ),
        # Static transform from map to base_radar_link for visualization of stand-alone mmWave sensor using Rviz
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_TF_map_radar',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', radar_frame],
            condition=IfCondition(launch_tf),
        ),
        # rviz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution(
                [get_package_share_directory("xwr_data"), 'cfg', 'viz', 'std.rviz'])
            ],
            condition=IfCondition(launch_rviz),
        )
    ])
