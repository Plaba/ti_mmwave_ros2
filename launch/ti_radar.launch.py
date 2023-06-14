from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def get_keys_and_values(obj):
    
    obj_dict = obj.__dict__
    return ", ".join([f"{key}={value}" for key, value in obj_dict.items()])

LaunchConfiguration.__str__ = lambda self: f"LaunchConfiguration({get_keys_and_values(self)})"
TextSubstitution.__str__ = lambda self: f"TextSubstitution(text='{self.text}')"
TextSubstitution.__repr__ = lambda self: f"TextSubstitution(text='{self.text}')"

def generate_launch_description():
    # Input arguments
    sensor_ns = LaunchConfiguration('mav_name', default='zero')
    # log_output = LaunchConfiguration('log_output', default='screen') # Substitutions are not performed on the output field
    radar_type = LaunchConfiguration('radar_type', default='1843AOP')
    radar_frame = LaunchConfiguration('radar_frame', default='ti_mmwave')

    launch_tf = LaunchConfiguration('launch_tf', default='true')
    launch_bag = LaunchConfiguration('launch_bag', default='false')
    launch_rviz = LaunchConfiguration('launch_rviz', default='false')

    return LaunchDescription([
        # Input arguments
        DeclareLaunchArgument('mav_name', default_value='zero'),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('radar_type', default_value='1843AOP', description='[1843AOP, 6843AOP]'),
        DeclareLaunchArgument('radar_frame', default_value='ti_mmwave'),

        # mmWave_Manager node
        GroupAction(
            actions=[
                Node(
                    package='ti_mmwave_ros2',
                    executable='mmWaveQuickConfig',
                    namespace=sensor_ns,
                    name='mmWaveQuickConfig',
                    arguments=[
                        PathJoinSubstitution([
                        get_package_share_directory("ti_mmwave_ros2"),
                        'cfg',
                        radar_type,
                        PythonExpression(['"',radar_type,'_shortRange_10fps.cfg"'])
                    ])],
                    output="screen", 
                ),
                Node(
                    package='ti_mmwave_ros2',
                    executable='ti_mmwave_waveloader',
                    # name='ti_mmwave_ros2',
                    namespace=sensor_ns,
                    output="screen",
                    # arguments=['--ros-args', '--log-level', 'debug'],
                    parameters=[
                        {'command_port': '/dev/ttyACM0'},
                        {'command_rate': 115200},
                        {'data_port': '/dev/ttyACM1'},
                        {'data_rate': 921600},
                        {'frame_id': radar_frame}
                    ]
                ),
            ],
        ),

        # Bag record node
        Node(
            package='rosbag2',
            executable='record',
            name='rosbag2',
            namespace=sensor_ns,
            output='screen',
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
            arguments=['-d', PathJoinSubstitution([ get_package_share_directory("ti_mmwave_ros2"),
                        'cfg', 'viz', 'std.rviz'])
            ],
            condition=IfCondition(launch_rviz),
        )
    ])
