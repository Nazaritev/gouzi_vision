from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    nav2_log_level = LaunchConfiguration('nav2_log_level')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')
    # 2026-05-02: Nav2 is started from the Point-LIO terminal by default so the
    # obstacle race terminal only shows race-manager/serial logs. The race launch
    # still has an opt-in fallback switch for rollback.
    enable_nav2_bringup_arg = DeclareLaunchArgument(
        'enable_nav2_bringup',
        default_value='true',
        description='Start Nav2 navigation nodes together with Point-LIO.',
    )
    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('auto_nav_pkg'),
            'config',
            'nav2_pointlio_navigation.yaml',
        ]),
        description='Nav2 params file for Point-LIO navigation.',
    )
    nav2_autostart_arg = DeclareLaunchArgument(
        'nav2_autostart',
        default_value='true',
        description='Autostart Nav2 lifecycle nodes.',
    )
    nav2_log_level_arg = DeclareLaunchArgument(
        'nav2_log_level',
        default_value='info',
        description='Nav2 log level.',
    )

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'mid360.yaml'
        ]),
        {
            'use_sim_time': use_sim_time,
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'odom_child_frame_id': 'body',
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,  # preserve Livox temporal density under continuous vibration
            'space_down_sample': True,
            'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args'
    )

    # 2026-05-01: Restore the old Nav2 base frame convention without changing
    # Point-LIO localization itself. Point-LIO still publishes camera_init->body;
    # Nav2 and the gait stack can use base_link as the quadruped-forward frame.
    tf_body_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '1.5707963267948966',
            '--frame-id', 'body',
            '--child-frame-id', 'base_link',
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('enable_nav2_bringup')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': nav2_autostart,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': nav2_log_level,
        }.items(),
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        enable_nav2_bringup_arg,
        nav2_params_file_arg,
        nav2_autostart_arg,
        nav2_log_level_arg,
        tf_body_to_base_link,
        nav2_bringup,
        laser_mapping_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
