import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    map_file = os.path.join(get_package_share_directory('turtlebot_map_server'), 'config', 'maprestricted.yaml')
    nav2_yaml = os.path.join(get_package_share_directory(
        'turtlebot_map_server'), 'config', 'amcl_config.yaml')
    controller_yaml = os.path.join(get_package_share_directory(
        'turtlebot_map_server'), 'config', 'controller.yaml')
    planner_yaml = os.path.join(get_package_share_directory(
        'turtlebot_map_server'), 'config', 'planner_server.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(
        'turtlebot_map_server'), 'config', 'bt_navigator.yaml')
    default_bt_xml_path = os.path.join(get_package_share_directory(
        'turtlebot_map_server'), 'config', 'behavior.xml')
    waypoint_follower_yaml = os.path.join(get_package_share_directory('turtlebot_map_server'), 'config', 'waypoint_follower.yaml')
    rviz_config_file_path = os.path.join(get_package_share_directory(
        'turtlebot_map_server'), 'rviz_config', 'pathplanning.rviz')
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file} 
                       ]),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),              
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoint_follower_yaml]),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz2_node',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_config_file_path]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'bt_navigator',
                                        'waypoint_follower'
                        ]}])
        ])


