from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value = 'true'
    )
    
    rviz_goal_relay_node = Node(
        package = 'asl_tb3_lib',
        executable = 'rviz_goal_relay.py',
        parameters=[{"output_channel": "/cmd_nav"}]
    )
    
    state_publisher_node = Node(
        package = 'asl_tb3_lib',
        executable = 'state_publisher.py',
    )
    
    navigator_node = Node(
        package = 'autonomy_repo',
        executable = 'navigator.py',
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}]
    )

    rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]),
        launch_arguments ={
            "config": PathJoinSubstitution([FindPackageShare("autonomy_repo"), "rviz", "default.rviz",]),
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            }.items(),  
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_goal_relay_node,
        state_publisher_node,
        navigator_node,
        rviz_launch
    ])