from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bumpgo_cmd = Node(
        package='fsm_bumpgo_cpp',
        executable='bumpgo_node_exe',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        remappings=[
            ('input_scan', '/scan'),
            ('output_vel', '/cmd_vel')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(bumpgo_cmd)
    return ld