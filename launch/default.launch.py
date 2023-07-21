from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hiros_skeleton_visualizer',
            executable='hiros_skeleton_visualizer',
            name='skeleton_visualizer',
            namespace='hiros',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'input_topic': '/input/topic'},
                {'output_topic': '/output/topic'},
                {'publish_tfs': False},
                {'seed': 0},
                {'lifetime': 0.1},
                {'alpha': 1.},
            ]
        )
    ])
