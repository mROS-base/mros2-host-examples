from launch import LaunchDescription
from launch_ros.actions import Node

# Note: `node_`, `prefix` and `output` will be removed on Foxy
def generate_launch_description():
  return LaunchDescription([
    Node(
      package='mros2_sub_twist',
      executable='sub_node',
      name='sub_twist',
			prefix=['stdbuf -o L'],
			output="screen"
    )
  ])