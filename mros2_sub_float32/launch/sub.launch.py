from launch import LaunchDescription
from launch_ros.actions import Node

# Note: `node_`, `prefix` and `output` will be removed on Foxy
def generate_launch_description():
  return LaunchDescription([
    Node(
      package='mros2_sub_float32',
      executable='sub_node',
      name='sub_mros2',
      prefix=['stdbuf -o L'],
      output="screen"
    )
  ])
