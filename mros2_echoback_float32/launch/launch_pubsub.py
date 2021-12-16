from launch import LaunchDescription
from launch_ros.actions import Node

# Note: `node_`, `prefix` and `output` will be removed on Foxy
def generate_launch_description():
  return LaunchDescription([
    Node(
      package='mros2_echoback_float32',
      executable='pub_node',
      name='pub_mros2',
      prefix=['stdbuf -o L'],
      output="screen"
    ),
    Node(
      package='mros2_echoback_float32',
      executable='sub_node',
      name='mros2_sub',
      prefix=['stdbuf -o L'],
      output="screen"
    )
  ])
