from launch import LaunchDescription
from launch_ros.actions import Node

# Note: `node_`, `prefix` and `output` will be removed on Foxy
def generate_launch_description():
  return LaunchDescription([
    Node(
      package='mros2_pub_pose',
      node_executable='pub_node',
      node_name='pub_pose',
			prefix=['stdbuf -o L'],
			output="screen"
    )
  ])
