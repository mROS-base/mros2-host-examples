# mros2-host-examples

This repository contains ROS 2 packages that can communicate with [mros2](https://github.com/mROS-base/mros2), which is an agent-less and lightweight runtime environment for ROS 2 nodes onto embedded devices.  
Please also check [mROS-base/mros2-asp3-f767zi/workspace](https://github.com/mROS-base/mros2-asp3-f767zi/workspace) for more details about example application on mros2.

## mros2_echoback_string

- Description:
  - The `pub_mros2` node on the host publishes `string` (`std_msgs::msg::String`) message to `/to_stm` topic.
  - (The mros2 node on the embedded device will echoreply this message as it is.)
  - The `mros2_sub` node on the host subscribes `string` message from `/to_linux` topic.
- Host operation:
  - `$ ros2 launch mros2_echoback_string pubsub.launch.py`
  - or, at two terminals:
    - `$ ros2 run mros2_echoback_string pub_node`
    - `$ ros2 run mros2_echoback_string sub_node`
- mros2 application on the embedded device:
  - [echoreply_string](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/echoreply_string)

## mros2_pub_uint16

- Description:
  - The `pub_mros2` node on the host publishes `uint16` (`std_msgs::msg::UInt16`) message to `/to_stm` topic.
- Host operation:
  - `$ ros2 launch mros2_pub_uint16 pub.launch.py`
  - or, `$ ros2 run mros2_pub_uint16 pub_node`
- mros2 application on the embedded device:
  - [sub_uint16](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/sub_uint16)

## mros2_sub_float32

- Description:
  - The `mros2_sub` node on the host subscribes `float32` (`std_msgs::msg::Float32`) message from `/to_linux` topic.
- Host operation:
  - `$ ros2 launch mros2_sub_float32 sub.launch.py`
  - or, `$ ros2 run mros2_sub_float32 sub_node`
- mros2 application on the embedded device:
  - [pub_float32](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/pub_float32)

## mros2_pub_pose

- Description:
  - The `pub_pose` node on the host publishes `Pose` (`geometry_msgs::msg::Pose`) message to `/cmd_vel` topic.
- Host operation:
  - `$ ros2 launch mros2_pub_pose launch.py`
  - or, `$ ros2 run mros2_pub_pose pub_node`
- mros2 application on the embedded device:
  - [sub_pose](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/sub_pose)

## mros2_sub_twist

- Description:
  - The `sub_twist` node on the host subscribes `Twist` (`geometry_msgs::msg::Twist`) message from `/cmd_vel` topic.
- Host operation:
  - `$ ros2 launch mros2_sub_twist sub.launch.py`
  - or, `$ ros2 run mros2_sub_twist sub_node`
- mros2 application on the embedded device:
  - [pub_twist](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/pub_twist)
