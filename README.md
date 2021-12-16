# mros2-host-examples

This repository contains ROS 2 packages that can communicate with [mros2](https://github.com/mROS-base/mros2), which is an agent-less and lightweight runtime environment for ROS 2 nodes onto embedded devices.  
Please also check [mROS-base/mros2-asp3-f767zi/workspace](https://github.com/mROS-base/mros2-asp3-f767zi/workspace) for more details about example application on mros2.

## mros2_echoback_string

- Description:
  - The `pub_mros2` node on the host publishes `string` (`std_msgs::msg::String`) message to `/to_stm` topic.
  - (The mros2 node on the embedded device will echoreply this message as it is.)
  - The `mros2_sub` node on the host subscribes `string` message from `/to_linux` topic.
- Host operation:
  - `$ ros2 launch mros2_echoback_string launch.py`
  - or, at two terminals:
    - `$ ros2 run mros2_echoback_string pub_node`
    - `$ ros2 run mros2_echoback_string sub_node`
- mros2 application on the embedded device:
  - [echoreply_string](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/echoreply_string)
  - [pub_string](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/pub_string)
  - [sub_string](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/sub_string)

## mros2_echoback_uint16

- Description:
  - The `pub_mros2` node on the host publishes `uint16` (`std_msgs::msg::UInt16`) message to `/to_stm` topic.
  - (The mros2 node on the embedded device will echoreply this message as it is.)
  - The `mros2_sub` node on the host subscribes `uint16` message from `/to_linux` topic.
- Host operation:
  - `$ ros2 launch mros2_echoback_uint16 launch.py`
  - or, at two terminals:
    - `$ ros2 run mros2_echoback_uint16 pub_node`
    - `$ ros2 run mros2_echoback_uint16 sub_node`
- mros2 application on the embedded device:
  - [echoreply_uint16](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/echoreply_uint16)

## mros2_echoback_string

- Description:
  - The `pub_mros2` node on the host publishes `float32` (`std_msgs::msg::Float32`) message to `/to_stm` topic.
  - (The mros2 node on the embedded device will echoreply this message as it is.)
  - The `mros2_sub` node on the host subscribes `float32` message from `/to_linux` topic.
- Host operation:
  - `$ ros2 launch mros2_echoback_float32 launch.py`
  - or, at two terminals:
    - `$ ros2 run mros2_echoback_float32 pub_node`
    - `$ ros2 run mros2_echoback_float32 sub_node`
- mros2 application on the embedded device:
  - [echoreply_float32](https://github.com/mROS-base/mros2-asp3-f767zi/workspace/echoreply_float32)
