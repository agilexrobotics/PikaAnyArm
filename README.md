<div align="center">
  <h1 align="center"> PikaAnyArm </h1>
  <h3 align="center"> Agilex Robotics </h3>
  <p align="center">
    <a href="README.md"> English </a> | <a>中文</a> 
  </p>
</div>

## 介绍

该仓库实现了使用 pika sense 对 Piper 以及 Xarm lite6 机械臂进行遥操作，需要将其放置到 pika_ros/src 路径下编译使用。

如果您在使用过程中遇到任何问题，或者有任何建议和反馈，请通过以下方式联系我们：

- GitHub Issues: https://github.com/agilexrobotics/PikaAnyArm/issues
- 电子邮件: [support@agilex.ai](mailto:support@agilex.ai)

我们的技术团队将尽快回复您的问题，并提供必要的支持和帮助。

pika sdk：https://github.com/agilexrobotics/pika_sdk

pika_ros：https://github.com/agilexrobotics/pika_ros

有关更多信息，您可以参考 [Pika 遥操作机械臂手册](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/axi8hh9h9t2sh2su#380914a8) 和 [PIKA使用QA查询](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/ltl2m8a3crra12kg)。

## 支持的环境平台

### 软件环境

- 架构：x86_64
- 操作系统：Ubuntu20.04
- ROS：noetic

## CHANGELOG

### 2025.06.17
#### Features
None

#### Bug Fixes
- 修复了需要从绝对路径加载URDF的问题

#### Other Changes
None

### 2025.06.11
#### Features
- 修改了遥操作机械臂的方式： 定义好的初始姿态启动控制，且结束遥操后机械臂回到初始姿态 ---> 可以双击停止控制，再次双击即可从停止位姿继续控制

#### Bug Fixes
None

#### Other Changes
None