## 2025.07.23
### Features
- 增加了遥操 ur12e 机械臂

### Bug Fixes
None

### Other Changes
None

## 2025.06.24
### Features
- 修改了遥操作双击逻辑：现阶段遥操作piper，无论pika显示灯光，双击后灯光改变就会触发遥操作或停止遥操作。若处于遥操作状态，双击即停止，若处于非遥操作状态，双击则开启遥操作。后续将使用灯光进行错误提示。

### Bug Fixes
None

### Other Changes
None

## 2025.06.17
### Features
None

### Bug Fixes
- 修复了需要从绝对路径加载URDF的问题

### Other Changes
None

## 2025.06.11
### Features
- 修改了遥操作机械臂的方式： 定义好的初始姿态启动控制，且结束遥操后机械臂回到初始姿态 ---> 可以双击停止控制，再次双击即可从停止位姿继续控制

### Bug Fixes
None

### Other Changes
None