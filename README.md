# 无人机控制节点概述和文档

## 概述

此代码定义了一个无人机（Unmanned Aerial Vehicle，UAV）控制节点，可发送各种指令来控制无人机的行为。该节点使用ROS（机器人操作系统）进行通信和控制。无人机可以执行起飞、降落、跟随目标和扫描等动作。

Based on the official TTA_UAV sample code for secondary development, suitable for the DJI Intelligent Warehousing Competition


### 代码结构

- `uavControl.cpp`：包含无人机控制节点的源代码
  - 按照10Hz的频率向`uavAction`话题发送控制指令。

- `GlobalControl.cpp`：包含全局控制节点的源代码
  - 通过订阅 `updateStatus`话题的消息，读取当前无人机状态，更新参数服务器中的状态值。

- `Test_Comm_Control.cpp`：包含测试代码
  - 用于测试无人机控制节点和全局控制节点的功能。


- `uavControl.h`：包含无人机控制节点的头文件。
- `GlobalControl.h`：包含全局控制节点的头文件。

### 主要功能

1. 无人机控制节点`uavControl_node`：

- 接收来自全局控制节点的指令。

- 根据接收到的指令，发送不同的无人机控制指令，如起飞、降落、跟随目标、扫描等。

- 通过更新参数服务器中的参数，实现动作的触发和连续执行。

2. 全局控制节点`global_control_node`：

- 通过发布消息更新参数服务器中的状态值。

- 根据状态更新消息，触发相应的无人机控制指令。

- 设置无人机的控制状态，如起飞、降落、跟随、扫描等。

### 依赖关系

- ROS（Robot Operating System）：用于无人机控制的通信和消息发布。

### 执行流程

1. 在无人车中分别启动无人机控制节点（uavControl_node）和全局控制节点（Global_control_node）。

2. Global_control_node在接收`updateStatus`话题的消息后，读取`StatusUpdate`的状态，由更新参数服务器中的状态值。

3. uavControl_node不断读取参数服务器中的状态值，根据`StatusUpdate`状态值以及无人机当前状态发送相应的无人机控制指令。

4. 无人机执行相应的动作，如起飞、降落、跟随、扫描等。

通过这样的控制系统框架，可以实现通过发布状态更新消息，来控制无人机执行不同的动作。全局控制节点负责接收外部信号，并触发不同的动作，而无人机控制节点负责接收指令，并执行相应的控制动作。
整体控制系统实现了只需要设置`StatusUpdate`状态值，以及发布`updateStatus`消息，就可以控制无人机执行不同动作的功能。方便无人车端的开发。


## 解释

### 类 `uavControl`
无人机控制发布器。用于在无人车上控制无人机， 包括起飞降落、飞行、云台控制等

- #### 函数 `paramUpdate()`

  - 用于更新参数服务器中的参数 频率为10Hz 
  - 逻辑：读取参数服务器中的参数，更新无人机控制节点的状态。
  - 参数包括起飞/降落、跟随、扫描和云台控制的标志。

- #### 函数 `sendUpdate()`

  - 用于更新无人机控制发布 频率为10Hz 
  - 逻辑：
  - 若`takeoffOrLanding`为1，则发送起飞指令，若为2，则发送降落指令，若为0，则不发送 
  - 若在已经起飞且`ifFollow`为true的情况下，发送follow指令 
  - 若在扫码模式下，则按照`flag`发送动作序列指令

- #### 函数 `uavControl()`

  - `uavControl` 类的构造函数。

  - 初始化ROS发布器，设置默认参数值，并为无人机定义扫描和跟随路径。

### 类 `GlobalControl`

全局控制节点。用于在无人车上控制全局状态，以触发无人机的不同动作，包括起飞、降落、跟随、扫描等。

- #### 函数 `GlobalParamUpdate()`

  - 用于更新参数服务器中的参数。频率为10Hz。
  - 逻辑：读取参数服务器中的参数，更新全局控制节点的状态。
  - 参数包括无人机状态的更新，通过状态更新来触发不同的动作。

- #### 函数 `GlobalControlUpdate()`

  - 用于更新无人机状态并设置相应的控制指令。
  - 逻辑：根据接收到的无人机状态更新，设置无人机的控制指令。
  - 当接收到起飞状态 `takeoff`，则设置起飞指令，使无人机起飞。
  - 当接收到跟随状态 `follow`，则设置跟随目标指令，使无人机开始跟随目标。
  - 当接收到降落状态 `landing`，则设置降落指令，使无人机降落。
  - 当接收到第一次扫描状态 `first_scan`，则设置第一次扫描指令，使无人机开始执行第一次扫描任务。
  - 当接收到第二次扫描状态 `second_scan`，则设置第二次扫描指令，使无人机开始执行第二次扫描任务。

- #### 回调函数 `StatusUpdate_cb(const std_msgs::Bool::ConstPtr& msg_p)`

  - 用于接收并处理来自话题 "updateStatus" 的状态更新消息。
  - 在接收到新消息时，调用 `GlobalControlUpdate()` 函数，更新无人机状态并设置相应的控制指令。

- #### 函数 `GlobalControl()`

  - `GlobalControl` 类的构造函数。
  - 初始化ROS订阅器，用于接收状态更新消息。
  - 设置默认无人机状态值，并在构造函数中进行初始化。

### 执行流程
### UAV Control Commands

```bash
roslaunch ttauav_node uav_launch.launch
```

1. To run the UAV control test node:

```bash
rosrun ttauav_node Test_Comm_Control <action>
```

Replace `<action>` with one of the following options:

- `1` for takeoff
- `2` for landing
- `3` for follow
- `4` for the first scan
- `5` for the second scan