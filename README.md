# robotiq_3f_gripper_ros2_stack
Used in the 5th semester [Robotics LLM-Planner-for-Bimanual-object-mnipulation project](https://github.com/andreasHovaldt/LLM-Planner-for-Bimanual-object-manipulation). Package stack provides functionalities for controlling a Robotiq 3 Finger Adaptive Gripper using ROS2.

## Required libraries
### Python3
```shell
pip install pymodbus
```

## Quick Start
Install [colcon](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html#install-colcon), then build this repository:

```shell
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/andreasHovaldt/robotiq_3f_gripper_ros2_stack.git
cd ~/ros_ws
colcon build --symlink-install
source ~/ros_ws/install/setup.bash
```

## Functionalities
For controlling the gripper, the custom interfaces used are based on the msg types ```Robotiq3FGripperInputRegisters.msg```
and ```Robotiq3FGripperOutputRegisters.msg```, found in the folder ```/robotiq_3f_gripper_ros2_interfaces/msg```. More can be read about the Modbus registers and their contents in [Robotiq's own documentation](https://assets.robotiq.com/website-assets/support_documents/document/3-Finger_PDF_20210617.pdf).
It is worth noting that this current implementation only uses the simplified control mode, which can also be read about in Robotiq's documentation. 

The gripper is able to be controlled in 3 different ways:

### Gripper topic subscription
```shell
ros2 run robotiq_3f_gripper_ros2_control gripper_control_listener_node
```
Using ```gripper_control_listener_node``` the gripper is subscribed to the topic /Robotiq3FGripper/OutputRegisters
Additionally this node also publishes the input registers (status of gripper) to the topic /Robotiq3FGripper/InputRegisters

#### Helper command for interacting with the topic subscriber
```shell
ros2 topic pub --once /Robotiq3FGripper/OutputRegisters robotiq_3f_gripper_ros2_interfaces/msg/Robotiq3FGripperOutputRegisters "{r_act: 1, r_mod: 1, r_gto: 1, r_atr: 0, r_pra: 80, r_spa: 255, r_fra: 0}"
```


### Gripper service server
```shell
ros2 run robotiq_3f_gripper_ros2_control gripper_control_service_server
```
This starts a service server for the gripper. Check the custom interface ```Robotiq3FGripperOutputService.srv``` for more information on how to interact with the service server.

#### Helper command for interacting with the service server
```shell
ros2 service call /Robotiq3FGripper/OutputRegistersService robotiq_3f_gripper_ros2_interfaces/srv/Robotiq3FGripperOutputService "{output_registers: {r_act: 1, r_mod: 1, r_gto: 1, r_atr: 0, r_pra: 255, r_spa: 255, r_fra: 0}}"
```


### Gripper action server
```shell
ros2 run robotiq_3f_gripper_ros2_control gripper_control_action_server
```
This starts an action server for the gripper. Check the custom interface ```Robotiq3FGripperOutputGoal.action``` for more information on how to interact with the action server.

#### Helper command for interacting with the action server
```shell
ros2 action send_goal -f /gripper_position robotiq_3f_gripper_ros2_interfaces/action/Robotiq3FGripperOutputGoal "{output_registers_goal: {r_act: 1, r_mod: 1, r_gto: 1, r_atr: 0, r_pra: 0, r_spa: 255, r_fra: 0}}"
```




