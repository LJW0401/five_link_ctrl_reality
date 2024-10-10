# StandardRobot++

<div align=center>

![Logo](./doc/pic/Logo6.png)

[![Author](https://img.shields.io/badge/Author-小企鹅-orange.svg)](https://gitee.com/Ljw0401)
[![Maintainer](https://img.shields.io/badge/Maintainer-YZX-orange.svg)](https://gitee.com/yuan-zaixiu)

![language](https://img.shields.io/badge/language-C-blue.svg)
![license](https://img.shields.io/badge/license-MIT-green.svg)

[![commit](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=StandardRobotpp&type=commit)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)
[![fork](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp/badge/fork.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)
[![star](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp/badge/star.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)
[![release](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=StandardRobotpp&type=release)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)

</div>

## 简介

> 也许不是最好的？但一定是最适合上手的C板电控框架！

本项目的计划是基于 DJI StandardRobot 的基础上改造成一个更适合北极熊uu们的通用型机器人代码框架。

本框架致力于实现不同类型的机器人的代码通用化，只需要选择底盘云台的类型，修改一下对应物理参数即可实现对机器人的适配。

> 写代码时遇到问题可以查看:

- [培训指南](./doc/培训阶段指南.md)
- [API](./doc/API.md)

## 25赛季培训计划

### 前言

为了让大家快速融入到北极熊现有电控体系中，本赛季的培训计划以实践为主，让大家在实操过程中逐渐掌握需要的技能。

### 第一阶段

> 本阶段主要让大家学会主要外设的使用，不深入讲解具体实现原理（如对其原理感兴趣的可私聊）。

1. 实现电机的开环控制，能使用提供的函数实现对 DJI、DM 系列电机的控制。
2. 了解遥控器的使用，能用[上位机调试工具](https://gitee.com/SMBU-POLARBEAR/Serial_Port_Assistant)输出遥控器各个通道值的波形。
3. 了解陀螺仪的使用，用上位机调试工具输出波形。

### 第二阶段

> 本阶段主要让大家理解控制的基本逻辑和一些相关名词，让大家达到能独立编写控制任务的水平。

1. 在底盘模块(CHASSIS)中，根据`chassis_task.c`的内容编写一个完善的控制任务`train_virtual_ctrl.c/h`\
`主要要求：`

   - 文件中有清晰的函数结构。
   - 根据控制的基本逻辑完成函数 $f(r,p,y) = \sin r + \tan p +\cos y$ ，实现一个陀螺仪角度相关的变换。
   - 用上位机调试工具输出结果波形。

2. 在底盘模块(CHASSIS)中，根据`chassis_task.c`的内容编写一个完善的控制任务`train_motor_ctrl.c/h`\
`主要要求：`

   - 文件中有清晰的函数结构。
   - 根据控制的基本逻辑实现任意型号DJI电机的 `速度闭环控制` 和 `位置速度闭环控制`。 （用两个电机同时由C板控制，分别实现两种控制。）
   - 使用遥控器信号作为目标值，速度范围为$[-6,6]$单位：rad/s，位置范围为$[-4\pi,4\pi]$单位：rad。 （速度和位置都指末端输出轴，非转子）
   - 早完成的同学可以尝试控制DM电机实现MIT模式下的速度闭环<!-- 、速度模式下的速度闭环和位置速度模式下的位置速度闭环。 -->

### 第三阶段

> 本阶段主要让大家和机械的同学配合完成一个小车车的项目。

1. 在底盘模块(CHASSIS)中，根据`chassis_task.c`的内容编写一个完善的控制任务`chassis_train.c/h`\
`主要要求：`
   - 文件中有清晰的函数结构。
   - 实现小车底盘的平移、旋转。（若机械结构导致不能左右平移的话可以只做前后平移。）
   - 用遥控器或上位机调试工具进行控制。

### 第四阶段

> 终于来到大家最想玩的环节了，本阶段将会带大家实现我们现有的步兵和英雄的整车控制。同时会穿插讲解一些外设的具体原理。

1. 实现底盘坐标系下的平移旋转运动。
2. 实现云台的陀螺仪绝对位置控制。
3. 实现云台坐标系下的平移旋转运动。

## 配套套件

- [StandardRobot++ Tool](https://gitee.com/SMBU-POLARBEAR/Serial_Port_Assistant) 上位机调试工具，可用于可视化查看机器人状态及相关参数，便于调试。
- [ROS2_StandardRobot++](https://gitee.com/SMBU-POLARBEAR/ROS2_StandardRobotpp) ROS2驱动包，可用于实现上位机对机器人的控制。

## 开发工具

使用VSCode作为代码编辑器，Keil5作为IDE。
> 虽然keil5老，但在嵌入式调试以及环境配置方面是非常简单易上手的。**千万不要用keil5来编辑代码！！！**
\
\
> VSCode作为现代化代码编辑器 ，通过各种插件即可实现各种提高效率的开发方式，**尤其是Github Copilot**

`.vscode` 文件夹已经提供了很多非常好用的插件，在`扩展`页面输入 `@recommended` 即可获取这些插件，点击安装即可。

## 开发日志

[LOG.md](./doc/LOG.md) 记录了大家在开发过程中的各种奇思妙想和经验。多翻一翻或许能发现惊喜哦。大家有什么想法也都可以往里面写。

[API.md](./doc/API.md) 为开发过程中所使用的模块接口标准，若在编写代码时发现接口代码与标准冲突，基于标准修改代码。

## 致谢

感谢各个战队的代码开源，所有本框架涉及到的主要的参考资料位于[reference.md](./doc/reference.md)

## 附录

本文的详细补充内容写在 [附录](./doc/appendix.md) 中。
