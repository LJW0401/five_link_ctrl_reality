/**
  * @file       robot_param_train.h
  * @brief      这里是培训用机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_BALANCE             // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                  // 选择云台类型

// clang-format off
/*-------------------- Chassis --------------------*/
// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357   // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 2    // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)   // 底盘任务控制间隔

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL   0  // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL      1  // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL      0  // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL     0  // 旋转的遥控器通道号码
#define CHASSIS_ANGLE_CHANNEL  2  // 腿摆角的遥控器通道号码
#define CHASSIS_LENGTH_CHANNEL 3  // 腿长的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL   4  // ROLL角的遥控器通道号码
#define CHASSIS_RC_DEADLINE    5  // 摇杆死区

// deadzone parameters ---------------------
// ratio parameters ---------------------
// motor parameters ---------------------
//physical parameters ---------------------
//upper_limit parameters ---------------------
//lower_limit parameters ---------------------
//PID parameters ---------------------
//LPF parameters ---------------------
//other parameters ---------------------











/*-------------------- Gimbal --------------------*/
//physical parameters ---------------------
//PID parameters ---------------------

/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
//PID parameters ---------------------

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
