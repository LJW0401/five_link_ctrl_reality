/**
  * @file       robot_param_example.h
  * @brief      这里是示例机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_EXAMPLE  // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE       // 选择云台类型

// clang-format off
/*-------------------- Chassis --------------------*/
// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357   // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 2    // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)   // 底盘任务控制间隔

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL   0  // 选择底盘状态 开关通道号
#define CHASSIS_POS_CHANNEL    0  // 位置的遥控器通道号码
#define CHASSIS_VEL_CHANNEL    2  // 速度的遥控器通道号码
#define CHASSIS_RC_DEADLINE    5  // 摇杆死区

// deadzone parameters ---------------------
// ratio parameters ---------------------
//motor parameters ---------------------

#define MOTOR_0_ID 1
#define MOTOR_1_ID 2
#define MOTOR_2_ID 3
#define MOTOR_3_ID 1

#define MOTOR_0_CAN 1
#define MOTOR_1_CAN 1
#define MOTOR_2_CAN 1
#define MOTOR_3_CAN 1

#define MOTOR_0_TYPE DJI_M6020
#define MOTOR_1_TYPE DJI_M6020
#define MOTOR_2_TYPE DJI_M3508
#define MOTOR_3_TYPE DJI_M2006

#define MOTOR_0_DIRECTION 1
#define MOTOR_1_DIRECTION 1
#define MOTOR_2_DIRECTION 1
#define MOTOR_3_DIRECTION 1

#define MOTOR_0_MODE 0
#define MOTOR_1_MODE 0
#define MOTOR_2_MODE 0
#define MOTOR_3_MODE 0

#define M0_ANGLE_TRANSFORM 0.0f
#define M1_ANGLE_TRANSFORM 0.0f
#define M2_ANGLE_TRANSFORM 0.0f
#define M3_ANGLE_TRANSFORM 0.0f
//upper_limit parameters ---------------------
#define MAX_MOTOR_0_POSITION 6.283185f  //2*M_PI
#define MAX_MOTOR_1_POSITION M_PI
#define MAX_MOTOR_2_POSITION M_PI
#define MAX_MOTOR_3_POSITION 6.283185f
//lower_limit parameters ---------------------
#define MIN_MOTOR_0_POSITION 0.0f
#define MIN_MOTOR_1_POSITION 0.0f
#define MIN_MOTOR_2_POSITION 0.0f
#define MIN_MOTOR_3_POSITION 0.0f
//LPF parameters ---------------------
#define M0_LPF_ALPHA 0.0f
#define M1_LPF_ALPHA 0.0f
#define M2_LPF_ALPHA 0.0f
#define M3_LPF_ALPHA 0.0f
//PID parameters ---------------------
//M0速度环PID参数
#define KP_MOTOR_0_VEL 0.0f
#define KI_MOTOR_0_VEL 0.0f
#define KD_MOTOR_0_VEL 0.0f
#define MAX_IOUT_MOTOR_0_VEL 0.0f
#define MAX_OUT_MOTOR_0_VEL 16000.0f
//M0位置环PID参数
#define KP_MOTOR_0_POS 0.0f
#define KI_MOTOR_0_POS 0.0f
#define KD_MOTOR_0_POS 0.0f
#define MAX_IOUT_MOTOR_0_POS 0.0f
#define MAX_OUT_MOTOR_0_POS 4.0f
//M1速度环PID参数
#define KP_MOTOR_1_VEL 0.0f
#define KI_MOTOR_1_VEL 0.0f
#define KD_MOTOR_1_VEL 0.0f
#define MAX_IOUT_MOTOR_1_VEL 0.0f
#define MAX_OUT_MOTOR_1_VEL 16000.0f
//M1位置环PID参数
#define KP_MOTOR_1_POS 0.0f
#define KI_MOTOR_1_POS 0.0f
#define KD_MOTOR_1_POS 0.0f
#define MAX_IOUT_MOTOR_1_POS 0.0f
#define MAX_OUT_MOTOR_1_POS 4.0f
//M2速度环PID参数
#define KP_MOTOR_2_VEL 0.0f
#define KI_MOTOR_2_VEL 0.0f
#define KD_MOTOR_2_VEL 0.0f
#define MAX_IOUT_MOTOR_2_VEL 0.0f
#define MAX_OUT_MOTOR_2_VEL 16000.0f
//M2位置环PID参数
#define KP_MOTOR_2_POS 0.0f
#define KI_MOTOR_2_POS 0.0f
#define KD_MOTOR_2_POS 0.0f
#define MAX_IOUT_MOTOR_2_POS 0.0f
#define MAX_OUT_MOTOR_2_POS 4.0f
//M3速度环PID参数
#define KP_MOTOR_3_VEL 0.0f
#define KI_MOTOR_3_VEL 0.0f
#define KD_MOTOR_3_VEL 0.0f
#define MAX_IOUT_MOTOR_3_VEL 0.0f
#define MAX_OUT_MOTOR_3_VEL 16000.0f
//M3位置环PID参数
#define KP_MOTOR_3_POS 0.0f
#define KI_MOTOR_3_POS 0.0f
#define KD_MOTOR_3_POS 0.0f
#define MAX_IOUT_MOTOR_3_POS 0.0f
#define MAX_OUT_MOTOR_3_POS 4.0f
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
