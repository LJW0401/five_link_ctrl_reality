/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是舵轮底盘参数配置文件，包括物理参数、PID参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-02-05     Harry_Wong        1. 初步建立底盘框架
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_RUDDER_WHEEL        // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                  // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                    // 选择发射机构类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  // 选择机械臂类型
#define CONTROL_TYPE CHASSIS_ONLY                // 选择控制类型

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS 0.08f                  //(m)轮子直径
#define WHEEL_CENTER_DISTANCE 0.304138127f  //(m)轮子到车的距离（0.22 + 0.21）

//motor parameters ---------------------
//电机ID ---------------------
#define WHEEL_1_ID (1)
#define WHEEL_2_ID (2)
#define WHEEL_3_ID (3)
#define WHEEL_4_ID (4)

#define RUDDER_1_ID (1)
#define RUDDER_2_ID (2)
#define RUDDER_3_ID (3)
#define RUDDER_4_ID (4)

//电机CAN ---------------------
#define WHEEL_CAN (1)

#define RUDDER_CAN (1)

//电机种类
#define WHEEL_MOTOR_TYPE ((MotorType_e)DJI_M3508)

#define RUDDER_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//电机方向
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (-1)
#define WHEEL_4_DIRECTION (-1)

#define RUDDER_1_DIRECTION (1)
#define RUDDER_2_DIRECTION (1)
#define RUDDER_3_DIRECTION (-1)
#define RUDDER_4_DIRECTION (-1)

//电机减速比
#define WHEEL_1_RATIO (19)
#define WHEEL_2_RATIO (19)
#define WHEEL_3_RATIO (19)
#define WHEEL_4_RATIO (19)

#define RUDDER_1_RATIO (19)
#define RUDDER_2_RATIO (19)
#define RUDDER_3_RATIO (19)
#define RUDDER_4_RATIO (19)
//电机模式
#define WHEEL_MODE (0)

#define RUDDER_MODE (0)

//舵机初始方向
#define RUDDER_1_INIT_POS (0.0f)
#define RUDDER_2_INIT_POS (0.0f)
#define RUDDER_3_INIT_POS (0.0f)
#define RUDDER_4_INIT_POS (0.0f)


//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_WHEEL_VEL (20.0f)
#define KI_WHEEL_VEL (0.3f)
#define KD_WHEEL_VEL (0.3f)
#define MAX_IOUT_WHEEL_VEL (10000.0f)
#define MAX_OUT_WHEEL_VEL (30000.0f)

//方向舵角度环PID参数
#define KP_RUDDER_POS (20.0f)
#define KI_RUDDER_POS (0.3f)
#define KD_RUDDER_POS (0.3f)
#define MAX_IOUT_RUDDER_POS (10000.0f)
#define MAX_OUT_RUDDER_POS (30000.0f)

//方向舵速度环PID参数
#define KP_RUDDER_VEL (20.0f)
#define KI_RUDDER_VEL (0.3f)
#define KD_RUDDER_VEL (0.3f)
#define MAX_IOUT_RUDDER_VEL (10000.0f)
#define MAX_OUT_RUDDER_VEL (30000.0f)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (200.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.3f)
#define KD_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (10000.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (2.0f)

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (2.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式

#endif
