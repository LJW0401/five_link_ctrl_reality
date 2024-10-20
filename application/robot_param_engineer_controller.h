/**
  * @file       robot_param_engineer_controller.h
  * @brief      这里是工程自定义控制器参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "motor.h"
#include "robot_typedef.h"

#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_ENGINEER  // 选择自定义控制器类型

// 机器人物理参数
/*-------------------- Custom Controller --------------------*/
//motor parameters ---------------------

#define JOINT_MOTOR_0_ID 1
#define JOINT_MOTOR_1_ID 2
#define JOINT_MOTOR_2_ID 3
#define JOINT_MOTOR_3_ID 1
#define JOINT_MOTOR_4_ID 2
#define JOINT_MOTOR_5_ID 3

#define JOINT_MOTOR_0_CAN 1
#define JOINT_MOTOR_1_CAN 1
#define JOINT_MOTOR_2_CAN 1
#define JOINT_MOTOR_3_CAN 1
#define JOINT_MOTOR_4_CAN 1
#define JOINT_MOTOR_5_CAN 1

#define JOINT_MOTOR_0_TYPE DJI_M6020
#define JOINT_MOTOR_1_TYPE DJI_M6020
#define JOINT_MOTOR_2_TYPE DJI_M6020
#define JOINT_MOTOR_3_TYPE DJI_M3508
#define JOINT_MOTOR_4_TYPE DJI_M3508
#define JOINT_MOTOR_5_TYPE DJI_M2006

#define JOINT_MOTOR_0_DIRECTION 1
#define JOINT_MOTOR_1_DIRECTION 1
#define JOINT_MOTOR_2_DIRECTION -1
#define JOINT_MOTOR_3_DIRECTION 1
#define JOINT_MOTOR_4_DIRECTION 1
#define JOINT_MOTOR_5_DIRECTION 1

#define JOINT_MOTOR_0_MODE 0
#define JOINT_MOTOR_1_MODE 0
#define JOINT_MOTOR_2_MODE 0
#define JOINT_MOTOR_3_MODE 0
#define JOINT_MOTOR_4_MODE 0
#define JOINT_MOTOR_5_MODE 0

#define J0_ANGLE_TRANSFORM 0.0f
#define J1_ANGLE_TRANSFORM 1.57f
#define J2_ANGLE_TRANSFORM 1.05f
#define J3_ANGLE_TRANSFORM 0.0f
#define J4_ANGLE_TRANSFORM 0.0f
#define J5_ANGLE_TRANSFORM 0.0f

//LPF parameters ---------------------

#define J0_LPF_ALPHA 0.8f
#define J1_LPF_ALPHA 0.8f
#define J2_LPF_ALPHA 0.8f
#define J3_LPF_ALPHA 0.35f
#define J4_LPF_ALPHA 0.35f
#define J5_LPF_ALPHA 0.35f

// Ratio parameters ---------------------

#define J0_VEL_TO_VALUE (-2000.0f)
#define J1_VEL_TO_VALUE (-2000.0f)
#define J2_VEL_TO_VALUE (-500.0f)
#define J3_VEL_TO_VALUE (-300.0f)
#define J4_VEL_TO_VALUE (-500.0f)
#define J5_VEL_TO_VALUE (-100.0f)

// Init parameters ---------------------
// Other parameters ---------------------

#endif /* INCLUDED_ROBOT_PARAM_H */
