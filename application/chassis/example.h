/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       example.c/h
  * @brief      示例文件
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-11-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef EXAMPLE_H
#define EXAMPLE_H
#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_EXAMPLE)

#include "custom_typedef.h"
#include "motor.h"
#include "pid.h"
#include "struct_typedef.h"
#include "user_lib.h"

/*-------------------- Structural definition --------------------*/

typedef enum {
    CHASSIS_MODE_POS,  // 位控模式
    CHASSIS_MODE_VEL,  // 速控模式
    CHASSIS_MODE_OFF
} ChassisMode_e;

/**
 * @brief  底盘数据结构体
 */
typedef struct
{
    ChassisMode_e mode;  // 底盘模式
    uint8_t error_code;  // 底盘错误代码

    /*-------------------- Motors --------------------*/
    Motor_s motor[4];
    /*-------------------- Values --------------------*/
    struct
    {
        float pos[4];  // 位置
        float vel[4];  // 速度
    } ref;             // 期望值
    struct
    {
        float pos[4];   // 位置
        float Dpos[4];  // 位置差
        float vel[4];   // 速度
    } fdb;              // 状态值
    struct
    {
        uint16_t value[4];  // 电机控制量
    } cmd;                  // 控制量
    struct
    {
        pid_type_def pos[4];  // 位置PID
        pid_type_def vel[4];  // 速度PID
    } pid;                    // PID控制器
    struct
    {
        LowPassFilter_t vel[4];  // 速度低通滤波器
    } lpf;                       // 低通滤波器
} Chassis_s;

extern void ChassisInit(void);

extern void ChassisHandleException(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif  /* CHASSIS_EXAMPLE */
#endif  // EXAMPLE_H
/*------------------------------ End of File ------------------------------*/
