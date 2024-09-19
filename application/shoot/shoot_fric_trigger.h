/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"

//#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)
#ifndef SHOOT_FRIC_H
#define SHOOT_FRIC_H
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "shoot.h"

#define shoot_fric_kp 8
#define shoot_fric_ki 0.5
#define shoot_fric_kd 0.05
#define shoot_fric_max_out 0
#define shoot_fric_max_iout 1000
#define shoot_trigger_kp 8
#define shoot_trigger_ki 0.5
#define shoot_trigger_kd 0.05
#define shoot_trigger_max_out 0
#define shoot_trigger_max_iout 1000

#define shoot_trigger_speed 1111
#define shoot_fric_speed 1111
#define PI 3.14
typedef struct
{
    const RC_ctrl_t * rc;  // 射击使用的遥控器指针
    LoadMode_e mode;       // 射击模式
    FricState_e state;     // 摩擦轮状态

    Motor_s fric_motor[4];  // 摩擦轮电机
    Motor_s trigger_motor;  // 拨弹盘电机

    /*目标量*/
    float shoot_frequency;  // (Hz)射频
    float shoot_speed;      // (m/s)射速
    float dangle;           // (rad)拨弹盘单次转动角度

    //pid
    pid_type_def trigger_pid;
    pid_type_def fric_pid[4];
    fric_PID[3] = {shoot_fric_kp, shoot_fric_ki, shoot_fric_kd};
    trigger_PID[3] = {shoot_trigger_kp, shoot_trigger_ki, shoot_trigger_kd};
} Shoot_s;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
}

extern void InitShoot(void);

extern void SetShootMode(void);

extern void ShootObserver(void);

extern void ShootReference(void);

extern void ShootConsole(void);

extern void SendShootCmd(void);

#endif  // SHOOT_FRIC_H
//#endif  // SHOOT_TYPE == SHOOT_FRIC
