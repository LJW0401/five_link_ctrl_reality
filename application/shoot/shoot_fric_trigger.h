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

#define SHOOT_Trigger_Motor_ID          1
#define SHOOT_Trigger_Motor_CAN         1
#define SHOOT_Trigger_Motor_TYPE        DJI_M3508
#define SHOOT_Trigger_Motor_DIRECTION   1
#define SHOOT_Trigger_Motor_RATIO       1
#define SHOOT_Trigger_Motor_MODE        1

#define SHOOT_fric_motor_ID             1
#define SHOOT_fric_motor_CAN            1
#define SHOOT_fric_motor_TYPE           DJI_M3508
#define SHOOT_fric_motor_DIRECTION      1
#define SHOOT_fric_motor_RATIO          1
#define SHOOT_fric_motor_MODE           1



#define shoot_angle_kp 8.0f
#define shoot_angle_ki 0.5f
#define shoot_angle_kd 0.05f
#define shoot_angle_max_out 100f
#define shoot_angle_max_iout 1000f
//角度控制的PID
#define shoot_speed_kp 8.0f
#define shoot_speed_ki 0.5f
#define shoot_speed_kd 0.05f
#define shoot_speed_max_out 100f
#define shoot_speed_max_iout 1000f
//速度控制的PID
#define PI 3.14f

typedef struct
{
    const RC_ctrl_t * rc;  // 射击使用的遥控器指针
    LoadMode_e mode;       // 射击模式
    FricState_e state;     // 摩擦轮状态

    Motor_s fric_motor[2];  // 摩擦轮电机
    Motor_s trigger_motor;  // 拨弹盘电机

    /*目标量*/
    float shoot_frequency;  // (Hz)射频
    float shoot_speed;      // (m/s)射速
    float dangle;           // (rad)拨弹盘单次转动角度
	  
	  //角度过圈判断
    int is_circle;
    //pid
    pid_type_def angle_pid;
    pid_type_def speed_pid;
    float angle_PID[3]; //= {shoot_angle_kp, shoot_angle_ki, shoot_angle_kd};
    float speed_PID[3]; //= {shoot_speed_kp, shoot_speed_ki, shoot_speed_kd};
} Shoot_s;

extern void InitShoot(void);

extern void SetShootMode(void);

extern void ShootObserver(void);

extern void ShootReference(void);

extern void ShootConsole(void);

extern void SendShootCmd(void);

#endif  // SHOOT_FRIC_H
//#endif  // SHOOT_TYPE == SHOOT_FRIC
