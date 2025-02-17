/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_mecanum.c/h
  * @brief      麦轮轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.1.19       Harry_Wong        1.重新构建舵轮底盘，完成单底盘控制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/


#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_RUDDER_WHEEL)
#include "chassis_rudder.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h" 
#include "detect_task.h"
#include "gimbal.h"
#include "math.h"
#include "usb_debug.h"

Chassis_s chassis;
PID_t chassis_pid;

/*-------------------- Init --------------------*/

/**
 * @brie。f          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
  //step1 获取所有所需变量指针
  chassis.rc = get_remote_control_point();

  //step2 PID数据清零，设置PID参数

  //驱动轮速度环
  const static fp32 wheel_vel[3]={KP_WHEEL_VEL,KI_WHEEL_VEL,KD_WHEEL_VEL};
  PID_init(&chassis_pid.wheel_velocity,PID_POSITION,wheel_vel,MAX_OUT_WHEEL_VEL,MAX_IOUT_WHEEL_VEL);
  


  //舵机角度环
  const static fp32 rudder_pos0[3]={KP_RUDDER1_POS,KI_RUDDER1_POS,KD_RUDDER1_POS};
  PID_init(&chassis_pid.rudder_position[0],PID_POSITION,rudder_pos0,MAX_OUT_RUDDER1_POS,MAX_IOUT_RUDDER1_POS);

  const static fp32 rudder_pos1[3]={KP_RUDDER2_POS,KI_RUDDER2_POS,KD_RUDDER2_POS};
  PID_init(&chassis_pid.rudder_position[1],PID_POSITION,rudder_pos1,MAX_OUT_RUDDER2_POS,MAX_IOUT_RUDDER2_POS);

  const static fp32 rudder_pos2[3]={KP_RUDDER3_POS,KI_RUDDER3_POS,KD_RUDDER3_POS};
  PID_init(&chassis_pid.rudder_position[2],PID_POSITION,rudder_pos2,MAX_OUT_RUDDER3_POS,MAX_IOUT_RUDDER3_POS);

  const static fp32 rudder_pos3[3]={KP_RUDDER4_POS,KI_RUDDER4_POS,KD_RUDDER4_POS};
  PID_init(&chassis_pid.rudder_position[3],PID_POSITION,rudder_pos3,MAX_OUT_RUDDER4_POS,MAX_IOUT_RUDDER4_POS);



  //舵机速度环
  const static fp32 rudder_vel0[3]={KP_RUDDER1_VEL,KI_RUDDER1_VEL,KD_RUDDER1_VEL};
  PID_init(&chassis_pid.rudder_velocity[0],PID_POSITION,rudder_vel0,MAX_OUT_RUDDER1_VEL,MAX_IOUT_RUDDER1_VEL);

  const static fp32 rudder_vel1[3]={KP_RUDDER2_VEL,KI_RUDDER2_VEL,KD_RUDDER2_VEL};
  PID_init(&chassis_pid.rudder_velocity[1],PID_POSITION,rudder_vel1,MAX_OUT_RUDDER2_VEL,MAX_IOUT_RUDDER2_VEL);
  
  const static fp32 rudder_vel2[3]={KP_RUDDER3_VEL,KI_RUDDER3_VEL,KD_RUDDER3_VEL};
  PID_init(&chassis_pid.rudder_velocity[2],PID_POSITION,rudder_vel2,MAX_OUT_RUDDER2_VEL,MAX_IOUT_RUDDER3_VEL);

  const static fp32 rudder_vel3[3]={KP_RUDDER4_VEL,KI_RUDDER4_VEL,KD_RUDDER4_VEL};
  PID_init(&chassis_pid.rudder_velocity[3],PID_POSITION,rudder_vel3,MAX_OUT_RUDDER2_VEL,MAX_IOUT_RUDDER4_VEL);

  //step3 初始化电机
  MotorInit(&chassis.wheel[0],WHEEL_1_ID,2,WHEEL_MOTOR_TYPE,WHEEL_1_DIRECTION,WHEEL_1_RATIO,WHEEL_MODE);
  MotorInit(&chassis.wheel[1],WHEEL_2_ID,2,WHEEL_MOTOR_TYPE,WHEEL_2_DIRECTION,WHEEL_2_RATIO,WHEEL_MODE);
  MotorInit(&chassis.wheel[2],WHEEL_3_ID,1,WHEEL_MOTOR_TYPE,WHEEL_3_DIRECTION,WHEEL_3_RATIO,WHEEL_MODE);
  MotorInit(&chassis.wheel[3],WHEEL_4_ID,1,WHEEL_MOTOR_TYPE,WHEEL_4_DIRECTION,WHEEL_4_RATIO,WHEEL_MODE);

  MotorInit(&chassis.rudder[0],RUDDER_1_ID,2,RUDDER_MOTOR_TYPE,RUDDER_1_DIRECTION,RUDDER_1_RATIO,RUDDER_MODE);
  MotorInit(&chassis.rudder[1],RUDDER_2_ID,2,RUDDER_MOTOR_TYPE,RUDDER_2_DIRECTION,RUDDER_2_RATIO,RUDDER_MODE);
  MotorInit(&chassis.rudder[2],RUDDER_3_ID,1,RUDDER_MOTOR_TYPE,RUDDER_3_DIRECTION,RUDDER_3_RATIO,RUDDER_MODE);
  MotorInit(&chassis.rudder[3],RUDDER_4_ID,1,RUDDER_MOTOR_TYPE,RUDDER_4_DIRECTION,RUDDER_4_RATIO,RUDDER_MODE);

  //step4 初始模式设置
  chassis.mode = CHASSIS_LOCK;
}


/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
  if ((toe_is_error(DBUS_TOE)) || switch_is_down(chassis.rc->rc.s[0]) )
  {
    chassis.mode = CHASSIS_LOCK;
  }

  else if (switch_is_mid(chassis.rc->rc.s[0]))
  {
    chassis.mode = CHASSIS_SINGLE;
  }

  else if (switch_is_up(chassis.rc->rc.s[0]))
  {
    chassis.mode = CHASSIS_SINGLE;
  }
}


/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) 
{
  //更新电机相关参数
  for (int i=0;i<4;++i)
  {
    GetMotorMeasure(&chassis.wheel[i]);
    GetMotorMeasure(&chassis.rudder[i]);
  }

  chassis.last_mode = chassis.mode;                                                                                        
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
  if (chassis.mode == CHASSIS_LOCK)
  {
    chassis.reference_chassis.vx=0;
    chassis.reference_chassis.vy=0;
    chassis.reference_chassis.wz=0;
  }
  else if (chassis.mode == CHASSIS_SINGLE)
  {
    chassis.reference_chassis.vx=fp32_deadline(chassis.rc->rc.ch[3],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED;
    chassis.reference_chassis.vy=fp32_deadline(-chassis.rc->rc.ch[2],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED;
    chassis.reference_chassis.wz=fp32_deadline(-chassis.rc->rc.ch[0],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_VELOCITY;
  }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量                    
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
  float vx = chassis.reference_chassis.vx,vy = chassis.reference_chassis.vy , wz = chassis.reference_chassis.wz;

  for (int i=0;i<4;++i)
  {
    chassis.reference_wheel[i].vx = vx - wz * WHEEL_CENTER_DISTANCE * (float)sin(M_PI_4 * (1 + 2*i));
    chassis.reference_wheel[i].vy = vy + wz * WHEEL_CENTER_DISTANCE * (float)cos(M_PI_4 * (1 + 2*i));

    chassis.reference_wheel[i].v = (float)sqrt( pow(chassis.reference_wheel[i].vx,2) + pow(chassis.reference_wheel[i].vy,2) );
    chassis.reference_wheel[i].theta = (float)atan2(chassis.reference_wheel[i].vy ,chassis.reference_wheel[i].vx );
  }
  
  chassis.reference_rudder[0] = loop_fp32_constrain(chassis.reference_wheel[0].theta + RUDDER_1_INIT_POS, -M_PI , M_PI);
  chassis.reference_rudder[1] = loop_fp32_constrain(chassis.reference_wheel[1].theta + RUDDER_2_INIT_POS, -M_PI , M_PI);
  chassis.reference_rudder[2] = loop_fp32_constrain(chassis.reference_wheel[2].theta + RUDDER_3_INIT_POS, -M_PI , M_PI);
  chassis.reference_rudder[3] = loop_fp32_constrain(chassis.reference_wheel[3].theta + RUDDER_4_INIT_POS, -M_PI , M_PI);

  for (int i=0;i<4;++i)
  {
    chassis.wheel[i].set.vel = chassis.reference_wheel[i].v * WHEEL_RADIUS;

    chassis.rudder[i].set.pos = chassis.reference_rudder[i] ;
  }

  for (int i=0;i<4;++i)
  {
    chassis.wheel[i].set.curr = PID_calc(&chassis_pid.wheel_velocity,chassis.wheel[i].fdb.vel,chassis.wheel[i].set.vel);

    fp32 rudder_del_pos = loop_fp32_constrain(chassis.rudder[i].set.pos - chassis.rudder[i].fdb.pos ,- M_PI , M_PI);
    chassis.rudder[i].set.curr = PID_calc(&chassis_pid.rudder_position[i],0,rudder_del_pos);
    //chassis.rudder[i].set.curr = PID_calc(&chassis_pid.rudder_velocity[i],chassis.rudder[i].fdb.vel,chassis.rudder[i].set.vel);
  }
 
  for (int i=0;i<4;++i)
  {
    chassis.wheel[i].set.curr = 0;
  }

  if (chassis.mode == CHASSIS_LOCK)
  {
    for (int i=0;i<4;++i)
  {
    chassis.wheel[i].set.curr = 0;

    chassis.rudder[i].set.curr = 0;
  }
  }
  
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ChassisSendCmd(void)
{
  CanCmdDjiMotor(2,0x200,0,chassis.wheel[0].set.curr,0,chassis.wheel[1].set.curr);
  CanCmdDjiMotor(1,0x200,chassis.wheel[2].set.curr,0,0,0);
  CanCmdDjiMotor(1,0x1FF,0,0,0,chassis.wheel[3].set.curr);

  CanCmdDjiMotor(2,0x1FF,0,0,chassis.rudder[0].set.curr,0);
  CanCmdDjiMotor(1,0x2FF,chassis.rudder[2].set.curr,chassis.rudder[3].set.curr,0,0);
  CanCmdDjiMotor(2,0x2FF,0,0,chassis.rudder[1].set.curr,0);

  ModifyDebugDataPackage(0,chassis.rudder[0].fdb.vel,"f0");
  ModifyDebugDataPackage(1,chassis.rudder[1].fdb.vel,"f1");
  ModifyDebugDataPackage(2,chassis.rudder[2].fdb.vel,"f2");
  ModifyDebugDataPackage(3,chassis.rudder[3].fdb.vel,"f3");

  ModifyDebugDataPackage(4,chassis.rudder[0].set.vel,"s0");
  ModifyDebugDataPackage(5,chassis.rudder[1].set.vel,"s1");
  ModifyDebugDataPackage(6,chassis.rudder[2].set.vel,"s2");
  ModifyDebugDataPackage(7,chassis.rudder[3].set.vel,"s3");

  ModifyDebugDataPackage(8,chassis.rudder[0].set.curr,"curr0");
  ModifyDebugDataPackage(9,chassis.rudder[1].set.curr,"curr1");
}
#endif
