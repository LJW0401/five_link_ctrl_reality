/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
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

#include "shoot.h"
#include "shoot_task.h"
#include "shoot_fric_trigger.h"
#include "shoot_fric_trigger.c"

shoot_bullet_data shoot_bullet;
shoot_1_bullet shoot_1;
static void shoot_bullet_control(void)
{
  if(SHOOT.mode == LOAD_STOP) break;
  if(SHOOT.trigger_motor.fdb.pos == shoot_bullet.last_pos)
  {
    shoot_bullet.time_list ++;
  }
  if(shoot_bullet.time_list >=500)
  {
    shoot_bullet.flag = 1;
    shoot_bullet.time_list = 0;
    SHOOT.trigger_motor.set.pos = SHOOT.trigger_motor.fdb.pos - PI;
  }
  if(shoot_bullet.flag == 1 && SHOOT.trigger_motor.fdb.pos == SHOOT.trigger_motor.set.pos)
  {
    flag = 0;
  }
}//防堵转，给出参数

static void shoot_LOAD_1_BULLET(void)
{
  if(shoot_bullet.flag == 1) break;//防堵转模式不进
  if(flag == 0)
  {
    SHOOT.trigger_motor.set.pos = SHOOT.trigger_motor.fdb.pos + SHOOT.dangle;//目标角度设定
  }
  if(flag == 1 && SHOOT.trigger_motor.fdb.pos == SHOOT.trigger_moter.set.pos ) flag = 0;
}//单发

static void shoot_LOAD_BURSTFIRE(void)
{
  if(shoot_bullet.flag == 1) break;//防堵转模式不进
  SHOOT.trigger_motor.set.vel = trigger_SPEED;
}//连发