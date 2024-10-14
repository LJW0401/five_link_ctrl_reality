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
//角度：PI到-PI
#include "shoot.h"
#include "shoot_task.h"
#include "shoot_fric_trigger.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

Shoot_s SHOOT;
shoot_bullet_data shoot_bullet;
shoot_1_bullet shoot_1;

void angle_judgement(float *angle,float add,int *is_circle)
{
	if(*angle + add > PI)
	{
		*angle += add - PI;
		*is_circle = 1;
	}
	if(*angle + add < PI)
	{
		*angle += add + PI;
		*is_circle = -1;
	}
	else
	{
		*angle += add;
		*is_circle = 0 ;
	}
}



void shoot_bullet_control(void)
{
  if(SHOOT.mode == LOAD_STOP) return;
  if(SHOOT.trigger_motor.fdb.pos == shoot_bullet.last_pos)
  {
    shoot_bullet.time_list ++;//时间累加
  }
  if(shoot_bullet.time_list >=500)
  {
    shoot_bullet.flag = 1;
    shoot_bullet.time_list = 0;
    SHOOT.trigger_motor.set.pos = SHOOT.trigger_motor.fdb.pos - PI;//防堵转
  }
  if(shoot_bullet.flag == 1 && SHOOT.trigger_motor.fdb.pos == SHOOT.trigger_motor.set.pos)
  {
    shoot_bullet.flag = 0;//出口
  }
}//防堵转，给出参数

void if_prepared(void)
{
	if(SHOOT.trigger_motor.fdb.vel == trigger_SPEED)
	{
		SHOOT.state = FRIC_READY;
	}	
  if(SHOOT.trigger_motor.fdb.vel == trigger_SPEED)
	{
		SHOOT.state = FRIC_NOT_READY;
	}		
}


void shoot_LOAD_1_BULLET(void)
{
  if(shoot_bullet.flag == 1 || SHOOT.state == FRIC_NOT_READY) return;//防堵转模式不进
  if(shoot_1.flag == 0 && SHOOT.state == FRIC_READY)
  {
		angle_judgement(&SHOOT.trigger_motor.set.pos ,SHOOT.dangle, &SHOOT.is_circle);//目标角度设定
  }
  if(shoot_1.flag == 1 && SHOOT.trigger_motor.fdb.pos == SHOOT.trigger_motor.set.pos ) shoot_1.flag = 0;
}//单发

void shoot_LOAD_BURSTFIRE(void)
{
  if(shoot_bullet.flag == 1|| SHOOT.state == FRIC_NOT_READY) return;//防堵转模式不进
  if(SHOOT.state == FRIC_READY)
	{
		SHOOT.trigger_motor.set.vel = trigger_SPEED;
	}
}//连发


   

