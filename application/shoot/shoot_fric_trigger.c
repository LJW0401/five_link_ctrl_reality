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
#include "motor.h"
#include "shoot_fric_trigger.h"
#include "shoot.c"
#include "shoot.h"
int last_pos;
//#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

static Shoot_s SHOOT = {
  .mode = LOAD_STOP,
};
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void InitShoot(void) 
{ 
  SHOOT.rc = get_remote_control_point(); 
  SHOOT.mode = LOAD_STOP;
  SHOOT.state = FRIC_NOT_READY;
//遥控器，射击，摩擦轮
  for(int i=0;i<4;i++)
  {
    GetMotorMeasure(SHOOT.fric_motor[i]);
  }
  GetMotorMeasure(SHOOT.trigger_motor);//读取数据
//摩擦轮电机，拨弹盘电机
  SHOOT.shoot_frequency = 0;
  SHOOT.shoot_speed = 0;
  SHOOT.dangle = 0;
//射频，射速，拨弹盘单次转动角度
  PID_init(trigger_pid,PID_POSITION, SHOOT.trigger_PID[3] , shoot_trigger_max_out, shoot_trigger_max_iout);
  for(int i=0;i<4;i++)
  {
    PID_init(fric_pid[4], PID_POSITION, SHOOT.fric_PID[3] , shoot_fricr_max_out, shoot_fric_max_iout);
  }
  //PID初始化
  shoot_bullet.last_pos = SHOOT.trigger_motor.fdb.pos;
  shoot_bullet.flag = 0;
  shoot_bullet.time_list = 0;
  //防堵转结构体
  shoot_1.flag = 0;
  //单发结构体
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void SetShootMode(void)
{
    if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) {
        SHOOT.mode = LOAD_STOP;
    } else if (switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) {
        SHOOT.mode = LOAD_1_BULLET;
    } else if (switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) {
        SHOOT.mode = LOAD_BURSTFIRE;
    }
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ShootObserver(void) //稍后再写
{
  for(int i=0;i<4;i++)
  {
    GetMotorMeasure(SHOOT.fric_motor[i]);
  }
  GetMotorMeasure(SHOOT.trigger_motor);//读取数据
  last_pos = SHOOT.trigger_motor.fdb.pos;
}
//摩擦轮速度、拨弹盘速度，拨弹盘角度，
/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ShootReference(void) 
{
  if(SHOOT.mode == LOAD_STOP)
  {
    SHOOT.shoot_frequency = 0;
    SHOOT.shoot_speed = 0;
    SHOOT.dangle = 0;
  }
  else if(SHOOT.mode == LOAD_1_BULLET)
  {
    shoot_bullet_control();
    shoot_LOAD_1_BULLET();
  }
  else if(SHOOT.mode == LOAD_BURSTFIRE)
  {
    shoot_bullet_control();
    shoot_LOAD_BURSTFIRE();
  }
  //目标量

}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ShootConsole(void)
{
  if(shoot_bullet.flag == 0 && SHOOT.mode == LOAD_BURSTFIRE)//连发速度控制
  {
    for(int i=0; i<4; i++)
    {
      SHOOT.fric_motor[i].set.value = PID_calc(SHOOT.fric_pid[i], SHOOT.fric_motor[i].fdb.vel, SHOOT.fric_motor[i].set.vel);
    }
    SHOOT.trigger_motor.set.value = PID_calc(SHOOT.trigger_pid, SHOOT.trigger_motor.fdb.vel, SHOOT.trigger_motor.set.vel);
  }
  else//其他都是角度控制
  {
    for(int i=0; i<4; i++)
    {
      SHOOT.fric_motor[i].set.value = PID_calc(SHOOT.fric_pid[i], SHOOT.fric_motor[i].pos, SHOOT.fric_motor[i].set.pos);
    }
    SHOOT.trigger_motor.set.value = PID_calc(SHOOT.trigger_pid, SHOOT.trigger_motor.fdb.pos, SHOOT.trigger_motor.set.pos);
  }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendShootCmd(void) //晚点再找发送的代码
{
  for(int i=0;i<4;i++)
  {
    GetMotorMeasure(SHOOT.fric_motor[i]);
  }
  GetMotorMeasure(SHOOT.trigger_motor);//读取数据
}

#endif  // SHOOT_TYPE == SHOOT_FRIC
