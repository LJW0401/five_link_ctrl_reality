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
#include "shoot.h"
#include "CAN_receive.h"
#include "CAN_cmd_dji.h"


int last_pos;
#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)



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
    GetMotorMeasure(&SHOOT.fric_motor[i]);
  }
  GetMotorMeasure(&SHOOT.trigger_motor);//读取数据
//摩擦轮电机，拨弹盘电机
  SHOOT.shoot_frequency = 0;
  SHOOT.shoot_speed = 0;
  SHOOT.dangle = 0;
//射频，射速，拨弹盘单次转动角度
	SHOOT.angle_PID[0] = shoot_angle_kp;
	SHOOT.angle_PID[1] = shoot_angle_ki;
	SHOOT.angle_PID[2] = shoot_angle_kd;
	SHOOT.speed_PID[0] = shoot_speed_kp;
	SHOOT.speed_PID[1] = shoot_speed_ki;
	SHOOT.speed_PID[2] = shoot_speed_kd;
  PID_init(&SHOOT.angle_pid,PID_POSITION, SHOOT.angle_PID, shoot_angle_max_out, shoot_angle_max_iout);
  PID_init(&SHOOT.speed_pid,PID_DELTA, SHOOT.speed_PID, shoot_speed_max_out, shoot_speed_max_iout);
	//PID初始化
	SHOOT -> is_circle = 0;
  //角度判断常量
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
    if (switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])|| switch_is_down(SHOOT.rc->rc.s(SHOOT_Control_CHANNEL)))
    {
      SHOOT.mode = LOAD_STOP;
    }//键位往下
    else if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) 
    {
        SHOOT.mode = LOAD_BURSTFIRE;
    } //
    else if (switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
    {
        SHOOT.mode = LOAD_1_BULLET;
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
    GetMotorMeasure(&SHOOT.fric_motor[i]);
  }
  GetMotorMeasure(&SHOOT.trigger_motor);//读取数据
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
	if_prepared();
	
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
    for(int i=0; i<2; i++)
    {
      SHOOT.fric_motor[i].set.value = PID_calc(&SHOOT.speed_pid, SHOOT.fric_motor[i].fdb.vel, SHOOT.fric_motor[i].set.vel);
    }
    SHOOT.trigger_motor.set.value = PID_calc(&SHOOT.speed_pid, SHOOT.trigger_motor.fdb.vel, SHOOT.trigger_motor.set.vel);
  }
  else//其他都是角度控制
  {
    for(int i=0; i<2; i++)
    {
      SHOOT.fric_motor[i].set.value = PID_calc(&SHOOT.angle_pid, SHOOT.fric_motor[i].fdb.pos, SHOOT.fric_motor[i].set.pos);
    }
    SHOOT.trigger_motor.set.value = PID_calc(&SHOOT.angle_pid, SHOOT.trigger_motor.fdb.pos, SHOOT.trigger_motor.set.pos);
  }
}

/*-------------------- Cmd --------------------*/
//GetMotorMeasure
/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendShootCmd(void) //晚点再找发送的代码
{

  //CanCmdDjiMotor(2, 0x200, SHOOT.fric_motor[0].set.value, SHOOT.fric_motor.set.value[1] , 0, 0);//发送摩擦轮电流
	CanCmdDjiMotor(2, 0x200, 1500, 1500, 1500, 1500);//发送拨弹盘电流
  //CanCmdDjiMotor(2, 0x200,  SHOOT.trigger_motor.set.value, 0, 0, 0);//发送拨弹盘电流
}

#endif  // SHOOT_TYPE == SHOOT_FRIC
