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

#include "example.h"

#if (CHASSIS_TYPE == CHASSIS_EXAMPLE)

#include "CAN_communication.h"
#include "custom_controller.h"
#include "string.h"

#define MOTOR_NUM 4

/*------------------------------ Macro Definition ------------------------------*/

#define EgMotorInit(index)                                                                    \
    MotorInit(                                                                                \
        &CHASSIS.motor[index], MOTOR_##index##_ID, MOTOR_##index##_CAN, MOTOR_##index##_TYPE, \
        MOTOR_##index##_DIRECTION, 1, MOTOR_##index##_MODE)

#define EgPidInit(index)                                                             \
    {                                                                                \
        float m##index##_pid_vel[3] = {                                              \
            KP_MOTOR_##index##_VEL, KI_MOTOR_##index##_VEL, KD_MOTOR_##index##_VEL}; \
        PID_init(                                                                    \
            &CHASSIS.pid.vel[index], PID_POSITION, m##index##_pid_vel,               \
            MAX_OUT_MOTOR_##index##_VEL, MAX_IOUT_MOTOR_##index##_VEL);              \
        float m##index##_pid_pos[3] = {                                              \
            KP_MOTOR_##index##_POS, KI_MOTOR_##index##_POS, KD_MOTOR_##index##_POS}; \
        PID_init(                                                                    \
            &CHASSIS.pid.pos[index], PID_POSITION, m##index##_pid_pos,               \
            MAX_OUT_MOTOR_##index##_POS, MAX_IOUT_MOTOR_##index##_POS);              \
    }
#define EgLpfInit(index) LowPassFilterInit(&CHASSIS.lpf.vel[index], M##index##_LPF_ALPHA)

/*------------------------------ Variable Definition ------------------------------*/

Chassis_s CHASSIS;

/*------------------------------ Function Definition ------------------------------*/

/******************************************************************/
/* Publish                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisPublish                             */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisPublish(void) {}

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      ChassisInit                                */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisInit(void)
{
    // #Motor init ---------------------
    EgMotorInit(0);
    EgMotorInit(1);
    EgMotorInit(2);
    EgMotorInit(3);
    // #PID init ---------------------
    EgPidInit(0);
    EgPidInit(1);
    EgPidInit(2);
    EgPidInit(3);
    // #LPF init ---------------------
    EgLpfInit(0);
    EgLpfInit(1);
    EgLpfInit(2);
    EgLpfInit(3);
    // #Initial value setting ---------------------
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));  // 目标量置零
    CHASSIS.mode = CHASSIS_MODE_OFF;
    CHASSIS.error_code = 0;
}

/******************************************************************/
/* HandleException                                                */
/*----------------------------------------------------------------*/
/* main function:      ChassisHandleException                     */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisHandleException(void) {}

/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisSetMode                             */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisSetMode(void) {}

/******************************************************************/
/* Observer                                                       */
/*----------------------------------------------------------------*/
/* main function:      ChassisObserver                            */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisObserver(void)
{
    uint8_t i;
    // 更新电机测量数据
    for (i = 0; i < MOTOR_NUM; i++) {
        GetMotorMeasure(&CHASSIS.motor[i]);
    }
    // 获取观测值
    float pos;
    for (i = 0; i < MOTOR_NUM; i++) {
        pos = CHASSIS.motor[i].fdb.pos;
        CHASSIS.fdb.Dpos[i] = pos - CHASSIS.fdb.pos[i];
        CHASSIS.fdb.pos[i] = pos;
        CHASSIS.fdb.vel[i] = CHASSIS.motor[i].fdb.vel;
    }
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:      ChassisReference                           */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisReference(void)
{
    int16_t rc_pos = 0, rc_vel = 0;
    fp32_deadline(Rc, 0, 180);
    // 位置环
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        CHASSIS.motor[i].set.value =
            PID_calc(&CHASSIS.pid.pos[i], CHASSIS.fdb.pos[i], CHASSIS.ref.pos[i]);
    }
    // 速度环
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        CHASSIS.motor[i].set.value =
            PID_calc(&CHASSIS.pid.vel[i], CHASSIS.fdb.vel[i], CHASSIS.ref.vel[i]);
    }
}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisConsole                    */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisConsole(void)
{
    uint8_t i;
    // 计算控制量
    for (i = 0; i < MOTOR_NUM; i++) {
        CHASSIS.motor[i].set.value =
            PID_calc(&CHASSIS.pid.joint[i], CHASSIS.fdb.joint[i].vel, CHASSIS.ref.joint[i].vel);
    }
}

/******************************************************************/
/* SendCmd                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisSendCmd                    */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisSendCmd(void)
{
    // clang-format off
    CanCmdDjiMotor(
        1, DJI_6020_MODE_VOLTAGE_1, 
        CHASSIS.motor[0].set.value,
        CHASSIS.motor[1].set.value, 
        CHASSIS.motor[2].set.value, 0);
    CanCmdDjiMotor(
        2, DJI_3508_MODE_CURRENT_1, 
        CHASSIS.motor[3].set.value,
        CHASSIS.motor[4].set.value, 
        CHASSIS.motor[5].set.value, 0);
    // clang-format on
}

#endif  // CHASSIS_TYPE
/*------------------------------ End of File ------------------------------*/
