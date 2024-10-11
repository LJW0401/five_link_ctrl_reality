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
#include "IMU.h"
#include "custom_controller.h"
#include "detect_task.h"
#include "macro_typedef.h"
#include "remote_control.h"
#include "string.h"

#define MOTOR_NUM 4

// clang-format off
#define MOTOR_ERROR_OFFSET   ((uint8_t)1 << 0)  // 电机错误偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 1)  // dbus错误偏移量
// clang-format on

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

void ChassisHandleException(void)
{
    if (toe_is_error(DBUS_TOE)) {
        CHASSIS.error_code |= DBUS_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~DBUS_ERROR_OFFSET;
    }
}

/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisSetMode                             */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisSetMode(void)
{
    if (CHASSIS.error_code != 0) {
        CHASSIS.mode = CHASSIS_MODE_OFF;
    }

    if (GetDt7RcSw(CHASSIS_MODE_CHANNEL) == RC_SW_UP) {
        CHASSIS.mode = CHASSIS_MODE_POS;
    } else if (GetDt7RcSw(CHASSIS_MODE_CHANNEL) == RC_SW_MID) {
        CHASSIS.mode = CHASSIS_MODE_VEL;
    } else {
        CHASSIS.mode = CHASSIS_MODE_OFF;
    }
}

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
    rc_pos = GetDt7RcCh(CHASSIS_POS_CHANNEL) * M_PI * 4.0f;
    rc_vel = fp32_deadline(GetDt7RcCh(CHASSIS_VEL_CHANNEL) * M_PI * 2.0f, -0.01f, 0.01f);

    switch (CHASSIS.mode) {
        case CHASSIS_MODE_POS: {
            CHASSIS.ref.pos[0] = GetImuAngle(AX_PITCH);
            for (uint8_t i = 1; i < MOTOR_NUM; i++) {
                CHASSIS.ref.pos[i] = rc_pos;
            }
        } break;
        case CHASSIS_MODE_VEL: {
            for (uint8_t i = 0; i < MOTOR_NUM; i++) {
                CHASSIS.ref.vel[i] = rc_vel;
            }
        } break;
        case CHASSIS_MODE_OFF:
        default: {
            for (uint8_t i = 0; i < MOTOR_NUM; i++) {
                CHASSIS.ref.vel[i] = 0;
            }
        }
    }
}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisConsole                             */
/* auxiliary function: ConsolePos                                 */
/*                     ConsoleVel                                 */
/*                     ConsoleOff                                 */
/******************************************************************/

void ConsolePos(void)
{
    float ref_vel = 0;
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        ref_vel = PID_calc(&CHASSIS.pid.pos[i], CHASSIS.fdb.pos[i], CHASSIS.ref.pos[i]);
        CHASSIS.motor[i].set.value = PID_calc(&CHASSIS.pid.vel[i], CHASSIS.fdb.vel[i], ref_vel);
    }
}

void ConsoleVel(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        CHASSIS.motor[i].set.value =
            PID_calc(&CHASSIS.pid.vel[i], CHASSIS.fdb.vel[i], CHASSIS.ref.vel[i]);
    }
}

void ConsoleOff(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        CHASSIS.motor[i].set.value = 0;
    }
}

void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_MODE_POS: {
            ConsolePos();
        } break;
        case CHASSIS_MODE_VEL: {
            ConsoleVel();

        } break;
        case CHASSIS_MODE_OFF:
        default: {
            ConsoleOff();
        }
    }
}

/******************************************************************/
/* SendCmd                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisSendCmd                             */
/* auxiliary function: None                                       */
/******************************************************************/

void ChassisSendCmd(void)
{
    // clang-format off
    CanCmdDjiMotor(
        1, DJI_6020_MODE_VOLTAGE_1, 
        CHASSIS.motor[0].set.value,
        CHASSIS.motor[1].set.value, 
        0, 0);
    CanCmdDjiMotor(
        2, DJI_3508_MODE_CURRENT_1, 
        CHASSIS.motor[2].set.value,
        CHASSIS.motor[3].set.value, 
        0, 0);
    // clang-format on
}

#endif  // CHASSIS_TYPE
/*------------------------------ End of File ------------------------------*/
