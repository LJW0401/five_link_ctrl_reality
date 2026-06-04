/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       motor.c/h
  * @brief      电机相关部分定义
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     May-5-2024      Penguin         1. 添加dji电机的速度和位置控制
  *  V1.0.2     Apr-02-2024     Penguin         1. 添加了离线电机的扫描
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "motor.h"

#include "cmsis_os.h"
#include "pid.h"

#define MAX_MOTOR_NUM 30
Motor_s * MOTORS[MAX_MOTOR_NUM];
uint32_t MOTORS_USED_NUM = 0;

/**
 * @brief       电机初始化
 * @param[in]   p_motor 电机结构体
 * @param[in]   id 电机id
 * @param[in]   can 电机使用的can口
 * @param[in]   motor_type 电机种类
 * @param[in]   direction 电机旋转方向
 * @param[in]   reduction_ratio 电机减速比（如电机反馈已经处理完了可以不用注意）
 * @param[in]   mode 电机运行模式（通常关节电机需要用到）
 */
void MotorInit(
    Motor_s * p_motor, uint8_t id, uint8_t can, MotorType_e motor_type, int8_t direction,
    float reduction_ratio, uint16_t mode)
{
    p_motor->id = id;
    p_motor->can = can;
    p_motor->type = motor_type;
    p_motor->direction = direction;
    p_motor->reduction_ratio = reduction_ratio;
    p_motor->mode = mode;

    p_motor->offline = true;

    MOTORS[MOTORS_USED_NUM] = p_motor;  // 将电机添加到电机列表中
    MOTORS_USED_NUM++;
    if (MOTORS_USED_NUM > MAX_MOTOR_NUM) {
        MOTORS_USED_NUM = MAX_MOTOR_NUM;
    }
}

/**
 * @brief       按电机型号返回达妙 MIT 映射范围
 * @param[in]   type 电机型号
 * @return      该型号的 ±p_max/±v_max/±t_max
 * @note        取值为达妙官方各型号出厂默认 MIT 范围(P_MAX/V_MAX/T_MAX)。
 *              DM_8009 峰值扭矩 40N*m，故 t_max 远大于 4310；若在调试助手里
 *              改过这三个值，必须同步修改这里，否则映射会按比例错位。
 *              未知型号回退到一组保守的小范围。
 */
DmRange_s DmGetRange(MotorType_e type)
{
    switch (type) {
        case DM_8009:
            return (DmRange_s){.p_max = 12.5f, .v_max = 45.0f, .t_max = 54.0f};
        case DM_4310:
            return (DmRange_s){.p_max = 12.5f, .v_max = 30.0f, .t_max = 10.0f};
        case DM_4340:
            return (DmRange_s){.p_max = 12.5f, .v_max = 10.0f, .t_max = 28.0f};
        case DM_2325:
            // S2325：峰值扭矩5N*m、空载最高约58.6rad/s(560rpm)，量程按能力覆盖取整；
            // P/V/T_MAX 是 DM3520 驱动器寄存器(0x15/0x16/0x17)，须与调试助手实际值核对
            return (DmRange_s){.p_max = 12.5f, .v_max = 60.0f, .t_max = 5.0f};
        default:
            return (DmRange_s){.p_max = 12.5f, .v_max = 30.0f, .t_max = 10.0f};
    }
}

/**
 * @brief          扫描所有电机，检测是否有离线电机
 * @return         true: 有离线电机 false: 全部在线
 */
bool ScanOfflineMotor(void)
{
    for (uint32_t i = 0; i < MOTORS_USED_NUM; i++) {
        if (MOTORS[i]->offline) {
            return true;
        }
    }
    return false;
}

/************************ END OF FILE ************************/
