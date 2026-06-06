// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "signal_generator.h"
#include "remote_control.h"
#include "usb_debug.h"
#include "chassis_balance.h"

const Sbus_t* SBUS;
const RC_ctrl_t* RC_CTRL;
extern Chassis_s CHASSIS;

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);

    SBUS = get_sbus_point();
    RC_CTRL = get_remote_control_point();

    while (1) {
        ModifyDebugDataPackage(0, CHASSIS.fdb.leg_state[0].phi, "phi");
        ModifyDebugDataPackage(1, CHASSIS.fdb.leg_state[0].phi_dot, "dphi");
        ModifyDebugDataPackage(2, CHASSIS.fdb.leg_state[0].theta, "theta");
        ModifyDebugDataPackage(3, CHASSIS.fdb.leg_state[0].theta_dot, "dtheta");
        ModifyDebugDataPackage(4, CHASSIS.fdb.leg_state[0].x, "x");
        ModifyDebugDataPackage(5, CHASSIS.fdb.leg_state[0].x_dot, "dx");

        ModifyDebugDataPackage(6, CHASSIS.cmd.leg[0].wheel.T, "wheel_T");
        ModifyDebugDataPackage(7, CHASSIS.cmd.leg[0].rod.Tp, "rod_Tp");


        vTaskDelay(1);
    }
}
