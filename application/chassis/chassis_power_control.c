/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_power_control.c/h
  * @brief      底盘功率控制
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
 #include "chassis_power_control.h"
 #include "referee.h"
 #include "arm_math.h"
 #include "detect_task.h"
 #include "usb_debug.h"
 #include "motor.h"
 
 #define K1 1.23e-07f
 #define K2 1.453e-07f
 #define TORQUE_DETA 1.99688994e-6f
 #define Constant 4.081f
 #define OMEGA_TO_RPM 9.55f
 void Power_control(Motor_s *motor)
 { 
   float initial_power[4];
   float initial_total_power=0;
   float scale_power[4];
   for(int i=0;i<4;i++)
   {
    initial_power[i]=motor[i].set.curr*motor[i].fdb.vel*OMEGA_TO_RPM*TORQUE_DETA +K1*motor[i].set.curr*motor[i].set.curr+K2*motor[i].fdb.vel*OMEGA_TO_RPM*motor[i].fdb.vel*OMEGA_TO_RPM+Constant;
    if(initial_power[i]<0)continue;
    initial_total_power+=initial_power[i];
   }
   if(initial_total_power>(robot_status.chassis_power_limit-5))
   {
     float scale=(robot_status.chassis_power_limit-5)/initial_total_power;
      for(int i=0;i<4;i++)
     {
        scale_power[i]=scale*initial_power[i];
        if(scale_power[i]<0)continue;
        float a=K1;
        float b=TORQUE_DETA*motor[i].fdb.vel*OMEGA_TO_RPM;
        float c=K2*motor[i].fdb.vel*OMEGA_TO_RPM*motor[i].fdb.vel*OMEGA_TO_RPM-scale_power[i]+Constant;
        float inside=b*b-4*a*c;
 
        if(inside<0)
        {
          continue;
        }
        else if(motor[i].set.curr>0)
        {
           float temp=(-b+sqrt(inside))/(2*a);
           if(temp>16000)
           {
             motor[i].set.curr=16000;
           }
           else
           {
             motor[i].set.curr=temp;
           }
        }
        else
        {
         float temp=(-b-sqrt(inside))/(2*a);
         if(temp<-16000)
         {
           motor[i].set.curr=-16000;
         }
         else
         {
             motor[i].set.curr=temp;
         }
        }
     }
   }
   ModifyDebugDataPackage(7,initial_total_power,"total_power");
 }
