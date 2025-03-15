/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-10      Harry_Wong      1.初始化uart2的传输结构体
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef UART1_TYPEDEF_H
#define UART1_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

#define COMMUNICATE_SOF ((uint8_t)0x5A)

#define Uart1_Test_ID ((uint8_t)0x01)
#define Rc_Data_ID ((uint8_t)0x02)
#define Gimbal_Data_ID ((uint8_t)0x03)

#define Uart1_Test_Duration ((uint8_t)100)

/*-------------------- Send & Receive --------------------*/
typedef struct 
{
  uint32_t Uart1_Test;
}LastSendTime_t;


//测试用数据包
typedef struct
{
  struct
  {
    uint8_t sof;   // 数据帧起始字节，固定值为 0xA5
    uint8_t len;   // 数据段长度
    uint8_t id;    // 数据段id
    uint8_t type;  // 数据段类型
    uint8_t crc;   // 数据帧头的 CRC8 
  } __attribute__((packed)) frame_header;  

  uint8_t time_stamp;  //数据段时间戳
  uint8_t crc16;       //crc16校验

  struct
  {
    uint8_t test_data;
  } __attribute__((packed)) data;
} __attribute__((packed)) Uart1_Test_s;

#endif
