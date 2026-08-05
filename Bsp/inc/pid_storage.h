#ifndef __PID_STORAGE_H
#define __PID_STORAGE_H
#include "main.h"

/* 把当前四路PID(Id/Iq/速度/位置)的kp/ki/kd/target写入片内Flash最后一个扇区(Sector 7),
   掉电后可通过PidStorage_Load()恢复,避免每次上电都要靠上位机重新调参。
   仅允许在FOC_MODE_IDLE下调用(擦除128KB扇区最坏耗时约2s,期间flash总线被整体占用,
   电机运行中调用会让FOC ISR失步),由调用方(uart_task.c)负责校验模式。 */
uint8_t PidStorage_Save(void);

/* 从Flash读取并校验PID参数,校验通过则应用到PID_Current_D/Q/PID_Speed/PID_Position并返回1;
   Flash内容为空/损坏(未保存过或校验失败)则不改动现有PID参数,返回0。 */
uint8_t PidStorage_Load(void);

#endif
