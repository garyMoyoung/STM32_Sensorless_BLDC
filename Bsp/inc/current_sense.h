#ifndef __CURRENT_SENSE_H
#define __CURRENT_SENSE_H

#include "main.h"

/*
 * 电流采样参数:
 * INA240A2QPWRQ1, 增益50V/V, VS供电为+3.3V(不是母线电压)。
 * 采样电阻实物焊接为 5mΩ(原理图丝印 R89/R173~R177 标注 1mΩ,与实物不符,以实物为准)。
 */
#define CURRENT_ADC_REF_VOLTAGE      3.3f
#define CURRENT_ADC_FULL_SCALE       4096.0f
#define CURRENT_INA240_GAIN          50.0f
#define CURRENT_SHUNT_RESISTOR_OHM   0.005f
#define CURRENT_ADC_TO_AMP           (CURRENT_ADC_REF_VOLTAGE / CURRENT_ADC_FULL_SCALE / (CURRENT_INA240_GAIN * CURRENT_SHUNT_RESISTOR_OHM))
#define CURRENT_OFFSET_DEFAULT_ADC   2048U
/* 标定出的零电流偏置若偏离 CURRENT_OFFSET_DEFAULT_ADC 超过此阈值(约0.48V),视为该相模拟前端异常 */
#define CURRENT_OFFSET_FAULT_TOL_ADC 600U

extern uint16_t ad_val_orig[3];
extern uint16_t current_adc_offset[3];

/* 电流采样诊断量: 原始ADC计数/换算电压/零点异常标志(bit0=A,bit1=B,bit2=C), 供上位机 $RAW / $TEL 读取 */
extern uint16_t Diag_RawAdc[3];
extern float Diag_RawVolt[3];
extern uint8_t Diag_CurrentFault;

void Current_ReadRaw(uint16_t raw_adc[3]);
void Current_CalibrateOffset(uint16_t sample_count);
void Current_read(void);

#endif
