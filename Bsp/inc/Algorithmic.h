#ifndef __ALGORITHMIC_H__
#define __ALGORITHMIC_H__

#include "main.h"
#include <math.h>
#define ADC_FILTER_SIZE 10

typedef struct {
    uint8_t Q_state;   // 当前Q输出状态
    uint8_t clk_last;  // 上一次时钟值
} T_FlipFlop_t;
typedef struct {
    uint8_t signal_last;      // 上一次的信号状态
    uint8_t rising_pulse;     // 上升沿脉冲输出
    uint8_t falling_pulse;    // 下降沿脉冲输出
} Edge_Detector_t;

typedef struct {
    uint16_t buffer[ADC_FILTER_SIZE];
    uint8_t index;
    uint32_t sum;
    uint8_t count;
    bool is_full;
} ADC_Filter_t;

// 卡尔曼滤波结构体
typedef struct {
    float Q;  // 过程噪声
    float R;  // 测量噪声
    float P;  // 估计误差协方差
    float K;  // 卡尔曼增益
    float x;  // 估计值
} Kalman_Filter_t;
void Edge_Detector_Init(Edge_Detector_t *edge, uint8_t initial_state);
uint8_t Edge_Detector_Update(Edge_Detector_t *edge, uint8_t signal);
void T_FlipFlop_Init(T_FlipFlop_t *t_ff, uint8_t initial_state);
uint8_t T_FlipFlop_Update(T_FlipFlop_t *t_ff, uint8_t clk, uint8_t T);
void ADC_Filter_Init(ADC_Filter_t *filter);
uint16_t ADC_Moving_Average_Filter(ADC_Filter_t *filter, uint16_t new_value);
float Low_Pass_Filter(float last_value, uint16_t new_value, float alpha);
uint16_t ADC_Median_Filter(uint16_t *buffer, uint8_t size);
void Kalman_Filter_Init(Kalman_Filter_t *kf, float Q, float R, float P, float initial_value);
float Kalman_Filter_Update(Kalman_Filter_t *kf, float measurement);

#endif
