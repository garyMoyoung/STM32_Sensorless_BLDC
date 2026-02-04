#include "main.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "Algorithmic.h"
/**
 * @brief 边缘检测器初始化
 * @param edge: 边缘检测器结构体指针
 * @param initial_state: 初始信号状态 (0 或 1)
 */
void Edge_Detector_Init(Edge_Detector_t *edge, uint8_t initial_state)
{
    edge->signal_last = initial_state;
    edge->rising_pulse = 0;
    edge->falling_pulse = 0;
}

/**
 * @brief 边缘检测更新 - 检测上升沿和下降沿
 * @param edge: 边缘检测器结构体指针
 * @param signal: 当前输入信号 (0 或 1)
 * @retval 返回脉冲信息 (bit0=上升沿, bit1=下降沿)
 *         0: 无脉冲
 *         1: 上升沿脉冲
 *         2: 下降沿脉冲
 *         3: 同时出现上升和下降沿（不可能，返回信息）
 */
uint8_t Edge_Detector_Update(Edge_Detector_t *edge, uint8_t signal)
{
    uint8_t pulse_info = 0;
    
    // 上升沿检测：当前为1，前一个为0
    if (signal == 1 && edge->signal_last == 0) {
        edge->rising_pulse = 1;
        pulse_info |= 0x01;  // bit0 置1表示上升沿
    } else {
        edge->rising_pulse = 0;
    }
    
    // 下降沿检测：当前为0，前一个为1
    if (signal == 0 && edge->signal_last == 1) {
        edge->falling_pulse = 1;
        pulse_info |= 0x02;  // bit1 置1表示下降沿
    } else {
        edge->falling_pulse = 0;
    }
    
    // 保存当前信号状态用于下次比较
    edge->signal_last = signal;
    
    return pulse_info;
}

void T_FlipFlop_Init(T_FlipFlop_t *t_ff, uint8_t initial_state)
{
    t_ff->Q_state = initial_state;
    t_ff->clk_last = 0;
}
uint8_t T_FlipFlop_Update(T_FlipFlop_t *t_ff, uint8_t clk, uint8_t T)
{
    // 捕捉时钟上升沿 (从0到1的跳变)
    if (clk == 1 && t_ff->clk_last == 0) {
        // T为1时翻转状态，T为0时保持状态
        if (T == 1) {
            t_ff->Q_state = ~t_ff->Q_state & 0x01; // 翻转状态
        }
    }
    
    // 保存当前时钟状态用于下次比较
    t_ff->clk_last = clk;
    
    return t_ff->Q_state;
}

/**
 * @brief 初始化ADC滤波器
 * @param filter: 滤波器结构体指针
 */
void ADC_Filter_Init(ADC_Filter_t *filter)
{
    filter->index = 0;
    filter->sum = 0;
    filter->count = 0;
    filter->is_full = false;
    memset(filter->buffer, 0, sizeof(filter->buffer));
}

/**
 * @brief 滑动平均滤波
 * @param filter: 滤波器结构体指针
 * @param new_value: 新的ADC值
 * @retval 滤波后的值
 */
uint16_t ADC_Moving_Average_Filter(ADC_Filter_t *filter, uint16_t new_value)
{
    // 如果缓冲区满了，减去要被替换的值
    if (filter->is_full) {
        filter->sum -= filter->buffer[filter->index];
    } else {
        filter->count++;
        if (filter->count >= ADC_FILTER_SIZE) {
            filter->is_full = true;
        }
    }
    
    // 添加新值
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;
    
    // 更新索引
    filter->index = (filter->index + 1) % ADC_FILTER_SIZE;
    
    // 返回平均值
    return filter->sum / (filter->is_full ? ADC_FILTER_SIZE : filter->count);
}

/**
 * @brief 一阶低通滤波 (IIR滤波)
 * @param last_value: 上次滤波值
 * @param new_value: 新的ADC值
 * @param alpha: 滤波系数 (0-1, 越小滤波越强)
 * @retval 滤波后的值
 */
float Low_Pass_Filter(float last_value, uint16_t new_value, float alpha)
{
    return alpha * new_value + (1.0f - alpha) * last_value;
}


/**
 * @brief 中位值滤波
 * @param buffer: 数据缓冲区
 * @param size: 缓冲区大小
 * @retval 中位值
 */
uint16_t ADC_Median_Filter(uint16_t *buffer, uint8_t size)
{
    uint16_t temp[size];
    memcpy(temp, buffer, size * sizeof(uint16_t));
    
    // 冒泡排序
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - 1 - i; j++) {
            if (temp[j] > temp[j + 1]) {
                uint16_t swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    
    return temp[size / 2];  // 返回中位值
}


void Kalman_Filter_Init(Kalman_Filter_t *kf, float Q, float R, float P, float initial_value)
{
    kf->Q = Q;
    kf->R = R;
    kf->P = P;
    kf->x = initial_value;
}

/**
 * @brief 卡尔曼滤波
 */
float Kalman_Filter_Update(Kalman_Filter_t *kf, float measurement)
{
    // 预测步骤
    kf->P = kf->P + kf->Q;
    
    // 更新步骤
    kf->K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + kf->K * (measurement - kf->x);
    kf->P = (1 - kf->K) * kf->P;
    
    return kf->x;
}
