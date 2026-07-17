#include "current_sense.h"
#include "adc.h"

uint16_t ad_val_orig[3] = {0U, 0U, 0U};
uint16_t current_adc_offset[3] = {
  CURRENT_OFFSET_DEFAULT_ADC,
  CURRENT_OFFSET_DEFAULT_ADC,
  CURRENT_OFFSET_DEFAULT_ADC
};

uint16_t Diag_RawAdc[3] = {0U, 0U, 0U};
float Diag_RawVolt[3] = {0.0f, 0.0f, 0.0f};
uint8_t Diag_CurrentFault = 0U;

extern Iabc_Struct Iabc_M0;

/*
 * 注入通道->物理引脚映射(adc.c: rank1=CH2/PA2, rank2=CH3/PA3, rank3=CH4/PA4):
 *   raw_adc[0] (Ia) <- InjectedRank3 <- ADC1_IN4 <- PA4
 *   raw_adc[1] (Ib) <- InjectedRank2 <- ADC1_IN3 <- PA3
 *   raw_adc[2] (Ic) <- InjectedRank1 <- ADC1_IN2 <- PA2
 * 这是当前代码假设的相序对应关系。对照 FOC_DRIVER_SCH 原理图,TIM2_CH2(即SVPWM第一相/tcm1)
 * 经 M0_IN1 驱动 HO1/LO1 产生 VS1,电流经分流电阻到达电机 M0_A 端,由 U50 采样输出 M0_INA_OUT1,
 * 这组信号是否最终接到 PA2 还是 PA4,取决于 CN20 连接器的具体引脚序,原理图导出文本未能完全
 * 确认逐脚对应关系。如果拿到板子后发现 Ia/Ic 对调(例如只在物理A相通电流时,固件却报告在Ic上),
 * 只需调整本文件里这三行下标,不影响其余算法。
 */
void Current_ReadRaw(uint16_t raw_adc[3])
{
    raw_adc[2] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
    raw_adc[1] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
    raw_adc[0] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
}

void Current_CalibrateOffset(uint16_t sample_count)
{
    uint32_t sum[3] = {0U, 0U, 0U};
    uint16_t raw_adc[3] = {0U, 0U, 0U};
    uint16_t valid_samples = 0U;

    if (sample_count == 0U)
    {
        sample_count = 1U;
    }

    for (uint16_t i = 0U; i < sample_count; i++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 2U) == HAL_OK)
        {
            Current_ReadRaw(raw_adc);
            sum[0] += raw_adc[0];
            sum[1] += raw_adc[1];
            sum[2] += raw_adc[2];
            valid_samples++;
        }
    }

    if (valid_samples > 0U)
    {
        uint16_t new_offset[3];
        uint8_t ch;

        new_offset[0] = (uint16_t)(sum[0] / valid_samples);
        new_offset[1] = (uint16_t)(sum[1] / valid_samples);
        new_offset[2] = (uint16_t)(sum[2] / valid_samples);

        Diag_CurrentFault = 0U;
        for (ch = 0U; ch < 3U; ch++)
        {
            int32_t dev = (int32_t)new_offset[ch] - (int32_t)CURRENT_OFFSET_DEFAULT_ADC;
            if ((dev > (int32_t)CURRENT_OFFSET_FAULT_TOL_ADC) || (dev < -(int32_t)CURRENT_OFFSET_FAULT_TOL_ADC))
            {
                /* 标定值偏离过大,判定该相模拟前端异常,零点仍用默认值,不采信可疑标定结果 */
                Diag_CurrentFault |= (uint8_t)(1U << ch);
            }
            else
            {
                current_adc_offset[ch] = new_offset[ch];
            }
        }
    }
}

void Current_read(void)
{
    uint8_t ch;

    Current_ReadRaw(ad_val_orig);
    Iabc_M0.Ia = ((float)ad_val_orig[0] - (float)current_adc_offset[0]) * CURRENT_ADC_TO_AMP;
    Iabc_M0.Ib = ((float)ad_val_orig[1] - (float)current_adc_offset[1]) * CURRENT_ADC_TO_AMP;
    Iabc_M0.Ic = ((float)ad_val_orig[2] - (float)current_adc_offset[2]) * CURRENT_ADC_TO_AMP;

    for (ch = 0U; ch < 3U; ch++)
    {
        Diag_RawAdc[ch] = ad_val_orig[ch];
        Diag_RawVolt[ch] = (float)ad_val_orig[ch] * (CURRENT_ADC_REF_VOLTAGE / CURRENT_ADC_FULL_SCALE);
    }
}
