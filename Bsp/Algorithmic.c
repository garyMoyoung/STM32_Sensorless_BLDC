#include "main.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "Algorithmic.h"

extern float Iab_fore_New[2] , Iab_fore_Last[2];
extern float Iab[2] , Iab_Last[2];
extern float Idq[2];
extern float Udq[2];
extern float Uab[2];
extern double sincos[2];
extern float h, Vab[2] , Vab_Filter[2];
extern float Eab[2];
extern float R, Ld, Lq, T, flux;
extern float We;
extern uint16_t Start_Flag, Start_CNT;
extern float Speed , Speed_sum , Speed_New;
extern float Theta_fore_New , Theta_fore_Last , We_fore;
extern float PLL_Kp, PLL_Ki;
extern float Vab_alpha;
void Idq_LPF_Filter(float*data)
{
	static float Last_x , Last_y;
	static float alpha = 0.2583;
	data[0] = alpha*data[0]+(1-alpha)*Last_x;
	data[1] = alpha*data[1]+(1-alpha)*Last_y;
	Last_x = data[0];
	Last_y = data[1];
}

float LPF_Filter(float data)
{
	static float Last_y;
	static float alpha = 0.3583;
	data=alpha*data+(1-alpha)*Last_y;
	Last_y = data;
	return data;
}

float Vab_LPF_Filter(float data)
{
	static float Last_y;
	data=Vab_alpha*data+(1-Vab_alpha)*Last_y;
	Last_y = data;
	return data;
}
float We_Filter(float data)
{
	static float y , Last_y;
	static float We_alpha = 0.00028;
	y=We_alpha*data+(1-We_alpha)*Last_y;
	Last_y = y;
	return y;
}
float Limit(float value,float lim)
{
	float tem = value;
	if(fabsf(tem)>lim)
	{
		if(tem>0)
		{	
		  tem=lim;
		}
		else
		{
		  tem=-lim;	
		}
	}
	return tem;
}
void SMO(void)
{

	if(Iab_fore_Last[0] - Iab_Last[0] > 1.0f)
		Vab[0] = h;
	else if(Iab_fore_Last[0] - Iab_Last[0] < -1.0f)
		Vab[0] = -h;
	else
		Vab[0] = h*(Iab_fore_Last[0] - Iab_Last[0]);

	if(Iab_fore_Last[1] - Iab_Last[1] > 1.0f)
		Vab[1] = h;
	else if(Iab_fore_Last[1] - Iab_Last[1] < -1.0f)
		Vab[1] = -h;
	else
		Vab[1] = h*(Iab_fore_Last[1] - Iab_Last[1]);

	Vab_Filter[0] = Vab_LPF_Filter(Vab[0]);
	Vab_Filter[1] = Vab_LPF_Filter(Vab[1]);
	
	Iab_fore_New[0] = (1-R*T/Ld)*Iab_fore_Last[0]+T*Uab[0]/Ld-T*Vab_Filter[0]/Ld;
	Iab_fore_New[1] = (1-R*T/Ld)*Iab_fore_Last[1]+T*Uab[1]/Ld-T*Vab_Filter[1]/Ld;
	Iab_fore_New[0] = LPF_Filter(Iab_fore_New[0]);
	Iab_fore_New[1] = LPF_Filter(Iab_fore_New[1]);
	if(Start_Flag == 1)//�������֣��и�
	{
	  We = 2*3.1415926*4*Speed/60;
	  Eab[0] = -We*flux*sincos[0];
	  Eab[1] = We*flux*sincos[1];
    }
	else
    {
		Speed = We_fore*7.5/3.1416;
		Eab[0] = -We_fore*flux*sincos[0];
	  Eab[1] = We_fore*flux*sincos[1];
	}
	//printf("%5.2f,%5.2f\n" , Iab_fore_New[0] , Iab[0]);//���۲����
	//printf("%5.2f,%5.2f\n" , Iab_fore_New[1] , Iab[1]);//�¹۲����
	//printf("%5.2f,%5.2f\n" , Eab[0],Vab_Filter[0]);
	PLL_SMO(Vab_Filter);
	
//  float h1 , h2 , signa , signb;//Ԥ�����������棬������ʹ��
//	if(Iab_fore_New[0] >= Iab[0])
//		signa = 1;
//	else
//		signa = -1;
//	
//	if(Iab_fore_New[1] >= Iab[1])
//		signb = 1;
//	else
//		signb = -1;
//	
//	h1 = -R*fabs(Iab_fore_New[0] - Iab[0])+Eab[0]*signa;
//	h2 = -R*fabs(Iab_fore_New[1] - Iab[1])+Eab[1]*signb;
//	printf("%5.2f,%5.2f\n" , h1 , h2);
	
	Iab_fore_Last[0] = Iab_fore_New[0];
	Iab_fore_Last[1] = Iab_fore_New[1];
	Iab_Last[0] = Iab[0];
	Iab_Last[1] = Iab[1];
}

void PLL_SMO(float*Vin)
{
	float x , P_Partern , I_Partern ;
	x = -cos(Theta_fore_Last)*Vin[0]-sin(Theta_fore_Last)*Vin[1];
	P_Partern = PLL_Kp*x;
	I_Partern += PLL_Ki*x;
	We_fore = P_Partern+I_Partern;
	We_fore = Limit(We_fore , 1257);//�޷����ת��Ϊ3000rpm
	We_fore = We_Filter(We_fore);
	Theta_fore_New += We_fore*T;
	if(Theta_fore_New > 6.283)
	Theta_fore_New = 0;

	//printf("%5.2f,%5.2f\n" , Alpha , Theta_fore_New);
	//AlphaΪ������������������ת��ʵ��λ����Ϣ
	//Theta_fore_NewΪ���໷�����ת�ӵ�Ƕ���Ϣ


	//printf("%5.2f,%5.2f\n" , Speed,We_fore*7.5/3.1416);
	//Speed:ʵ��ת��
	//We_fore:Ԥ������ٶȣ�ʹ��ʱת��ΪԤ��ת��

	Theta_fore_Last = Theta_fore_New;
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
float ADC_Low_Pass_Filter(float last_value, uint16_t new_value, float alpha)
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
