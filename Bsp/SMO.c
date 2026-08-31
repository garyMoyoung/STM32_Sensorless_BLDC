#include "main.h"
#include <math.h>
#include "SMO.h"

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

float LPF_Filter(float data, uint8_t ch)
{
	/* 原来只有一个static Last_y,alpha轴和beta轴两次调用共用同一份滤波器状态,
	   等于把两个独立信号的历史值混在一起滤波。改成按ch(0=alpha,1=beta)分开存状态。 */
	static float Last_y[2];
	static float alpha = 0.3583;
	if(ch > 1) ch = 1;
	data=alpha*data+(1-alpha)*Last_y[ch];
	Last_y[ch] = data;
	return data;
}

float Vab_LPF_Filter(float data, uint8_t ch)
{
	static float Last_y[2];
	if(ch > 1) ch = 1;
	data=Vab_alpha*data+(1-Vab_alpha)*Last_y[ch];
	Last_y[ch] = data;
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

	Vab_Filter[0] = Vab_LPF_Filter(Vab[0], 0);
	Vab_Filter[1] = Vab_LPF_Filter(Vab[1], 1);

	Iab_fore_New[0] = (1-R*T/Ld)*Iab_fore_Last[0]+T*Uab[0]/Ld-T*Vab_Filter[0]/Ld;
	Iab_fore_New[1] = (1-R*T/Ld)*Iab_fore_Last[1]+T*Uab[1]/Ld-T*Vab_Filter[1]/Ld;
	Iab_fore_New[0] = LPF_Filter(Iab_fore_New[0], 0);
	Iab_fore_New[1] = LPF_Filter(Iab_fore_New[1], 1);
	if(Start_Flag == 1)//启动阶段：用外部给定的真实转速
	{
	  We = 2*3.1415926f*MOTOR_POLE_PAIRS*Speed/60;
	  Eab[0] = -We*flux*sincos[0];
	  Eab[1] = We*flux*sincos[1];
    }
	else
    {
		Speed = We_fore*60.0f/(2*3.1415926f*MOTOR_POLE_PAIRS);
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
	float x , P_Partern;
	/* 积分项必须跨调用persist,之前用局部自动变量,每次调用都是栈上垃圾值,
	   += 读到的是未定义的初值,积分永远不会真正累加,锁相环失效。 */
	static float I_Partern = 0.0f;
	x = -cos(Theta_fore_Last)*Vin[0]-sin(Theta_fore_Last)*Vin[1];
	P_Partern = PLL_Kp*x;
	I_Partern += PLL_Ki*x;
	We_fore = P_Partern+I_Partern;
	We_fore = Limit(We_fore , 1257);//�޷����ת��Ϊ3000rpm
	We_fore = We_Filter(We_fore);
	Theta_fore_New += We_fore*T;
	/* 原来只处理正向溢出且直接清0(相位跳变),We_fore为负(反转/启动瞬态)时
	   Theta_fore_New会一直往负方向跑,不会被归一化。改成双向按2π折叠，
	   并且是减/加一圈而不是直接清0,不会产生额外的相位跳变。 */
	if(Theta_fore_New >= 6.28318530718f)
	Theta_fore_New -= 6.28318530718f;
	else if(Theta_fore_New < 0.0f)
	Theta_fore_New += 6.28318530718f;

	//printf("%5.2f,%5.2f\n" , Alpha , Theta_fore_New);
	//AlphaΪ������������������ת��ʵ��λ����Ϣ
	//Theta_fore_NewΪ���໷�����ת�ӵ�Ƕ���Ϣ


	//printf("%5.2f,%5.2f\n" , Speed,We_fore*7.5/3.1416);
	//Speed:ʵ��ת��
	//We_fore:Ԥ������ٶȣ�ʹ��ʱת��ΪԤ��ת��

	Theta_fore_Last = Theta_fore_New;
}

void SMO_ShadowUpdate(float i_alpha, float i_beta, float u_alpha, float u_beta, float elec_angle)
{
	Iab[0] = i_alpha;
	Iab[1] = i_beta;
	Uab[0] = u_alpha;
	Uab[1] = u_beta;
	sincos[0] = (double)cosf(elec_angle);
	sincos[1] = (double)sinf(elec_angle);
	SMO();
}
