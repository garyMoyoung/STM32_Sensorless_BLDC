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
