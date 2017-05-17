#include "timer.h"
#include "led.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

extern u8 ov_frame;
extern volatile u16 jpeg_data_len;


//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
#include "mpu6050.h"
#include "delay.h"
#include "imu.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "bmp.h"
//��ʱ��3�жϷ�����
u8 ov_frame_rx;

double P_kf_baro[9]={1,0,0,1,0,0,1,0,0}; 
double X_kf_baro[3];
float t_mems;
u8 bmp_rx;
void TIM3_IRQHandler(void)
{static u8 cnt_1s,cnt_20ms;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{ if(cnt_1s++>100){cnt_1s=0;
		ov_frame_rx=ov_frame;
		ov_frame=0;
	  }

		float t_mems = Get_Cycle_T(2)/1000000.0f;								//???????????				
		if(!bmp_rx){
		MPU6050_Read(); 															//??mpu6????
		MPU6050_Data_Prepare( t_mems );			//mpu6????????	
		}
//		IMUupdate(0.5f *t_mems,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z	,&Rol_fc,&Pit_fc,&Yaw_fc);	
 //		MS5611_ThreadNew();		
//		static float wz_acc ;	 
//			
//		float acc_temp1=(float)(reference_vr_imd_down_fc[2] *mpu6050_fc.Acc.z + reference_vr_imd_down_fc[0] *mpu6050_fc.Acc.x + reference_vr_imd_down_fc[1] *mpu6050_fc.Acc.y - 4096  )/4096.0f*9.8;
//		wz_acc+= ( 1 / ( 1 + 1 / ( 20 *3.14f *t_mems ) ) )*( (acc_temp1 - wz_acc));	
//		double Z_kf[3]={MS5611_Altitude,0,0};
//	  kf_oldx( X_kf_baro,  P_kf_baro,  Z_kf,  wz_acc, gh,  ga,  gwa,t_mems);
	
	
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}
