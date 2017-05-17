#include "timer.h"
#include "led.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

extern u8 ov_frame;
extern volatile u16 jpeg_data_len;


//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
#include "mpu6050.h"
#include "delay.h"
#include "imu.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "bmp.h"
//定时器3中断服务函数
u8 ov_frame_rx;

double P_kf_baro[9]={1,0,0,1,0,0,1,0,0}; 
double X_kf_baro[3];
float t_mems;
u8 bmp_rx;
void TIM3_IRQHandler(void)
{static u8 cnt_1s,cnt_20ms;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
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
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
