#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "usmart.h"  
#include "usart2.h"  
#include "timer.h" 
#include "ov5640.h" 
#include "dcmi.h" 
#include "jcapi.h"	
#include "flow.h"	
#include "dma.h"	


#include "mpu6050.h"
#include "i2c_soft.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "bmp.h"
#include "imu.h"
#include "flash_w25.h" 

#define Frame_size 2
#define W (int)(64*Frame_size)
#define H W
#define FULL_IMAGE_SIZE (H*W)
#define jpeg_buf_size H*W/2  			//定义JPEG数据缓存jpeg_buf的大小(*4字节)
#define RGB_size jpeg_buf_size
u8 flow_q;
float jpg_q=0.1;
u8 en_lcd=0;
u8 en_flow=1;
u8 en_jpg=0;
float t_flow;
//-----------------------------------------------------------------
u8 ovx_mode=0;							//bit0:0,RGB565模式;1,JPEG模式 

u16 offsetx=480/2-H/2,offsety=800/2-W/2;

__align(4) u32 jpeg_buf[jpeg_buf_size];	//JPEG数据缓存buf

uint8_t image_buffer_8bit_1[64*64]={0};
uint8_t image_buffer_8bit_2[64*64]={0};

volatile u32 jpeg_data_len=0; 			//buf中的JPEG有效数据长度 
volatile u8 jpeg_data_ok=0;				//JPEG数据采集完成标志 
										//0,数据没有采集完;
										//1,数据采集完了,但是还没处理;
										//2,数据已经处理完成了,可以开始下一帧接收
//JPEG尺寸支持列表
const u16 jpeg_img_size_tbl[][2]=
{
	160,120,	//QQVGA 
	320,240,	//QVGA  
	640,480,	//VGA
	800,600,	//SVGA
	1024,768,	//XGA
	1280,800,	//WXGA 
	1440,900,	//WXGA+
	1280,1024,	//SXGA
	1600,1200,	//UXGA	
	1920,1080,	//1080P
	2048,1536,	//QXGA
	2592,1944,	//500W 
}; 
const u8*EFFECTS_TBL[7]={"Normal","Cool","Warm","B&W","Yellowish ","Inverse","Greenish"};	//7种特效 
const u8*JPEG_SIZE_TBL[12]={"QQVGA","QVGA","VGA","SVGA","XGA","WXGA","WXGA+","SXGA","UXGA","1080P","QXGA","500W"};//JPEG图片

u8 get_one;
u16 off_pix1=0;
//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
void jpeg_data_process(void)
{u16 x,y,color,i,j,k,l;
  volatile	uint8_t image_buffer_8bit_now[FULL_IMAGE_SIZE];	
	volatile  uint8_t image_buffer_8bit_2_t[64*64]={0};	
	if(!ovx_mode)//只有在RGB
	{
	 if(jpeg_data_ok==0)	//jpeg数据还未采集完?
		{	
			DMA_Cmd(DMA2_Stream1, DISABLE);//停止当前传输 
			while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}//等待DMA2_Stream1可配置  
			jpeg_data_len=RGB_size-DMA_GetCurrDataCounter(DMA2_Stream1);//得到此次数据传输的长度
			
		jpeg_data_ok=1; 				//标记JPEG数据采集完按成,等待其他函数处理
		if(!get_one||0){		
		x=y=j=i=k=l=0;
		for(x=0;x<W;x++)
		for(y=0;y<H;y++){							
		image_buffer_8bit_now[k++]=RGB_GRAY_16(read_xy_rgb( jpeg_buf,  x,  y, W));				  
		}
		
		x=y=j=i=k=l=0;
  	for(x=W/2-32;x<64+W/2-32;x++)
		 for(y=H/2-32;y<64+H/2-32;y++)	
			image_buffer_8bit_2_t[k++]=image_buffer_8bit_now[(x)*W+(y)];
	  
		x=y=j=i=k=l=0;	
    		
	  for(i=0;i<64*64;i++)
			 image_buffer_8bit_2[i]=image_buffer_8bit_2_t[i];
   	}
//	 for(x=0;x<64-off_pix1;x++)
//		 for(y=0;y<64;y++)	
//				{image_buffer_8bit_2[k]=image_buffer_8bit_2_t[LIMIT((x+off_pix1)*64+y,0,64*64-1)];
//					if(k<64*64-2)
//						k++;
//				}
				
//	 for(x=off_pix1;x<64;x++)
//		 for(y=0;y<64;y++)	
//			{image_buffer_8bit_2[k]=image_buffer_8bit_2_t[LIMIT((x-off_pix1)*64+y,0,64*64-1)];	
//    	if(k<64*64-2)
//						k++;
//			}
				
		get_one=1;	
		jpeg_data_ok=2;
		}
		if(jpeg_data_ok==2)	//上一次的jpeg数据已经被处理了
		{
			DMA2_Stream1->NDTR=RGB_size;	
			DMA_SetCurrDataCounter(DMA2_Stream1,RGB_size);//传输长度为jpeg_buf_size*4字节
			DMA_Cmd(DMA2_Stream1, ENABLE);			//重新传输
			jpeg_data_ok=0;						//标记数据未采集
		}
	}
} 
float gh=0.05;
float ga=0.1;
float gwa=0.1;
double X_KF_NAV[2][3];
double P_KF_NAV[2][9];
float ga_nav= 0.1; 
float gwa_nav=0.1;
float g_pos_flow= 0.0086;//0.0051;
float g_spd_flow= 0.0006;
float K_spd_flow=0.87;
u8 en_imu=1;
//RGB565测试
//RGB数据直接显示在LCD上面
u8 fig_view[22]={0};
void rgb565_test(void)
{ volatile	uint8_t image_buffer_8bit_now[FULL_IMAGE_SIZE];
	u16 x,y,color,i,j,k,l;
	u16 cow,row;
	u8 key;
	u8 *p;
	u8 effect=0,contrast=2,fac;
	u8 scale=1;		//默认是全尺寸缩放
	u8 msgbuf[15];	//消息缓存区 
		//自动对焦初始化
	OV5640_RGB565_Mode();	//RGB565模式 
	OV5640_Focus_Init();
	
	OV5640_Light_Mode(0);	//自动模式
	OV5640_Color_Saturation(3);//色彩饱和度0
	OV5640_Brightness(4);	//亮度0
	OV5640_Contrast(3);		//对比度0
	OV5640_Sharpness(33);	//自动锐度
	OV5640_Focus_Constant();//启动持续对焦
	
	My_DCMI_Init();			//DCMI配置
	DCMI_DMA_Init((u32)&jpeg_buf,jpeg_buf_size,DMA_MemoryDataSize_Word,DMA_MemoryInc_Enable);//DCMI DMA配置  
 	//OV5640_ImageWin_Set(4,0,64,64);				//全尺寸缩放
	OV5640_OutSize_Set(0,0,W,H);
	//OV5640_Test_Pattern(2);
	DCMI_Start(); 		//启动传输
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,?????1,????SendBuff,???:SEND_BUF_SIZE.
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);   
while(1)
	{ 
		
		if(get_one==1)	//已经采集完一帧图像了
		{  
	  x=y=j=i=k=l=0;
				for(k=0;k<22;k++)
				 fig_view[k]=image_buffer_8bit_2[k];	
//show origin fig on lcd					
x=y=j=i=k=l=0;	
if(!en_flow&&en_lcd){							
			for(x=0;x<H;x++)
			  for(y=0;y<W;y++)	{	
						  for(cow=0;cow<=lcddev.width/W*1;cow++){
								  for(row=0;row<=lcddev.height/H*1;row++)	
								LCD_Fast_DrawPoint(offsetx+y*lcddev.width/W*1+cow,
								offsety+x*lcddev.height/H*1+row,GRAY_RGB(image_buffer_8bit_now[k]));
							}	
				k++;}
			}
//auto resize			
x=y=j=i=k=l=0;	
	
//resize fig		

				
//gray_balance

//cal bright

//show flow fig on lcd
t_flow = Get_Cycle_T(0)/1000000.0f;								//???????????											
static u8 cnt_show[3];							
x=y=j=i=k=l=0;	
			static u8 cnt_flow;
	    if(en_flow&&cnt_flow++>0)
			{ cnt_flow=0;		
				flow_q=flow_task(image_buffer_8bit_2, image_buffer_8bit_1,t_flow);
			}	
			
			for(i=0;i<64*64;i++)
			 image_buffer_8bit_1[i]=image_buffer_8bit_2[i];
			
static float acc_flt[3]={0};
		if(en_imu){
		IMUupdate(0.5f *t_flow,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z	,&Rol_fc,&Pit_fc,&Yaw_fc);	
		bmp_rx=1;
		MS5611_ThreadNew();		
		bmp_rx=0;
		//		static float wz_acc ;	 
		//			
		float acc_temp1=(float)(reference_vr_imd_down_fc[2] *mpu6050_fc.Acc.z + reference_vr_imd_down_fc[0] *mpu6050_fc.Acc.x + reference_vr_imd_down_fc[1] *mpu6050_fc.Acc.y - 4096  )/4096.0f*9.8;

		acc_flt[2] = ( 1 / ( 1 + 1 / ( 20 *3.14f *t_flow ) ) )*( (acc_temp1 - acc_flt[2] ));	
		double Z_kf[3]={MS5611_Altitude,0,0};
		kf_oldx( X_kf_baro,  P_kf_baro,  Z_kf,  acc_flt[2] , gh,  0.1,  0.1,t_flow);

		float a_br[3]={0};	
		float acc_temp[3]={0};

		a_br[0] =(float) mpu6050_fc.Acc.x/4096.*9.8;//16438.;
		a_br[1] =(float) mpu6050_fc.Acc.y/4096.*9.8;//16438.;
		a_br[2] =(float) mpu6050_fc.Acc.z/4096.*9.8;//16438.;
		// acc
		acc_temp[0] = a_br[1]*reference_vr_imd_down_fc[2]  - a_br[2]*reference_vr_imd_down_fc[1] ;
		acc_temp[1] = a_br[2]*reference_vr_imd_down_fc[0]  - a_br[0]*reference_vr_imd_down_fc[2] ;
		acc_flt[0] += ( 1 / ( 1 + 1 / ( 15 *3.14f *t_flow ) ) ) *( acc_temp[0] - acc_flt[0] );
		acc_flt[1] += ( 1 / ( 1 + 1 / ( 15 *3.14f *t_flow ) ) ) *( acc_temp[1] - acc_flt[1] );
		float T=t_flow;
		double A[9]={1,       0,    0,T,       1,    0,-T*T/2, -T,    1};
		double B[3]={T*T/2,T,0}; 
		double H1[9]={0,0,0,0,1,0,0,0,0}; 	
		float flowx,flowy;
		float Sdpx=flowx*K_spd_flow;
		float Accx=acc_flt[0];
		double Zx[3]={0,Sdpx,0};
		KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H1,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
		float Sdpy=flowy*K_spd_flow;
		float Accy=acc_flt[1];
		double Zy[3]={0,Sdpy,0};
		KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H1,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);	
		}else bmp_rx=0;
//upload flow fig in jpg	
 static u16 cnt_up_fig;			
			
		  if(en_jpg){
				DCMI_Stop(); 		//启动传输
				for(i=0;i<64*64;i++){//IWDG_Feed();			
				RGB565TORGB24(i,GRAY_RGB(image_buffer_8bit_1[i]));
				}
 		    Compression(64,64,jpg_q);//压缩 
				DCMI_Start(); 		//启动传输
				}
//		   	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//??DMA2_Steam7????
//							{ 	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//??DMA2_Steam7??????
//								 SendBuff1_cnt=0;
//									data_per_uart1(0,0,0,
//															 0,pixel_flow_x*100,0,
//															 0,pixel_flow_y*100,0,
//							0,0,0,0,0,0,0);
//							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //????1?DMA??     
//							MYDMA_Enable(DMA2_Stream7,SendBuff1_cnt+2);     //????DMA??!	  
//							}	
				
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//??DMA2_Steam7????
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//??DMA2_Steam7??????		
									SendBuff2_cnt=0;	
								
									if(en_jpg){cnt_up_fig=0;
								p=(u8*)JPG_enc_buf;
								for(i=0;i<JUGG_BUF;i++)		//dma传输1次等于4字节,所以乘以4.
								SendBuff2[SendBuff2_cnt++]=p[i];
						  	}else{
							   data_per_uart1(acc_flt[0]*100,acc_flt[1]*100,acc_flt[2]*100,
															 0,pixel_flow_x*100,0,
															 0,pixel_flow_y*100,0,
						    	Yaw_fc*10,Pit_fc*10,Rol_fc*10,0,0,0,0);			
								}									
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //????1?DMA??     
							MYDMA_Enable(DMA1_Stream6,SendBuff2_cnt+2);     //????DMA??!	
								}
//				else
//				GOL_LINK(0,0,0,
//															 0,pixel_flow_x*1000,0,
//															 0,pixel_flow_y*1000,0);		
				static u16 cnt_focus;
				if(cnt_focus++/(t_flow*1000)>16&&0){cnt_focus=0;
				OV5640_Focus_Single();}
			get_one=0;		
	}
	}       
} 
	
int main(void)
{ 
	u8 key;
	u8 t;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	uart_init2(115200);		//初始化串口波特率为115200
//	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,?????1,????SendBuff,???:SEND_BUF_SIZE.
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);     
//	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);  
  I2c_Soft_Init();					//?????I2C
	MPU6050_Init(20);   			//???????????,??20hz??
	HMC5883L_SetUp();
	MS5611_Init_FC();
	W25QXX_Init();			//W25QXX初始化
	while(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)								//检测不到W25Q128
	{	
		delay_ms(200);
		LED1=!LED1;
	}
  READ_PARM(); 
	LED_Init();					//初始化LED 
	if(en_lcd)
 	LCD_Init();					//LCD初始化  
 	KEY_Init();					//按键初始化 
	TIM3_Int_Init(100-1,8400-1);//10Khz计数,1秒钟中断一次
	Initial_Timer5();
	Cycle_Time_Init();
	while(OV5640_Init())//初始化OV2640
	{
		delay_ms(200);
		LED0=!LED0;
	}	

	  rgb565_test(); 	
		
}
