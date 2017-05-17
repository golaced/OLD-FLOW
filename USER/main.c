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

#define Frame_size 2
float  SCALE=1;
#define W 64*Frame_size
#define H W//120
#define FULL_IMAGE_SIZE (H*W)
#define jpeg_buf_size H*W/2  			//����JPEG���ݻ���jpeg_buf�Ĵ�С(*4�ֽ�)
#define RGB_size H*W/2 
float scale_x=0.2,scale_y=0.2;
u8 flow_q;
float jpg_q=0.66;
u8 en_lcd=0;
u8 en_flow=1;
u8 en_jpg=0;
float t_flow;
//-----------------------------------------------------------------
u8 ovx_mode=0;							//bit0:0,RGB565ģʽ;1,JPEGģʽ 

u16 offsetx=480/2-H/2,offsety=800/2-W/2;

__align(4) u32 jpeg_buf[jpeg_buf_size];	//JPEG���ݻ���buf

uint8_t image_buffer_8bit_1[64*64]={0};
uint8_t image_buffer_8bit_2[64*64]={0};

volatile u32 jpeg_data_len=0; 			//buf�е�JPEG��Ч���ݳ��� 
volatile u8 jpeg_data_ok=0;				//JPEG���ݲɼ���ɱ�־ 
										//0,����û�вɼ���;
										//1,���ݲɼ�����,���ǻ�û����;
										//2,�����Ѿ����������,���Կ�ʼ��һ֡����
//JPEG�ߴ�֧���б�
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
const u8*EFFECTS_TBL[7]={"Normal","Cool","Warm","B&W","Yellowish ","Inverse","Greenish"};	//7����Ч 
const u8*JPEG_SIZE_TBL[12]={"QQVGA","QVGA","VGA","SVGA","XGA","WXGA","WXGA+","SXGA","UXGA","1080P","QXGA","500W"};//JPEGͼƬ

u8 get_one;
//����JPEG����
//���ɼ���һ֡JPEG���ݺ�,���ô˺���,�л�JPEG BUF.��ʼ��һ֡�ɼ�.
void jpeg_data_process(void)
{u16 x,y,color,i,j,k,l;
  volatile	uint8_t image_buffer_8bit_now[FULL_IMAGE_SIZE];	
	if(!ovx_mode)//ֻ����RGB
	{
	 if(jpeg_data_ok==0)	//jpeg���ݻ�δ�ɼ���?
		{	
			DMA_Cmd(DMA2_Stream1, DISABLE);//ֹͣ��ǰ���� 
			while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}//�ȴ�DMA2_Stream1������  
			jpeg_data_len=RGB_size-DMA_GetCurrDataCounter(DMA2_Stream1);//�õ��˴����ݴ���ĳ���
			
		jpeg_data_ok=1; 				//���JPEG���ݲɼ��갴��,�ȴ�������������
		x=y=j=i=k=l=0;
		for(x=0;x<W;x++)
		for(y=0;y<H;y++){							
		image_buffer_8bit_now[k++]=RGB_GRAY_16(read_xy_rgb( jpeg_buf,  x,  y, W));				  
		}
		
		x=y=j=i=k=l=0;
  	for(x=H/2-32;x<64+H/2-32;x++)
		 for(y=W/2-32;y<64+W/2-32;y++)	
			image_buffer_8bit_2[k++]=image_buffer_8bit_now[(x)*H+(y)];
							
		
		
//		 if(en_jpg){
//				for(i=0;i<64*64;i++){//IWDG_Feed();			
//				RGB565TORGB24(i,GRAY_RGB(image_buffer_8bit_2[i]));
//				}
// 		   Compression(64,64,jpg_q);//ѹ�� 
//				u8 *p=(u8*)JPG_enc_buf;
//					for(i=0;i<JUGG_BUF;i++)		//dma����1�ε���4�ֽ�,���Գ���4.
//						UsartSend_GOL_LINK(p[i]); 	
//				}
		 
		get_one=1;	
		jpeg_data_ok=2;
		}
		if(jpeg_data_ok==2)	//��һ�ε�jpeg�����Ѿ���������
		{
			DMA2_Stream1->NDTR=RGB_size;	
			DMA_SetCurrDataCounter(DMA2_Stream1,RGB_size);//���䳤��Ϊjpeg_buf_size*4�ֽ�
			DMA_Cmd(DMA2_Stream1, ENABLE);			//���´���
			jpeg_data_ok=0;						//�������δ�ɼ�
		}
	}
} 
//JPEG����
//JPEG����,ͨ������2���͸�����.
void jpeg_test(void)
{
	u32 i,jpgstart,jpglen; 
	u8 *p;
	u8 key,headok=0;
	u8 effect=0,contrast=2;
	u8 size=2;			//Ĭ����QVGA 640*480�ߴ�
	u8 msgbuf[15];		//��Ϣ������ 
	
 	//�Զ��Խ���ʼ��
	OV5640_RGB565_Mode();	//RGB565ģʽ 
	OV5640_Focus_Init(); 
	
 	OV5640_JPEG_Mode();		//JPEGģʽ
	
	OV5640_Light_Mode(0);	//�Զ�ģʽ
	OV5640_Color_Saturation(3);//ɫ�ʱ��Ͷ�0
	OV5640_Brightness(4);	//����0
	OV5640_Contrast(3);		//�Աȶ�0
	OV5640_Sharpness(33);	//�Զ����
	OV5640_Focus_Constant();//���������Խ�
	
	My_DCMI_Init();			//DCMI����
	DCMI_DMA_Init((u32)&jpeg_buf,jpeg_buf_size,DMA_MemoryDataSize_Word,DMA_MemoryInc_Enable);//DCMI DMA����   
	OV5640_OutSize_Set(4,0,jpeg_img_size_tbl[size][0],jpeg_img_size_tbl[size][1]);//��������ߴ� 
	DCMI_Start(); 		//��������
	while(1)
	{
				
	}     
} 

//RGB565����
//RGB����ֱ����ʾ��LCD����
u8 fig_view[22]={0};
void rgb565_test(void)
{ volatile	uint8_t image_buffer_8bit_now[FULL_IMAGE_SIZE];
	u16 x,y,color,i,j,k,l;
	u16 cow,row;
	u8 key;
	u8 *p;
	u8 effect=0,contrast=2,fac;
	u8 scale=1;		//Ĭ����ȫ�ߴ�����
	u8 msgbuf[15];	//��Ϣ������ 
		//�Զ��Խ���ʼ��
	OV5640_RGB565_Mode();	//RGB565ģʽ 
	OV5640_Focus_Init();
	
	OV5640_Light_Mode(0);	//�Զ�ģʽ
	OV5640_Color_Saturation(3);//ɫ�ʱ��Ͷ�0
	OV5640_Brightness(4);	//����0
	OV5640_Contrast(3);		//�Աȶ�0
	OV5640_Sharpness(33);	//�Զ����
	OV5640_Focus_Constant();//���������Խ�
	
	My_DCMI_Init();			//DCMI����
	DCMI_DMA_Init((u32)&jpeg_buf,jpeg_buf_size,DMA_MemoryDataSize_Word,DMA_MemoryInc_Enable);//DCMI DMA����  
 	//OV5640_ImageWin_Set(4,0,64,64);				//ȫ�ߴ�����
	OV5640_OutSize_Set(4,0,W*SCALE,H*SCALE);
	
	DCMI_Start(); 		//��������
while(1)
	{ 
		
		if(get_one==1)	//�Ѿ��ɼ���һ֡ͼ����
		{  
	  x=y=j=i=k=l=0;
				for(k=0;k<22;k++)
				 fig_view[k]=image_buffer_8bit_2[k+W*H/2-22];	
//show origin fig on lcd					
x=y=j=i=k=l=0;	
if(!en_flow&&en_lcd){							
			for(x=0;x<H;x++)
			  for(y=0;y<W;y++)	{	
						  for(cow=0;cow<=lcddev.width/W*scale_x;cow++){
								  for(row=0;row<=lcddev.height/H*scale_y;row++)	
								LCD_Fast_DrawPoint(offsetx+y*lcddev.width/W*scale_x+cow,
								offsety+x*lcddev.height/H*scale_y+row,GRAY_RGB(image_buffer_8bit_now[k]));
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
	    if(en_flow)
			{ 		
				flow_q=flow_task(image_buffer_8bit_2, image_buffer_8bit_1,0);
			}	
			
			memcpy( image_buffer_8bit_1,image_buffer_8bit_2,64*64);			
			
//upload flow fig in jpg				
		  if(en_jpg){
				DCMI_Stop(); 		//��������
				for(i=0;i<64*64;i++){//IWDG_Feed();			
				RGB565TORGB24(i,GRAY_RGB(image_buffer_8bit_2[i]));
				}
 		   Compression(64,64,jpg_q);//ѹ�� 
//				p=(u8*)JPG_enc_buf;
//					for(i=0;i<JUGG_BUF;i++)		//dma����1�ε���4�ֽ�,���Գ���4.
//						UsartSend_GOL_LINK(p[i]); 	
				DCMI_Start(); 		//��������
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
														if(!en_jpg)
							   data_per_uart1(0,0,0,
															 0,pixel_flow_x*100,0,
															 0,pixel_flow_y*100,0,
							0,0,0,0,0,0,0);
														else{
								p=(u8*)JPG_enc_buf;
								for(i=0;i<JUGG_BUF;i++)		//dma����1�ε���4�ֽ�,���Գ���4.
								SendBuff2[SendBuff2_cnt++]=p[i];}
														
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	uart_init2(115200);		//��ʼ�����ڲ�����Ϊ115200
//	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,?????1,????SendBuff,???:SEND_BUF_SIZE.
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);     
//	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);  
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,?????1,????SendBuff,???:SEND_BUF_SIZE.
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     	
   
	LED_Init();					//��ʼ��LED 
	if(en_lcd)
 	LCD_Init();					//LCD��ʼ��  
 	KEY_Init();					//������ʼ�� 
	TIM3_Int_Init(10000-1,8400-1);//10Khz����,1�����ж�һ��
	Initial_Timer5();
	Cycle_Time_Init();
	while(OV5640_Init())//��ʼ��OV2640
	{
		delay_ms(200);
		LED0=!LED0;
	}	
	  rgb565_test(); 	
		
}
