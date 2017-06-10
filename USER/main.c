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
#include "imu.h"
#include "sys.h"
#include "mymath.h"
#include "filter.h"
#include "iwdg.h"
#include "imu.h"
#include "flash_w25.h" 

u8 MAX_BRIGHT=250;
u16 max_num=3;
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

int offsetx=10*Frame_size,offsety=10*Frame_size;

static __align(4) u32 jpeg_buf[jpeg_buf_size];	//JPEG数据缓存buf


uint8_t image_buffer_8bit_2[64*64]={0};
uint8_t image_buffer_8bit_1[64*64]={0};
uint8_t image_buffer_8bit_2_t[64*64]={0};
uint8_t image_buffer_8bit_1_t[64*64]={0};
volatile u32 jpeg_data_len=0; 			//buf中的JPEG有效数据长度 
volatile u8 jpeg_data_ok=0;				//JPEG数据采集完成标志 
										
u8 get_one;
u16 off_pix1=0;
//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.

float fHistogram[256],fHistogram1[256];
unsigned char lut[256];

 void Histogram(unsigned char *pImage,int nWidth,int nHeight,float fHisto[256])
{
   int i,j;
   unsigned int uWork;
   unsigned char *pWork;
   
    for ( i=0;i<256;i++ )    fHisto[i]=0.0f;
    pWork=pImage;
    for ( i=0;i<nHeight;i++ )
    {  
        for ( j=0;j<nWidth;j++,pWork++ )
        {
            uWork=(unsigned int)(*pWork);
            fHisto[uWork]++;
        }
    }
    uWork=nWidth*nHeight;
    for ( i=0;i<256;i++ )
    {
        fHisto[i]/=uWork;
        fHisto[i]*=100;
    }
}

void Enhance(unsigned char *pImage,unsigned char *pImage1,int nWidth,int nHeight,float fHisto[256],float fHisto1[256])
{
   int i,j;
   unsigned int uWork;
   unsigned char *pWork,*pWork1;
   
    for ( i=0;i<256;i++ )
        fHisto1[i]=fHisto[i]/100;
    for ( i=1;i<256;i++ )
        fHisto1[i]+=fHisto1[i-1];
    for ( i=0;i<256;i++ )
        lut[i]=fHisto1[i]*256;
    for ( i=0;i<256;i++ )
        if ( lut[i]>=255 )
            lut[i]=255;
    pWork=pImage; pWork1=pImage1;
    for ( i=0;i<nHeight;i++ )
        for ( j=0;j<nWidth;j++,pWork++,pWork1++ )
            (*pWork1)=lut[(*pWork)]>>max_num;
}


void gray_balance(unsigned char *pImage,unsigned char *pImage1){
    Histogram(pImage,64,64,fHistogram); //做直方图统计
    Enhance(pImage,pImage1,64,64,fHistogram,fHistogram1); //
}

u8 en_guass=0;
void gaussianFilter(unsigned char *I_image,unsigned char *O_image, int width, int height,float eps)
{
   int i, j, index, sum;
    int templates[9] = { 1, 2, 1,
                         2, 4, 2,
                         1, 2, 1 };
    sum = height * width;
    for(i = 1;i < height - 1;i++)
    {
        for(j = 1;j < width - 1;j++)
        {
            index = sum = 0;
            for(int m = i - 1;m < i + 2;m++)
            {
                for(int n = j - 1; n < j + 2;n++)
                {
                    sum +=
                        I_image[m * width + n] *
                        templates[index++];
                }
            }
            O_image[i * width + j] = sum / 16;
        }
    }
}

void HistNormolize(unsigned char *pImg, unsigned char *pNormImg,int width,int height)
{  int hist[256];
 float  fpHist[256];
 float eqHistTemp[256];
 int eqHist[256];
 int size = height *width;
 int i ,j;
 memset(&hist,0x00,sizeof(int)*256);
 memset(&fpHist,0x00,sizeof(float)*256);
 memset(&eqHistTemp,0x00,sizeof(float)*256);
 for (i = 0;i < height; i++) //?????????
 {
  for (j = 0; j < width; j++)
  {
   unsigned char GrayIndex = pImg[i*height+j];
   hist[GrayIndex] ++ ;
  }
 }
 for (i = 0; i< 256; i++)   // ????????
 {
  fpHist[i] = (float)hist[i] / (float)size;
 }
 for ( i = 1; i< 256; i++)   // ?????????
 {
  if (i == 0)
  {
   eqHistTemp[i] = fpHist[i];
  }
  else
  {
   eqHistTemp[i] = eqHistTemp[i-1] + fpHist[i];
  }
 }
 //??????,?????????????
 for (i = 0; i< 256; i++)
 {
  eqHist[i] = (int)(255.0 * eqHistTemp[i] + 0.5);
 }
 for (i = 0;i < height; i++) //?????? ???
 {
  for (j = 0; j < width; j++)
  {
   unsigned char GrayIndex = pImg[i*height+j];
   pNormImg[i*height+j] = eqHist[GrayIndex]>>max_num;
  }
 }
}

void jpeg_data_process(void)
{u16 x,y,color,i,j,k,l;
uint8_t image_buffer_8bit_2_t[64*64]={0};	
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
  	for(x=W/2-32-offsetx;x<64+W/2-32-offsetx;x++)
		 for(y=H/2-32-offsety;y<64+H/2-32-offsety;y++)	
			image_buffer_8bit_2_t[k++]=RGB_GRAY_16(read_xy_rgb( jpeg_buf,  x,  y, W));	  
			
		x=y=j=i=k=l=0;	  			
		if(MAX_BRIGHT!=0)	
		//gray_balance(image_buffer_8bit_2_t,image_buffer_8bit_2);	
		HistNormolize(image_buffer_8bit_2_t,image_buffer_8bit_2,64,64);	
		else	
	  for(i=0;i<64*64;i++)
			 image_buffer_8bit_2[i]=image_buffer_8bit_2_t[i];	
  	}
		if( en_guass){
    gaussianFilter(image_buffer_8bit_2,image_buffer_8bit_2_t, 64,64,0);
		for(i=0;i<64*64;i++)
		 image_buffer_8bit_2[i]=image_buffer_8bit_2_t[i];
		}			
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
double X_KF_NAV[2][3];
double P_KF_NAV[2][9];
float ga_nav= 0.1; 
float gwa_nav=0.1;
float g_pos_flow= 0.0086;//0.0051;
float g_spd_flow= 0.000888;

u8 en_camera=1;
u8 en_imu=0;
float ALT_POS_SONAR=0.7;
float k_px=50;
FLOW_RAD flow;
#if USE_FPS60
float K_spd_flow=0.44;
#else
float K_spd_flow=1;
#endif
u8 Shape=33;
u8 Contract=6;
u8 Effect=0;
void rgb565_test(void)
{ 
 
	u16 x,y,color,i,j,k,l;
	u8 *p;

		//自动对焦初始化
	OV5640_RGB565_Mode();	//RGB565模式 
	OV5640_Focus_Init();	
	OV5640_Light_Mode(0);	//自动模式
	OV5640_Color_Saturation(6);//色彩饱和度0
	OV5640_Brightness(8);	//亮度0
	OV5640_Exposure(6);
//0:正常    
//1,冷色
//2,暖色   
//3,黑白
//4,偏黄
//5,反色
//6,偏绿	    
  OV5640_Special_Effects(Effect);
	OV5640_Contrast(Contract);		//对比度0
	OV5640_Sharpness(Shape);	//自动锐度
	//OV5640_Focus_Constant();//启动持续对焦
	My_DCMI_Init();			//DCMI配置
	DCMI_DMA_Init((u32)&jpeg_buf,jpeg_buf_size,DMA_MemoryDataSize_Word,DMA_MemoryInc_Enable);//DCMI DMA配置  
 	//OV5640_ImageWin_Set(4,0,64,64);				//全尺寸缩放
	OV5640_OutSize_Set(0,0,W,H);
	//OV5640_Test_Pattern(2);
	if(en_camera)
	DCMI_Start(); 		//启动传输
	TIM3_Int_Init(10000-1,8400-1);//10Khz计数,1秒钟中断一次
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,?????1,????SendBuff,???:SEND_BUF_SIZE.
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	//en_guass=0;  
	//en_hist_filter=1; 
while(1)
	{ static float t_mpu,t_focus;
		float dt= Get_Cycle_T(4)/1000000.0f;								//???????????	
    t_mpu+=dt;
    t_focus+=dt;
		
    if(t_focus>45&&1){t_focus=0;
		OV5640_Focus_Single	();
		}		
		if(t_mpu>0.015){
//		MPU6050_Read();
//		MPU6050_Data_Prepare( t_mpu );			
   
	  //Send_FLOW();
	 if(fc_connect_loss++>33)fc_connect=0;	
		t_mpu=0;} 														
    
		

	//??mpu6????	
		if(get_one==1)	//已经采集完一帧图像了
		{  
		t_flow = Get_Cycle_T(0)/1000000.0f;								//???????????		

		IWDG_Feed();												
		static u8 cnt_flow;
		if(en_flow)
		{ 
		k=0;	
		for(i=0;i<64;i++)
			for(j=0;j<64;j++)
		image_buffer_8bit_2_t[k++]=image_buffer_8bit_2[i+j*64];	
			
		flow_q=flow_task(image_buffer_8bit_2, image_buffer_8bit_1,image_buffer_8bit_2_t, image_buffer_8bit_1_t,t_flow);
		for(i=0;i<64*64;i++)
		image_buffer_8bit_1[i]=image_buffer_8bit_2[i];	
		for(i=0;i<64*64;i++)
		image_buffer_8bit_1_t[i]=image_buffer_8bit_2_t[i];	
		}	
		  
		static uint32_t lasttime = 0;	
    float x_rate = 0;//-mpu6050_fc.Gyro_deg.x; // change x and y rates
		float y_rate = 0;//-mpu6050_fc.Gyro_deg.y;
		float z_rate = 0;//mpu6050_fc.Gyro_deg.z; // z is correct
    float focal_length_px = (3) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled
    uint32_t deltatime = (micros() - lasttime);
		flow.integration_time_us=deltatime ;
		flow.integrated_x=pixel_flow_x  / focal_length_px * 1.0f*k_px; 
		flow.integrated_y=pixel_flow_y  / focal_length_px * -1.0f*k_px;
		flow.integrated_xgyro=x_rate *flow.integration_time_us/1000000.;
		flow.integrated_ygyro=y_rate *flow.integration_time_us/1000000.;
		flow.integrated_zgyro=z_rate *flow.integration_time_us/1000000.;

    lasttime = micros();

		bmp_rx=0;
//upload flow fig in jpg	
 static u16 cnt_up_fig;			
			
		  if(en_jpg){
				#if EN_JPG	
				DCMI_Stop(); 		//启动传输
				for(i=0;i<64*64;i++){//IWDG_Feed();			
			
				RGB565TORGB24(i,GRAY_RGB(image_buffer_8bit_1[i]));
				
				}
 		    Compression(64,64,jpg_q);//压缩 
				DCMI_Start(); 		//启动传输
				#endif
				}
				  static u8 uart_init;
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET||uart_init==0)
								{ uart_init=1;
								DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
								SendBuff2_cnt=0;	
                     
									if(en_jpg){cnt_up_fig=0;	
								#if EN_JPG	
								p=(u8*)JPG_enc_buf;
								for(i=0;i<JUGG_BUF;i++)SendBuff2[SendBuff2_cnt++]=p[i];		//dma传输1次等于4字节,所以乘以4.
								#endif
						  	}else{
							  data_per_uart1(pixel_flow_x_sad*100,pixel_flow_x_sadt*100,pixel_flow_x_klt*100,													 
								pixel_flow_y_sad*100,pixel_flow_y_sadt*100,pixel_flow_y_klt*100,				//flow.integrated_x*100,1*flow.integrated_xgyro*100,0*flow.h_x_pix*100,															
								pixel_flow_x*100,pixel_flow_y*100,0*flow.h_y_pix*0,
						    	0*10,0*10,0*10,0,0,0,0);			
								}									
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //????1?DMA??     
							MYDMA_Enable(DMA1_Stream6,SendBuff2_cnt+2);     //????DMA??!	
								}


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
	Initial_Timer5();
	Cycle_Time_Init();
	uart_init(576000);		//初始化串口波特率为115200
	uart_init2(115200);		//初始化串口波特率为115200
	LED_Init();					//初始化LED 
 	KEY_Init();					//按键初始化 
	delay_ms(10);
//	W25QXX_Init();			//W25QXX初始化
//	delay_ms(10);
//	while(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)	
//	{	
//		delay_ms(200);
//		LED1=!LED1;
//	}
//  READ_PARM(); 

	while(OV5640_Init())//初始化OV2640
	{
		delay_ms(200);
		LED0=!LED0;
	}	
    IWDG_Init(4,500);
	  rgb565_test(); 		
}
