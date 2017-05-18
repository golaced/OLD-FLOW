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
#include "mpu6050.h"
#include "ak8975.h"
#include "mymath.h"
#include "filter.h"
#include "ultrasonic.h"

u16 ultra_distance_old;

_height_st ultra;

u8 measure_num=8;
u16 Distance[50]  = {0};
float sonar_filter_oldx(float in) 
{
    u8 IS_success =1;
    u16 Distance1 = 0;
    u16 MAX_error1 = 0;
    u8 MAX_error_targe = 0;
    u8 count = 0;
    u8 i =0;
    u8 j =0;
    u8 num =0; 
    Distance[measure_num-1]=in;
    for(i=0;i<measure_num-1;i++)
		{
		 Distance[i]=Distance[i+1]; 
		}
        for(i = 0 ; i < measure_num-1 ; i++)
        {

            for(j = 0 ; j < measure_num-1-i; j++)       
            {
                if(Distance[j] > Distance[j+1] )
                {
                    Distance1 = Distance[j];
                    Distance[j] =  Distance[j+1];
                    Distance[j+1] = Distance1; 
                }
            }

        }
        MAX_error1 = Distance[1] - Distance[0];
        for(i = 1 ; i < measure_num-1 ; i++)
        {
            if(MAX_error1 < Distance[i+1] - Distance[i] )//如：1 2 3 4 5    8 9 10    MAX_error_targe=4;
            {
                MAX_error1 =  Distance[i+1] - Distance[i];//最大差距
                MAX_error_targe = i;  //记下最大差距值的位置（这组数中的位置）
            }
        }
        float UltrasonicWave_Distance1=0;
        if(MAX_error_targe+1 > (measure_num+1)/2) //前部分有效  1 2 3 4 5    8 9 10  (如果位于中间，后半部优先)
        {
            for(i = 0 ; i <= MAX_error_targe ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }
            UltrasonicWave_Distance1 /= (MAX_error_targe+1);//取平均
        }
        else  //后部分有效  1 2 3   7 8 9 10
        {
             for(i = MAX_error_targe + 1 ; i < measure_num ; i++)
            {
                UltrasonicWave_Distance1 += Distance[i];
            }
            UltrasonicWave_Distance1 /= (measure_num - MAX_error_targe -1);//取平均
        }
    return  (float)UltrasonicWave_Distance1/1000.; //转化为米为单位的浮点数
}


static float sonar_values[3] = { 0.0f };
static unsigned insert_index = 0;

static void sonar_bubble_sort(float sonar_values[], unsigned n);

void sonar_bubble_sort(float sonar_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (sonar_values[j] > sonar_values[j+1]) {
				/* swap two values */
				t = sonar_values[j];
				sonar_values[j] = sonar_values[j + 1];
				sonar_values[j + 1] = t;
			}
		}
	}
}

float insert_sonar_value_and_get_mode_value(float insert)
{
	const unsigned sonar_count = sizeof(sonar_values) / sizeof(sonar_values[0]);

	sonar_values[insert_index] = insert;
	insert_index++;
	if (insert_index == sonar_count) {
		insert_index = 0;
	}

	/* sort and return mode */

	/* copy ring buffer */
	float sonar_temp[sonar_count];
	memcpy(sonar_temp, sonar_values, sizeof(sonar_values));

	sonar_bubble_sort(sonar_temp, sonar_count);

	/* the center element represents the mode after sorting */
	return sonar_temp[sonar_count / 2];
}


void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
		ultra.height =  ((ultra_tmp<<8) + com_data);
		
		if(ultra.height < 5000) // 5????????,????10?.
		{
			//ultra.relative_height =sonar_filter_oldx(ultra.height);
			ultra.relative_height =insert_sonar_value_and_get_mode_value((float)ultra.height/1000.);
			ultra.measure_ok = 1;		
		}
		else
		{
			ultra.measure_ok = 2; //?????
		}
		ultra_start_f = 0;
	}
	ultra.measure_ot_cnt = 0; //??????(??)
	ultra.h_delta = ultra.relative_height - ultra_distance_old;
	ultra_distance_old = ultra.relative_height;
}




#define Kp 0.3f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.001f                	// 0.001  integral gain governs rate of convergence of gyroscope biases
#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)
xyz_f_t reference_v;
ref_t 	ref;
float reference_vr_imd_down[3],reference_vr[3];
	
float Rol_fc,Pit_fc,Yaw_fc;    				//???
float ref_q_imd_down_fc[4] ;
float reference_vr_imd_down_fc[3];
float ref_q[4] = {1,0,0,0},q_nav[4],ref_q_imd_down[4];
float norm_acc,norm_q;
float norm_acc_lpf;

float mag_norm ,mag_norm_xyz ;

xyz_f_t mag_sim_3d,acc_3d_hg,acc_ng,acc_ng_offset;

void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{	static u8 init;
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	
	if(!init)
	{
	init=1;
	ref.err_tmp.x=ref.err_tmp.y=ref.err_tmp.z=0;
	ref.err.x=ref.err.y=ref.err.z=0;
	ref.err_lpf.x=ref.err_lpf.y=ref.err_lpf.z=0;
	ref.err_Int.x=ref.err_Int.y=ref.err_Int.z=0;
	ref.g.x=ref.g.y=ref.g.z=0;
	}
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
  if(norm_acc==0)
		norm_acc=1;

	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//???????????????
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
	}
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	yaw_correct = 0; //????,????,???????????
	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  
	/* ?????? normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	if(norm_q==0)
		norm_q=1;
	ref_q_imd_down_fc[0]=ref_q[0] = ref_q[0] / norm_q;
	ref_q_imd_down_fc[1]=ref_q[1] = ref_q[1] / norm_q;
	ref_q_imd_down_fc[2]=ref_q[2] = ref_q[2] / norm_q;
	ref_q_imd_down_fc[3]=ref_q[3] = ref_q[3] / norm_q;
	
  reference_vr_imd_down_fc[0] = 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2]);
	reference_vr_imd_down_fc[1] = 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]);
	reference_vr_imd_down_fc[2] = 1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2]);
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
}

#include "mpu6050.h"
#include "i2c_soft.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "bmp.h"
#include "imu.h"
#include "flash_w25.h" 


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
volatile u32 jpeg_data_len=0; 			//buf中的JPEG有效数据长度 
volatile u8 jpeg_data_ok=0;				//JPEG数据采集完成标志 
										
u8 get_one;
u16 off_pix1=0;
//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
u8 MAX_BRIGHT=250;
u16 max_num=3;
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

void jpeg_data_process(void)
{u16 x,y,color,i,j,k,l;
//uint8_t image_buffer_8bit_now[FULL_IMAGE_SIZE];	
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
		gray_balance(image_buffer_8bit_2_t,image_buffer_8bit_2);	
		else	
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
u8 Contract=3;
u8 Effect=0;
void rgb565_test(void)
{ 
 
	u16 x,y,color,i,j,k,l;
	u8 *p;

		//自动对焦初始化
	OV5640_RGB565_Mode();	//RGB565模式 
	OV5640_Focus_Init();	
	OV5640_Light_Mode(0);	//自动模式
	OV5640_Color_Saturation(3);//色彩饱和度0
	OV5640_Brightness(4);	//亮度0
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
	OV5640_Focus_Constant();//启动持续对焦
	My_DCMI_Init();			//DCMI配置
	DCMI_DMA_Init((u32)&jpeg_buf,jpeg_buf_size,DMA_MemoryDataSize_Word,DMA_MemoryInc_Enable);//DCMI DMA配置  
 	//OV5640_ImageWin_Set(4,0,64,64);				//全尺寸缩放
	OV5640_OutSize_Set(0,0,W,H);
	//OV5640_Test_Pattern(2);
	if(en_camera)
	DCMI_Start(); 		//启动传输
	TIM3_Int_Init(680-1,8400-1);//10Khz计数,1秒钟中断一次
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,?????1,????SendBuff,???:SEND_BUF_SIZE.
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	  
	//en_hist_filter=1; 
while(1)
	{ static float t_mpu;
		float dt= Get_Cycle_T(4)/1000000.0f;								//???????????	
    t_mpu+=dt;		
		if(t_mpu>0.015){
		MPU6050_Read();
		MPU6050_Data_Prepare( t_mpu );			
		IMUupdate(0.5f *t_mpu,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z	,&Rol_fc,&Pit_fc,&Yaw_fc);		
		float a_br[3];
		a_br[0] =(float) mpu6050_fc.Acc.x/4096.;//16438.;
		a_br[1] =(float) mpu6050_fc.Acc.y/4096.;//16438.;
		a_br[2] =(float) mpu6050_fc.Acc.z/4096.;//16438.;
		// acc
		float acc_temp[3];
		acc_temp[0] = a_br[1]*reference_vr_imd_down_fc[2]  - a_br[2]*reference_vr_imd_down_fc[1] ;
		acc_temp[1] = a_br[2]*reference_vr_imd_down_fc[0]  - a_br[0]*reference_vr_imd_down_fc[2] ;
	  acc_temp[2] =(reference_vr_imd_down_fc[2] *a_br[2] + reference_vr_imd_down_fc[0] *a_br[0] + reference_vr_imd_down_fc[1] *a_br[1]);
	  float acc_neo_temp[3];
	  acc_neo_temp[0]=-acc_temp[0]*9.87;
		acc_neo_temp[1]=acc_temp[1]*9.87;
		acc_neo_temp[2]=(acc_temp[2]-1.0f)*9.87;		
		acc_flt[0] += ( 1 / ( 1 + 1 / ( K_acc_flt *3.14f *t_mpu ) ) ) *( acc_neo_temp[0] - acc_flt[0] );
	  acc_flt[1] += ( 1 / ( 1 + 1 / ( K_acc_flt *3.14f *t_mpu ) ) ) *( acc_neo_temp[1] - acc_flt[1] );
		acc_flt[2] += ( 1 / ( 1 + 1 / ( K_acc_flt *3.14f *t_mpu ) ) ) *( acc_neo_temp[2] - acc_flt[2] );
		float T=t_mpu;
		double A[9]=
		{1,       0,    0,
		T,       1,    0,
		-T*T/2, -T,    1};

		double B[3]={T*T/2,T,0}; 
		double H1[9]={
		0,0,0,
		0,1,0,
		0,0,0}; 
		float Sdpx=Moving_Median(0,5,flow.h_x_pix);
		float  Accx=acc_flt[0];
	  double Zx[3]={0,Sdpx,0};
		KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H1,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
		float  Sdpy=Moving_Median(1,5,flow.h_y_pix);
		float  Accy=acc_flt[1];
		double Zy[3]={0,Sdpy,0};
		KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H1,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
		flow.x_kf[0]=X_KF_NAV[0][0];
		flow.x_kf[1]=X_KF_NAV[0][1];
		flow.x_kf[2]=X_KF_NAV[0][2];
		flow.y_kf[0]=X_KF_NAV[1][0];
		flow.y_kf[1]=X_KF_NAV[1][1];
		flow.y_kf[2]=X_KF_NAV[1][2];
	  Send_FLOW();
	 if(fc_connect_loss++>33)fc_connect=0;	
		t_mpu=0;} 														
    
		

	//??mpu6????	
		if(get_one==1)	//已经采集完一帧图像了
		{  
		t_flow = Get_Cycle_T(0)/1000000.0f;								//???????????		

														
		static u8 cnt_flow;
		if(en_flow)
		{ 	
		flow_q=flow_task(image_buffer_8bit_2, image_buffer_8bit_1,t_flow);
		for(i=0;i<64*64;i++)
		image_buffer_8bit_1[i]=image_buffer_8bit_2[i];	
		}	
		  
		static uint32_t lasttime = 0;	
    float x_rate = -mpu6050_fc.Gyro_deg.x; // change x and y rates
		float y_rate = -mpu6050_fc.Gyro_deg.y;
		float z_rate = mpu6050_fc.Gyro_deg.z; // z is correct
    float focal_length_px = (3) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled
    uint32_t deltatime = (micros() - lasttime);
		flow.integration_time_us=deltatime ;
		flow.integrated_x=pixel_flow_x  / focal_length_px * 1.0f*k_px; 
		flow.integrated_y=pixel_flow_y  / focal_length_px * -1.0f*k_px;
		flow.integrated_xgyro=x_rate *flow.integration_time_us/1000000.;
		flow.integrated_ygyro=y_rate *flow.integration_time_us/1000000.;
		flow.integrated_zgyro=z_rate *flow.integration_time_us/1000000.;
    flow_pertreatment_oldx( &flow , 1);
		if(sonar_fc==0||!fc_connect)
		ALT_POS_SONAR=ultra.relative_height;
		flow.h_x_pix=flow_per_out[3]*ALT_POS_SONAR;
	  flow.h_y_pix=-flow_per_out[2]*ALT_POS_SONAR;
    lasttime = micros();

		bmp_rx=0;
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
				  static u8 uart_init;
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET||uart_init==0)
								{ uart_init=1;
								DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
								SendBuff2_cnt=0;	

									if(en_jpg){cnt_up_fig=0;
								p=(u8*)JPG_enc_buf;
								for(i=0;i<JUGG_BUF;i++)		//dma传输1次等于4字节,所以乘以4.
								SendBuff2[SendBuff2_cnt++]=p[i];
						  	}else{
							   data_per_uart1(flow.x_kf[1]*100,flow.h_x_pix*100,ALT_POS_SONAR*1000,													 
								flow.integrated_x*100,flow.integrated_xgyro*100,flow.h_x_pix*100,															
								flow.integrated_y*100,flow.integrated_ygyro*100,flow.h_y_pix*100,
						    	Yaw_fc*10,Pit_fc*10,Rol_fc*10,0,0,0,0);			
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
  MPU6050_Init(20);
	Ultrasonic_Init();
	W25QXX_Init();			//W25QXX初始化
	delay_ms(10);
	while(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)	
	{	
		delay_ms(200);
		LED1=!LED1;
	}
  READ_PARM(); 

	while(OV5640_Init())//初始化OV2640
	{
		delay_ms(200);
		LED0=!LED0;
	}	

	  rgb565_test(); 		
}
