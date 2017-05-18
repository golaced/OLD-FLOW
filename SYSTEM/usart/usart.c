#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 


//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
}
float sonar_fc;
u8 fc_connect;
u16 fc_connect_loss=0;
void Data_app(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????
	
		if(*(data_buf+2)==0x01)								//?????,=0x8a,?????
	{ fc_connect=1;
		fc_connect_loss=0;
	  sonar_fc = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
	}

}


u8 Rx_Buf_app[256];	//??????
u8 RxBuffer_app[50];
u8 RxState_app = 0;
u8 RxBufferNum_app = 0;
u8 RxBufferCnt_app = 0;
u8 RxLen_app = 0;
static u8 _data_len_app = 0,_data_cnt_app = 0;

void USART1_IRQHandler(void)
{
	u8 com_data;
	if(USART1->SR & USART_SR_ORE)//ORE??
	{
		com_data = USART1->DR;
	}

  //????
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//??????
   
		com_data = USART1->DR;
	 		if(RxState_app==0&&com_data==0xAA)
		{
			RxState_app=1;
			RxBuffer_app[0]=com_data;
		}
		else if(RxState_app==1&&com_data==0xAF)
		{
			RxState_app=2;
			RxBuffer_app[1]=com_data;
		}
		else if(RxState_app==2&&com_data>0&&com_data<0XF1)
		{
			RxState_app=3;
			RxBuffer_app[2]=com_data;
		}
		else if(RxState_app==3&&com_data<50)
		{
			RxState_app = 4;
			RxBuffer_app[3]=com_data;
			_data_len_app = com_data;
			_data_cnt_app = 0;
		}
		else if(RxState_app==4&&_data_len_app>0)
		{
			_data_len_app--;
			RxBuffer_app[4+_data_cnt_app++]=com_data;
			if(_data_len_app==0)
				RxState_app = 5;
		}
		else if(RxState_app==5)
		{
			RxState_app = 0;
			RxBuffer_app[4+_data_cnt_app]=com_data;
			Data_app(RxBuffer_app,_data_cnt_app+5);
		}
		else
			RxState_app = 0;
	
	}
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}

}

 

void UsartSend_FLOW(uint8_t ch)
{

while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); ;//USART1, ch); 
}

static void Send_Data_FLOW(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_FLOW(dataToSend[i]);
}

#include "flow.h"
void Send_FLOW(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//???
	data_to_send[_cnt++]=0;//???
	
	_temp = (vs16)(flow.x_kf[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(flow.x_kf[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ALT_POS_SONAR*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_FLOW(data_to_send, _cnt);
}






void UsartSend_GOL_LINK(u8 ch)
{

while(USART_GetFlagStatus(USART_LINK, USART_FLAG_TXE) == RESET);
USART_SendData(USART_LINK, ch); 
}

void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}
u8 SendBuff2[SEND_BUF_SIZE2];	//???????
u16 SendBuff2_cnt;
u8 SendBuff1[SEND_BUF_SIZE1];	//???????
u16 SendBuff1_cnt;
int16_t BLE_DEBUG[16];
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{ 
u8 i;	vs16 _temp;
BLE_DEBUG[1]=ax;
BLE_DEBUG[2]=ay;
BLE_DEBUG[3]=az;	
BLE_DEBUG[4]=gx;
BLE_DEBUG[5]=gy;
BLE_DEBUG[6]=gz;
BLE_DEBUG[7]=hx;
BLE_DEBUG[8]=hy;
BLE_DEBUG[9]=hz;
BLE_DEBUG[10]=yaw;
BLE_DEBUG[11]=pitch;
BLE_DEBUG[12]=roll;
BLE_DEBUG[13]=alt;
BLE_DEBUG[13]=tempr;
BLE_DEBUG[14]=press;
BLE_DEBUG[15]=IMUpersec;
	
 	unsigned int temp=0xaF+9;
	char ctemp;
	SendBuff2[SendBuff2_cnt++]=(0xa5);
	SendBuff2[SendBuff2_cnt++]=(0x5a);
	SendBuff2[SendBuff2_cnt++]=(14+8);
	SendBuff2[SendBuff2_cnt++]=(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=ax;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=ay;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=az;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=gx;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=gy;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=gz;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=hx;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=hy;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=hz;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	SendBuff2[SendBuff2_cnt++]=(temp%256);
	SendBuff2[SendBuff2_cnt++]=(0xaa);
	
	temp=0xaF+2+2;

	SendBuff2[SendBuff2_cnt++]=(0xa5);
	SendBuff2[SendBuff2_cnt++]=(0x5a);
	SendBuff2[SendBuff2_cnt++]=(14+4);
	SendBuff2[SendBuff2_cnt++]=(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=yaw;							
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
								 
	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=roll;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;				
	ctemp=alt;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	SendBuff2[SendBuff2_cnt++]=(ctemp);	   
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=press;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	SendBuff2[SendBuff2_cnt++]=(ctemp);
	temp+=ctemp;

	SendBuff2[SendBuff2_cnt++]=(temp%256);
	SendBuff2[SendBuff2_cnt++]=(0xaa);
}

 	
void GOL_LINK(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UsartSend_GOL_LINK(0xa5);
	UsartSend_GOL_LINK(0x5a);
	UsartSend_GOL_LINK(14+8);
	UsartSend_GOL_LINK(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=az;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UsartSend_GOL_LINK(ctemp);
	temp+=ctemp;

	UsartSend_GOL_LINK(temp%256);
	UsartSend_GOL_LINK(0xaa);
}


void uart_init2(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART2_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}