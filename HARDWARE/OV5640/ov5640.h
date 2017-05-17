#ifndef _OV5640_H
#define _OV5640_H
#include "sys.h"
#include "sccb.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32������
//OV5640 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/4/30
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//*******************************************************************************
//�޸���Ϣ
//20160226 V1.1
//����OV5640_Exposure����.
////////////////////////////////////////////////////////////////////////////////// 
#define USB_Module 1
	
#if USB_Module	
#define OV5640_RST  	PEout(3)			//POWER DOWN�����ź� 
#define OV5640_PWDN 	PDout(15)			//��λ�����ź� 
#else
#define OV5640_PWDN  	PGout(9)			//POWER DOWN�����ź� 
#define OV5640_RST  	PGout(15)			//��λ�����ź� 
#endif
////////////////////////////////////////////////////////////////////////////////// 
#define OV5640_ID               0X5640  	//OV5640��оƬID
 

#define OV5640_ADDR        		0X78		//OV5640��IIC��ַ
 
//OV5640��ؼĴ�������  
#define OV5640_CHIPIDH          0X300A  	//OV5640оƬID���ֽ�
#define OV5640_CHIPIDL          0X300B  	//OV5640оƬID���ֽ�
 

								
u8 OV5640_WR_Reg(u16 reg,u8 data);
u8 OV5640_RD_Reg(u16 reg);
void OV5640_PWDN_Set(u8 sta);
u8 OV5640_Init(void);  
void OV5640_JPEG_Mode(void);
void OV5640_RGB565_Mode(void);
void OV5640_Exposure(u8 exposure);
void OV5640_Light_Mode(u8 mode);
void OV5640_Color_Saturation(u8 sat);
void OV5640_Brightness(u8 bright);
void OV5640_Contrast(u8 contrast);
void OV5640_Sharpness(u8 sharp);
void OV5640_Special_Effects(u8 eft);
void OV5640_Test_Pattern(u8 mode);
void OV5640_Flash_Ctrl(u8 sw);
u8 OV5640_OutSize_Set(u16 offx,u16 offy,u16 width,u16 height);
u8 OV5640_ImageWin_Set(u16 offx,u16 offy,u16 width,u16 height); 
u8 OV5640_Focus_Init(void);
u8 OV5640_Focus_Single(void);
u8 OV5640_Focus_Constant(void);
#endif




















