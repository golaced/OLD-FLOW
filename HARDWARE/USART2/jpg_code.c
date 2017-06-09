#include "jinclude.h"			 //ͼ��ѹ����ͷ�ļ�
#include "jcapi.h"				 
#include "stm32f4xx.h"	
#include "delay.h"	
#include "usart.h"
#if EN_JPG
unsigned char JPG_enc_buf[JUGG_BUF];//jpeg ���
volatile unsigned int pt_buf;//������ָ��

//u32 inbuf_buf[250000] __attribute__((at(0X68000000)));//����������
//ע�⣺ԭʼRGB565���ݺ����뻺����ͬһ������RAM�����������ǽ�ʡ�ռ�!!
jpeg_compress_info info1;
JQUANT_TBL  JQUANT_TBL_2[2];   //������
JHUFF_TBL   JHUFF_TBL_4[4];	   //��������

//ѹ��
void Compression(int width,int height,float quality) 
{ u16 i;
	unsigned char *p;
  jpeg_compress_info *cinfo;

  pt_buf=0;
  cinfo=jpeg_create_compress();			   //cinfo:ͼ��������Ϣ
  cinfo->image_width=width;//�趨ͼƬ��С
  cinfo->image_height=height;
  cinfo->output=JPG_enc_buf;//�趨�������λ��
  jpeg_set_default(cinfo,quality);  
  jpeg_start_compress(cinfo);
  while(cinfo->next_line<cinfo->image_height)//һ��һ�е�ɨ������뻺���� 
  {
    jpeg_write_scanline(cinfo,&inbuf_buf[(cinfo->next_line*cinfo->image_width*3)]);
  }
  jpeg_finish_compress(cinfo);
//	p=(u8*)JPG_enc_buf;
//					for(i=0;i<JUGG_BUF;i++)		//dma����1�ε���4�ֽ�,���Գ���4.
//						UsartSend_GOL_LINK(p[i]); 	
}

#define RGB565_MASK_RED   0xF800//��ȡ��ɫ���� 
#define RGB565_MASK_GREEN 0x07E0//��ȡ��ɫ����
#define RGB565_MASK_BLUE  0x001F//��ȡ��ɫ����    
void RGB565TORGB24(u32 num,u16 Pixel)
{    u8 test[3];
    u32 Addr=0;
    Addr=(num)*3;//,0,(64*64-1)*3);
    inbuf_buf[Addr]=(Pixel&RGB565_MASK_RED)>>11;  //R	vs??  ��ȡ��ɫ��������buf����11λ,��Ϊ���5λ
    inbuf_buf[Addr+1]=(Pixel&RGB565_MASK_GREEN)>>5; //G	  ��ȡ��ɫ��������buf����5λ
    inbuf_buf[Addr+2]=(Pixel&RGB565_MASK_BLUE);    //B
    inbuf_buf[Addr]<<=3; 
    inbuf_buf[Addr+1]<<=2; 
    inbuf_buf[Addr+2]<<=3;  
}
#endif