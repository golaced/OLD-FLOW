#include "jinclude.h"			 //图像压缩的头文件
#include "jcapi.h"				 
#include "stm32f4xx.h"	
#include "delay.h"	
#include "usart.h"
#if EN_JPG
unsigned char JPG_enc_buf[JUGG_BUF];//jpeg 输出
volatile unsigned int pt_buf;//缓冲区指针

//u32 inbuf_buf[250000] __attribute__((at(0X68000000)));//测试用数组
//注意：原始RGB565数据和输入缓冲用同一段外扩RAM！我这样做是节省空间!!
jpeg_compress_info info1;
JQUANT_TBL  JQUANT_TBL_2[2];   //量化表
JHUFF_TBL   JHUFF_TBL_4[4];	   //哈弗曼表

//压缩
void Compression(int width,int height,float quality) 
{ u16 i;
	unsigned char *p;
  jpeg_compress_info *cinfo;

  pt_buf=0;
  cinfo=jpeg_create_compress();			   //cinfo:图像输入信息
  cinfo->image_width=width;//设定图片大小
  cinfo->image_height=height;
  cinfo->output=JPG_enc_buf;//设定输出缓冲位置
  jpeg_set_default(cinfo,quality);  
  jpeg_start_compress(cinfo);
  while(cinfo->next_line<cinfo->image_height)//一行一行的扫描进输入缓冲中 
  {
    jpeg_write_scanline(cinfo,&inbuf_buf[(cinfo->next_line*cinfo->image_width*3)]);
  }
  jpeg_finish_compress(cinfo);
//	p=(u8*)JPG_enc_buf;
//					for(i=0;i<JUGG_BUF;i++)		//dma传输1次等于4字节,所以乘以4.
//						UsartSend_GOL_LINK(p[i]); 	
}

#define RGB565_MASK_RED   0xF800//获取红色分量 
#define RGB565_MASK_GREEN 0x07E0//获取绿色分量
#define RGB565_MASK_BLUE  0x001F//获取蓝色分量    
void RGB565TORGB24(u32 num,u16 Pixel)
{    u8 test[3];
    u32 Addr=0;
    Addr=(num)*3;//,0,(64*64-1)*3);
    inbuf_buf[Addr]=(Pixel&RGB565_MASK_RED)>>11;  //R	vs??  获取蓝色分量存入buf右移11位,变为最低5位
    inbuf_buf[Addr+1]=(Pixel&RGB565_MASK_GREEN)>>5; //G	  获取绿色分量存入buf右移5位
    inbuf_buf[Addr+2]=(Pixel&RGB565_MASK_BLUE);    //B
    inbuf_buf[Addr]<<=3; 
    inbuf_buf[Addr+1]<<=2; 
    inbuf_buf[Addr+2]<<=3;  
}
#endif