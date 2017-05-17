#ifndef JCAPI_H
#define JCAPI_H
#include "jinclude.h"			 //ͼ��ѹ����ͷ�ļ�
#include "stm32f4xx.h"			 //ͼ��ѹ����ͷ�ļ�
jpeg_compress_info * jpeg_create_compress (void);     
void jpeg_destory_compress(jpeg_compress_info *cinfo);

void jpeg_set_default(jpeg_compress_info *cinfo,float quality); 
void jpeg_calc_value      (jpeg_compress_info *cinfo);

void jpeg_start_compress  (jpeg_compress_info *cinfo);
void jpeg_finish_compress (jpeg_compress_info *cinfo);

void jpeg_write_scanline  (jpeg_compress_info *cinfo, JSAMPLE *samp_row);
void Compression(int width,int height,float quality) ;
#define JUGG_BUF 64*64*4
extern unsigned char JPG_enc_buf[JUGG_BUF];//jpeg ���
extern void RGB565TORGB24(u32 num,u16 Pixel);
#define IN_BUF 25000
extern  unsigned char dcttab[3][512];
extern  unsigned char inbuf_buf[IN_BUF];

#endif /*JCAPI_H*/

