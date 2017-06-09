#ifndef JCAPI_H
#define JCAPI_H
#include "jinclude.h"			 //图像压缩的头文件
#include "stm32f4xx.h"			 //图像压缩的头文件
#include "sys.h"			 //图像压缩的头文件
jpeg_compress_info * jpeg_create_compress (void);     
void jpeg_destory_compress(jpeg_compress_info *cinfo);

void jpeg_set_default(jpeg_compress_info *cinfo,float quality); 
void jpeg_calc_value      (jpeg_compress_info *cinfo);

void jpeg_start_compress  (jpeg_compress_info *cinfo);
void jpeg_finish_compress (jpeg_compress_info *cinfo);

void jpeg_write_scanline  (jpeg_compress_info *cinfo, JSAMPLE *samp_row);
void Compression(int width,int height,float quality) ;
#define JUGG_BUF (int)(64*64*2*1.4)
#define IN_BUF JUGG_BUF
extern unsigned char JPG_enc_buf[JUGG_BUF];//jpeg 输出
extern void RGB565TORGB24(u32 num,u16 Pixel);

extern  unsigned char dcttab[3][512];
extern  unsigned char inbuf_buf[IN_BUF];

#endif /*JCAPI_H*/


