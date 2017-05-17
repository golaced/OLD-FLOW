#ifndef __FLOW_H
#define __FLOW_H	 
#include "stm32f4xx.h" 
#include "math.h" 
#define USE_30FPS 1

u8 RGB_GRAY(u32 RGB,u8 SEL);
u8 RGB_GRAY_16(u16 RGB);
u16 GRAY_RGB(u8 gray);
unsigned short read_xy_rgb(u32 *jpeg_buf_i, unsigned int x,unsigned int y,int W);
u8 flow_task(uint8_t * current_image,uint8_t * previous_image ,float get_time_between_images);
extern double pixel_flow_x,pixel_flow_y;
#endif











