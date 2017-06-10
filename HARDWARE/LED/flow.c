#include "flow.h"
#include "filter.h"
#include "stdlib.h"
#define KLT2 0
#define SAD2 0
#define USE_SAD 1

#define FRAME_SIZE	64


#define SEARCH_SIZE 8//	4*2 ;// maximum offset to search: 4 + 1/2 pixels
#define NUM_BLOCKS	6//5*2 ;// x & y number of tiles to check
#define TILE_SIZE	SEARCH_SIZE*2//8*2;  
u16 PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD=10;//14;//100;
u16 PARAM_BOTTOM_FLOW_VALUE_THRESHOLD=3500;//5000;
float PARAM_GYRO_COMPENSATION_THRESHOLD=0.1;

int CORNER_THRE = 10;
int SAD_THRE = 3000;
int SEARCH_SIZE1 = 8;        // 8 maximum offset to search: 4 + 1/2 pixels  (better do not bigger than 8)
int NUM_BLOCKS1 = 5;         // 5 do not bigger than 8 ,do not less than 4
#define IMAGE_COLS 64
#define IMAGE_ROWS 64

u8 meancount_set=5;
u8 en_hist_filter=1;
u8 en_gro_filter=0;

#define  NUM_BLOCKS_KLT	8//3 8 x & y number of tiles to check  6
//this are the settings for KLT based flow
#define PYR_LVLS 1
#define HALF_PATCH_SIZE 4       //this is half the wanted patch size minus 1  6
#define PATCH_SIZE (HALF_PATCH_SIZE*2+1)
u8 meancount_set_klt=1;
u8 max_iter=5;

#if USE_30FPS
u8 gray_scal=8;
#else
u8 gray_scal=8;
#endif
u8 RGB_GRAY(u32 RGB,u8 SEL)
{u8 Gray,Gray_r;
 u16 rgb_use;
u8 R,G,B;
	if(SEL)
		rgb_use=RGB;
	else
		rgb_use=RGB>>16;
	
R = (rgb_use & 0xF800) >> 8;
G = (rgb_use & 0x07E0) >> 3; 
B = (rgb_use & 0x001F) << 3;

//Gray_r=(R*77 + G*150 + B*29+128) >>11;// (256*gray_scal);
Gray_r=((R<<6) + (G<<7) + (B<<5)+128) >>11;// (256*gray_scal);	
return Gray= Gray_r;
}

u8 RGB_GRAY_16(u16 RGB)
{u8 Gray,Gray_r;
 u16 rgb_use;
u8 R,G,B;

R = (RGB & 0xF800) >> 8;
G = (RGB & 0x07E0) >> 3; 
B = (RGB & 0x001F) << 3;

Gray_r=(R*77 + G*150 + B*29+128)/(256*gray_scal);// >>11;// (256*gray_scal);
//Gray_r=((R<<6) + (G<<7) + (B<<5)+128) >>11;// (256*gray_scal);	
return Gray= Gray_r;
}

u16 GRAY_RGB(u8 gray)
{
 u16 RGB;
//gray=gray/8;
RGB=(0x001f&gray)<<11;
RGB=RGB|(((0x003f)&(gray*2))<<5);
RGB=RGB|(0x001f&gray);
	
return RGB;
}

//读取RBG格式颜色，唯一需要移植的函数
//extern unsigned short GUI_ReadBit16Point(unsigned short x,unsigned short y);
u8 off_pix=112;
unsigned short read_xy_rgb(u32 *jpeg_buf_i, unsigned int x,unsigned int y,int W)
{unsigned short temp;
	u16 i,j,k,l;
	unsigned int yt;
if(y<=W-off_pix){
	if((y)%2!=0)
			temp=jpeg_buf_i[(x)*W/2+(-off_pix+y)/2]>>16;
	else
			temp=jpeg_buf_i[(x)*W/2+(-off_pix+y)/2];
}
else{
		if((y)%2!=0)
			temp=jpeg_buf_i[(x)*W/2+(W-off_pix+y)/2]>>16;
	else
			temp=jpeg_buf_i[(x)*W/2+(W-off_pix+y)/2];
}

return temp;
}


#if !SAD2
#define sign(x) (( x > 0 ) - ( x < 0 ))

// compliments of Adam Williams
#define ABSDIFF(frame1, frame2) \
({ \
 int result = 0; \
 asm volatile( \
  "mov %[result], #0\n"           /* accumulator */ \
 \
  "ldr r4, [%[src], #0]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #0]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #4]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #4]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 1)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 1)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 1 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 1 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 2)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 2)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 2 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 2 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 3)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 3)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 3 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 3 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 4)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 4 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 4 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 5)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 5)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 5 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 5 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 6)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 6)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 6 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 6 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 7)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 7)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 7 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 7 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  : [result] "+r" (result) \
  : [src] "r" (frame1), [dst] "r" (frame2) \
  : "r4", "r5" \
  ); \
  \
 result; \
})

/**
 * @brief Computes the Hessian at a pixel location
 *
 * The hessian (second order partial derivatives of the image) is
 * a measure of the salience of the image at the appropriate
 * box filter scale. It allows to judge wether a pixel
 * location is suitable for optical flow calculation.
 *
 * @param image the array holding pixel data
 * @param x location of the pixel in x
 * @param y location of the pixel in y
 *
 * @return gradient magnitude
 */
static __inline uint32_t compute_hessian_4x6(uint8_t *image, uint16_t x, uint16_t y, uint16_t row_size)
{
	// candidate for hessian calculation:
	uint16_t off1 = y*row_size + x;   	// First row of ones
	uint16_t off2 = (y+1)*row_size + x;   // Second row of ones
	uint16_t off3 = (y+2)*row_size + x;   // Third row of minus twos
	uint16_t off4 = (y+3)*row_size + x;   // Third row of minus twos
	uint16_t off5 = (y+4)*row_size + x;   // Third row of minus twos
	uint16_t off6 = (y+5)*row_size + x;   // Third row of minus twos
	uint32_t magnitude;

	// Uncentered for max. performance:
	// center pixel is in brackets ()

	//  1   1   1   1
	//  1   1   1   1
	// -2 (-2) -2  -2
	// -2  -2  -2  -2
	//  1   1   1   1
	//  1   1   1   1

	magnitude = __UADD8(*((uint32_t*) &image[off1 - 1]), *((uint32_t*) &image[off2 - 1]));
	magnitude -= 2*__UADD8(*((uint32_t*) &image[off3 - 1]), *((uint32_t*) &image[off4 - 1]));
	magnitude += __UADD8(*((uint32_t*) &image[off5 - 1]), *((uint32_t*) &image[off6 - 1]));

	return magnitude;
}


/**
 * @brief Compute the average pixel gradient of all horizontal and vertical steps
 *
 * TODO compute_diff is not appropriate for low-light mode images
 *
 * @param image ...
 * @param offX x coordinate of upper left corner of 8x8 pattern in image
 * @param offY y coordinate of upper left corner of 8x8 pattern in image
 */
static __inline uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off = (offY + 2) * row_size + (offX + 2); // we calc only the 4x4 pattern
	uint32_t acc;
	uint32_t col1,col2,col3,col4;
	/* calc row diff */
	acc = __USAD8 (*((uint32_t*) &image[off + 0 + 0 * row_size]), *((uint32_t*) &image[off + 0 + 1 * row_size]));
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 1 * row_size]), *((uint32_t*) &image[off + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 2 * row_size]), *((uint32_t*) &image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	 col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 0 + 1 * row_size] << 16 | image[off + 0 + 2 * row_size] << 8 

| image[off + 0 + 3 * row_size];
	 col2 = (image[off + 1 + 0 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 1 + 2 * row_size] << 8 

| image[off + 1 + 3 * row_size];
	 col3 = (image[off + 2 + 0 * row_size] << 24) | image[off + 2 + 1 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 

| image[off + 2 + 3 * row_size];
	 col4 = (image[off + 3 + 0 * row_size] << 24) | image[off + 3 + 1 * row_size] << 16 | image[off + 3 + 2 * row_size] << 8 

| image[off + 3 + 3 * row_size];

	/* calc column diff */
	acc = __USADA8(col1, col2, acc);
	acc = __USADA8(col2, col3, acc);
	acc = __USADA8(col3, col4, acc);

	return acc;

}

/**
 * @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 * @param acc array to store SAD distances for shift in every direction
 */
static __inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, 

uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2
	uint16_t i;
	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for ( i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	 * calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	 * every iteration is one line of the 8x8 field.
	 *
	 *  + - + - + - + - + - + - + - + - +
	 *  |   |   |   |   |   |   |   |   |
	 *  + - + - + - + - + - + - + - + - +
	 *
	 *
	 */

	for ( i = 0; i < 8; i++)
	{
		/*
		 * first column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  | x | x | x | x |   |   |   |   |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 * the 8 s values are from following positions for each pixel (X):
		 *  + - + - + - +
		 *  +   5   7   +
		 *  + - + 6 + - +
		 *  +   4 X 0   +
		 *  + - + 2 + - +
		 *  +   3   1   +
		 *  + - + - + - +
		 *
		 *  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		 *
		 */

		/* compute average of two pixel values */
		s0 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+0) * 

row_size])));
		s1 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+1) * 

row_size])));
		s2 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i+1) * 

row_size])));
		s3 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+1) * 

row_size])));
		s4 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+0) * 

row_size])));
		s5 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i-1) * 

row_size])));
		s6 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i-1) * 

row_size])));
		s7 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i-1) * 

row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		/*
		 * finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		 *  + - + - + - +
		 *  |   |   |   |
		 *  + - 5 6 7 - +
		 *  |   4 X 0   |
		 *  + - 3 2 1 - +
		 *  |   |   |   |
		 *  + - + - + - +
		 */

		/* fill accumulation vector */
		acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		 * same for second column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  |   |   |   |   | x | x | x | x |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 */

		s0 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+0) * 

row_size])));
		s1 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+1) * 

row_size])));
		s2 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i+1) * 

row_size])));
		s3 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+1) * 

row_size])));
		s4 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+0) * 

row_size])));
		s5 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i-1) * 

row_size])));
		s6 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i-1) * 

row_size])));
		s7 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i-1) * 

row_size])));

		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}

/**
 * @brief Compute SAD of two 8x8 pixel windows.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 */
static __inline uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, 

uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	acc = __USAD8 (*((uint32_t*) &image1[off1 + 0 + 0 * row_size]), *((uint32_t*) &image2[off2 + 0 + 0 * row_size]));
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 0 * row_size]), *((uint32_t*) &image2[off2 + 4 + 0 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 1 * row_size]), *((uint32_t*) &image2[off2 + 0 + 1 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 1 * row_size]), *((uint32_t*) &image2[off2 + 4 + 1 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 2 * row_size]), *((uint32_t*) &image2[off2 + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 2 * row_size]), *((uint32_t*) &image2[off2 + 4 + 2 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 3 * row_size]), *((uint32_t*) &image2[off2 + 0 + 3 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 3 * row_size]), *((uint32_t*) &image2[off2 + 4 + 3 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 4 * row_size]), *((uint32_t*) &image2[off2 + 0 + 4 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 4 * row_size]), *((uint32_t*) &image2[off2 + 4 + 4 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 5 * row_size]), *((uint32_t*) &image2[off2 + 0 + 5 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 5 * row_size]), *((uint32_t*) &image2[off2 + 4 + 5 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 6 * row_size]), *((uint32_t*) &image2[off2 + 0 + 6 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 6 * row_size]), *((uint32_t*) &image2[off2 + 4 + 6 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 7 * row_size]), *((uint32_t*) &image2[off2 + 0 + 7 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 7 * row_size]), *((uint32_t*) &image2[off2 + 4 + 7 * row_size]), acc);

	return acc;
}

/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * and calculates the average offset of all.
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */
uint16_t meancount_view = 0;

float y_rate_view;
uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y,float get_time_between_images) {
	/* constants */
	const int16_t winmin = -SEARCH_SIZE;
	const int16_t winmax = SEARCH_SIZE;
	const uint16_t hist_size = 2*(winmax-winmin+1)+1;
	//Start change
    int tempCount=4;
    float tempx[4];
    float tempy[4];
    for(int i=0;i<tempCount;++i)
    {
        tempx[i]=0.0f;
        tempy[i]=0.0f;
    }
    //End change
    
    //Start change
    uint16_t imageSizeCount=64*64;
    float imageChangeFactor=0.0f;
    uint32_t image1Sum=0;
    uint32_t image2Sum=0;
    for(int i=0;i<imageSizeCount;++i)
    {
        image1Sum+=*(image1+i);
        image2Sum+=*(image2+i);
    }
    if (image1Sum>=image2Sum)
    {
        imageChangeFactor=image1Sum/image2Sum;
        for(int j=0;j<imageSizeCount;++j)
        {
            if(*(image2+j)*imageChangeFactor<=256)
                *(image2+j)*=imageChangeFactor;
            else
                *(image2+j)=255;
        }
    }
    else
    {
        imageChangeFactor=image2Sum/image1Sum;
        for(int j=0;j<imageSizeCount;++j)
        {
            if(*(image1+j)*imageChangeFactor<=256)
                *(image1+j)*=imageChangeFactor;
            else
                *(image1+j)=255;
        }
    }
    //When image i < avg(image) =0
    for(int i=0;i<imageSizeCount;++i)
    {
        image1Sum+=*(image1+i);
        image2Sum+=*(image2+i);
    }
    for(int i=0;i<imageSizeCount;++i)
    {
        if(*(image1+i)<image1Sum/imageSizeCount/4)
            *(image1+i)=0;
        if(*(image2+i)<image2Sum/imageSizeCount/4)
            *(image2+i)=0;
    }
    //End change
    
		
	uint8_t qual;
	/* variables */
	uint16_t pixLo = SEARCH_SIZE + 1;
	uint16_t pixHi = FRAME_SIZE - (SEARCH_SIZE + 1) - TILE_SIZE;
	uint16_t pixStep = (pixHi - pixLo) / NUM_BLOCKS + 1;
//	
  //uint16_t pixHi = IMAGE_ROWS - (SEARCH_SIZE + 1) - 2*SEARCH_SIZE;
  //uint16_t pixStep = (pixHi - pixLo) / (NUM_BLOCKS-1);

	uint16_t i, j;
	uint32_t acc_flow[8]; // subpixels
	uint16_t histx[64]; // counter for x shift
	uint16_t histy[64]; // counter for y shift
	int8_t  dirsx[64]; // shift directions in x
	int8_t  dirsy[64]; // shift directions in y
	uint8_t  subdirs[64]; // shift directions of best subpixels
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;
	uint32_t diff;
	/* initialize with 0 */
	uint32_t dist;
	int8_t sumx ;
	int8_t sumy;
	int8_t ii, jj;
	uint32_t temp_dist;
	uint8_t k;
	uint32_t mindist = dist; // best SAD until now
	uint8_t mindir = 8; // dir
	uint8_t hist_index_x ;
	uint8_t hist_index_y;
	int16_t maxpositionx = 0;
	int16_t maxpositiony = 0;
	uint16_t maxvaluex = 0;
	uint16_t maxvaluey = 0;
	uint16_t hist_x_min = 0;
	uint16_t hist_x_max = 0;
	uint16_t hist_y_min = 0;
	uint16_t hist_y_max = 0;
	float hist_x_value = 0.0f;
	float hist_x_weight = 0.0f;
	float focal_length_px = (16) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled
	float hist_y_value = 0.0f;
	float hist_y_weight = 0.0f;
	/* use average of accepted flow values */
	uint32_t meancount_x = 0;
	float subdiry=0;
	uint32_t meancount_y = 0;
	uint8_t *base1;//(uint16_t) global_data.param[PARAM_IMAGE_WIDTH] + i;
	uint8_t *base2;

for (j = 0; j < hist_size; j++) { histx[j] = 0; histy[j] = 0; }

	/* iterate over all patterns
	 */
	for (j = pixLo; j < pixHi; j += pixStep)
	{
		for (i = pixLo; i < pixHi; i += pixStep)
		{
			/* test pixel if it is suitable for flow tracking */
			 diff = compute_diff(image1, i, j,FRAME_SIZE);// (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
			if (diff < PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD)//global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD])
			{
				continue;
			}

			 dist = 0xFFFFFFFF; // set initial distance to "infinity"
			 sumx = 0;
			 sumy = 0;

			
				 base1 = image1 + j * FRAME_SIZE+i;
			for (jj = winmin; jj <= winmax; jj++)
			{
				 base2 = image2 + (j+jj) *  FRAME_SIZE+i;//(uint16_t) global_data.param[PARAM_IMAGE_WIDTH] + i;

				for (ii = winmin; ii <= winmax; ii++)
				{
 				 temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj,FRAME_SIZE);// (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
		     //temp_dist = ABSDIFF(base1, base2 + ii);
					if (temp_dist < dist)
					{
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			if (dist < PARAM_BOTTOM_FLOW_VALUE_THRESHOLD)//global_data.param[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD])
			{
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc_flow, FRAME_SIZE);//(uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
				 mindist = dist; // best SAD until now
				 mindir = 8; // direction 8 for no direction
				for( k = 0; k < 8; k++)
				{
					if (acc_flow[k] < mindist)
					{
						// SAD becomes better in direction k
						mindist = acc_flow[k];
						mindir = k;
					}
				}
				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
				subdirs[meancount] = mindir;
				meancount++;

				/* feed histogram filter*/
				 hist_index_x = 2*sumx + (winmax-winmin+1);
				if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) hist_index_x += 1;
				if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) hist_index_x += -1;
				 hist_index_y = 2*sumy + (winmax-winmin+1);
				if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) hist_index_y += -1;
				if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) hist_index_y += 1;

				histx[hist_index_x]++;
				histy[hist_index_y]++;

			}
		}
	}
meancount_view=0.2*meancount+0.8*meancount_view;


	/* evaluate flow calculation */
	if (meancount > meancount_set)
	{
		meanflowx /= meancount;
		meanflowy /= meancount;

		 maxpositionx = 0;
		 maxpositiony = 0;
		 maxvaluex = 0;
		 maxvaluey = 0;

		/* position of maximal histogram peek */
		for (j = 0; j < hist_size; j++)
		{
			if (histx[j] > maxvaluex)
			{
				maxvaluex = histx[j];
				maxpositionx = j;
			}
			if (histy[j] > maxvaluey)
			{
				maxvaluey = histy[j];
				maxpositiony = j;
			}
		}

		/* check if there is a peak value in histogram */
		if(1)// (histx[maxpositionx] > meancount / 6 && histy[maxpositiony] > meancount / 6)
		{
			if (en_hist_filter)//global_data.param[PARAM_BOTTOM_FLOW_HIST_FILTER])
			{

				/* use histogram filter peek value */
				 hist_x_min = maxpositionx;
				 hist_x_max = maxpositionx;
				 hist_y_min = maxpositiony;
				 hist_y_max = maxpositiony;

				/* x direction */
				if (maxpositionx > 1 && maxpositionx < hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 2;
				}
				else if (maxpositionx == 0)
				{
					hist_x_min = maxpositionx;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-1)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx;
				}
				else if (maxpositionx == 1)
				{
					hist_x_min = maxpositionx - 1;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 1;
				}

				/* y direction */
				if (maxpositiony > 1 && maxpositiony < hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == 0)
				{
					hist_y_min = maxpositiony;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-1)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony;
				}
				else if (maxpositiony == 1)
				{
					hist_y_min = maxpositiony - 1;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 1;
				}

				 hist_x_value = 0.0f;
				 hist_x_weight = 0.0f;

				 hist_y_value = 0.0f;
				 hist_y_weight = 0.0f;

				for ( i = hist_x_min; i < hist_x_max+1; i++)
				{
					hist_x_value += (float) (i*histx[i]);
					hist_x_weight += (float) histx[i];
				}

				for ( i = hist_y_min; i<hist_y_max+1; i++)
				{
					hist_y_value += (float) (i*histy[i]);
					hist_y_weight += (float) histy[i];
				}

				histflowx = (hist_x_value/hist_x_weight - (winmax-winmin+1)) / 2.0f ;
				histflowy = (hist_y_value/hist_y_weight - (winmax-winmin+1)) / 2.0f;
							//Start change
                for(int i=tempCount-1;i>=1;--i)
                {
                    tempx[i]=tempx[i-1];
                    tempy[i]=tempy[i-1];
                }
                tempx[0]=histflowx;
                tempy[0]=histflowy;
                //End change
                
			}
			else
			{

				/* use average of accepted flow values */
				 meancount_x = 0;
				 meancount_y = 0;

				for ( i = 0; i < meancount; i++)
				{
					float subdirx = 0.0f;
					if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) subdirx = 0.5f;
					if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) subdirx = -0.5f;
					histflowx += (float)dirsx[i] + subdirx;
					meancount_x++;

					 subdiry = 0.0f;
					if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) subdiry = -0.5f;
					if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) subdiry = 0.5f;
					histflowy += (float)dirsy[i] + subdiry;
					meancount_y++;
				}

				histflowx /= meancount_x;
				histflowy /= meancount_y;
								//Start change
                for(int i=tempCount-1;i>=1;--i)
                {
                    tempx[i]=tempx[i-1];
                    tempy[i]=tempy[i-1];
                }
                tempx[0]=histflowx;
                tempy[0]=histflowy;
                //End change
			}

			/* compensate rotation */
			/* calculate focal_length in pixel */
			

			/*
			 * gyro compensation
			 * the compensated value is clamped to
			 * the maximum measurable flow value (param BFLOW_MAX_PIX) +0.5
			 * (sub pixel flow can add half pixel to the value)
			 *
			 * -y_rate gives x flow
			 * x_rates gives y_flow
			 */
			if (en_gro_filter)
			{y_rate_view=y_rate;
				if(fabsf(y_rate) > PARAM_GYRO_COMPENSATION_THRESHOLD)
				{
					/* calc pixel of gyro */
					float y_rate_pixel = y_rate * (get_time_between_images) * focal_length_px;
					float comp_x = histflowx + y_rate_pixel;

                    /* clamp value to maximum search window size plus half pixel from subpixel search */
                    if (comp_x < (-SEARCH_SIZE - 0.5f))
                    	*pixel_flow_x = (-SEARCH_SIZE - 0.5f);
                    else if (comp_x > (SEARCH_SIZE + 0.5f))
                    	*pixel_flow_x = (SEARCH_SIZE + 0.5f);
                    else
                    	*pixel_flow_x = comp_x;
				}
				else
				{
					*pixel_flow_x = histflowx;
				}

				if(fabsf(x_rate) >PARAM_GYRO_COMPENSATION_THRESHOLD)
				{
					/* calc pixel of gyro */
					float x_rate_pixel = x_rate * (get_time_between_images) * focal_length_px;
					float comp_y = histflowy - x_rate_pixel;

					/* clamp value to maximum search window size plus/minus half pixel from subpixel search 

*/
					if (comp_y < (-SEARCH_SIZE - 0.5f))
						*pixel_flow_y = (-SEARCH_SIZE - 0.5f);
					else if (comp_y > (SEARCH_SIZE + 0.5f))
						*pixel_flow_y = (SEARCH_SIZE + 0.5f);
					else
						*pixel_flow_y = comp_y;
				}
				else
				{
					*pixel_flow_y = histflowy;
				}

				/* alternative compensation */
//				/* compensate y rotation */
//				*pixel_flow_x = histflowx + y_rate_pixel;
//
//				/* compensate x rotation */
//				*pixel_flow_y = histflowy - x_rate_pixel;

			} else
			{
				/* without gyro compensation */
				*pixel_flow_x = histflowx;
				*pixel_flow_y = histflowy;
			}

		}
		else
		{
			*pixel_flow_x = 0.0f;
			*pixel_flow_y = 0.0f;
			return 0;
		}
	}
	else
	{
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}
  float scale_x = 2.0f;
	float scale_y = 2.0f;

    
    //Start change #Soft the waves
    float tempxSum=0.0f;
    float tempySum=0.0f;
    
    for(int i=0;i<tempCount;++i)
    {
        tempxSum+=tempx[i];
        tempySum+=tempy[i];
    }
    
   // *pixel_flow_x = (*pixel_flow_x*1+tempxSum)/tempCount*scale_x;
   // *pixel_flow_y = (*pixel_flow_y*1+tempySum)/tempCount*scale_y;
	/* calc quality */
	 qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS*NUM_BLOCKS));

	return qual;
}




float Jx[PATCH_SIZE*PATCH_SIZE];
float Jy[PATCH_SIZE*PATCH_SIZE];
u8 meancount_v_klt;
/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * with the KLT method and outputs the average value of all flow vectors
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */
/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * with the KLT method and outputs the average value of all flow vectors
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */
//??????????????
//Kanade-Lucas-Tomasi(KLT)
uint8_t compute_klt(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y)
{
  /* variables */
  uint16_t i, j;

  float meanflowx = 0.0f;
  float meanflowy = 0.0f;
  uint16_t meancount = 0;

  /*
   * compute image pyramid for current frame
   * there is 188*120 bytes per buffer, we are only using 64*64 per buffer,
   * so just add the pyramid levels after the image
   */
  //first compute the offsets in the memory for the pyramid levels
  uint16_t lvl_off[PYR_LVLS];
  uint16_t s = FRAME_SIZE;//64
  uint16_t off = 0;
  for (int l = 0; l < PYR_LVLS; l++)//PYR_LVLS = 2
  {
    lvl_off[l] = off;//lvl_off[0] = 0,s = 64;lvl_off[1] = 64*64,s=32
    off += s*s;
    s /= 2;
  }
  
  //then subsample the images consecutively, no blurring is done before the subsampling (if someone volunteers, please go ahead...)
  //???????,???
  for (int l = 1; l < PYR_LVLS; l++)
  {
    uint16_t src_size = FRAME_SIZE >> (l-1);
    uint16_t tar_size = FRAME_SIZE >> l;
    uint8_t *source = &image2[lvl_off[l-1]]; //pointer to the beginning of the previous level
    uint8_t *target = &image2[lvl_off[l]];   //pointer to the beginning of the current level
    for (j = 0; j < tar_size; j++)
      for (i = 0; i < tar_size; i+=2)
      {
        //subsample the image by 2, use the halving-add instruction to do so
        uint32_t l1 = (__UHADD8(*((uint32_t*) &source[(j*2+0)*src_size + i*2]), *((uint32_t*) &source[(j*2+0)*src_size + i*2+1])));
        uint32_t l2 = (__UHADD8(*((uint32_t*) &source[(j*2+1)*src_size + i*2]), *((uint32_t*) &source[(j*2+1)*src_size + i*2+1])));
        uint32_t r = __UHADD8(l1, l2);

        //the first and the third byte are the values we want to have
        target[j*tar_size + i+0] = (uint8_t) r;
        target[j*tar_size + i+1] = (uint8_t) (r>>16);
      }
  }

  //need to store the flow values between pyramid level changes
  //??????????????,???????????
  float us[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];
  float vs[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];
  uint16_t is[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];
  uint16_t js[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];

  //initialize flow values with the pixel value of the previous image
  //???????????,??????
  uint16_t pixLo = FRAME_SIZE / (NUM_BLOCKS_KLT + 1);
  if (pixLo < PATCH_SIZE) pixLo = PYR_LVLS * PATCH_SIZE;
  uint16_t pixHi = (uint16_t)FRAME_SIZE * NUM_BLOCKS_KLT / (NUM_BLOCKS_KLT + 1);
  if (pixHi > (FRAME_SIZE - PATCH_SIZE)) pixHi = FRAME_SIZE - (PYR_LVLS * PATCH_SIZE);
  uint16_t pixStep = (pixHi - pixLo) / (NUM_BLOCKS_KLT-1);

  j = pixLo;
  for (int y = 0; y < NUM_BLOCKS_KLT; y++, j += pixStep)
  {
    i = pixLo;
    for (int x = 0; x < NUM_BLOCKS_KLT; x++, i += pixStep)
    {
      //TODO: for proper rotation compensation, insert gyro values here and then substract at the end
      us[y*NUM_BLOCKS_KLT+x] = i; //position in new image at level 0
      vs[y*NUM_BLOCKS_KLT+x] = j;
      is[y*NUM_BLOCKS_KLT+x] = i; //position in previous image  at level 0
      js[y*NUM_BLOCKS_KLT+x] = j;
    }
  }

  //for all pyramid levels, start from the smallest level
  //??????
  for (int l = PYR_LVLS-1; l >= 0; l--)
  {
    //iterate over all patterns
    //??????,????????
    for (int k = 0; k < NUM_BLOCKS_KLT*NUM_BLOCKS_KLT; k++)
    {
        uint16_t i = is[k] >> l;  //reference pixel for the current level
        uint16_t j = js[k] >> l;

        uint16_t iwidth = FRAME_SIZE >> l;
        uint8_t *base1 = image1 + lvl_off[l] + j * iwidth + i;

        float JTJ[4];   //the 2x2 Hessian
        JTJ[0] = 0;
        JTJ[1] = 0;
        JTJ[2] = 0;
        JTJ[3] = 0;
        int c = 0;

        //compute jacobians and the hessian for the patch at the current location
        //???????jacobians-hessian???????????
        for (int8_t jj = -HALF_PATCH_SIZE; jj <= HALF_PATCH_SIZE; jj++)//HALF_PATCH_SIZE = 4, this is half the wanted patch size minus 1
        {
          uint8_t *left = base1 + jj*iwidth;
          for (int8_t ii = -HALF_PATCH_SIZE; ii <= HALF_PATCH_SIZE; ii++)//????????????????
          {
            const float jx = ((uint16_t)left[ii+1] - (uint16_t)left[ii-1]) * 0.5f;
            const float jy = ((uint16_t)left[ii+iwidth] - (uint16_t)left[ii-iwidth]) * 0.5f;
            Jx[c] = jx;
            Jy[c] = jy;
            JTJ[0] += jx*jx;//????,??????????
            JTJ[1] += jx*jy;//???????????????
            JTJ[2] += jx*jy;
            JTJ[3] += jy*jy;
            c++;
          }
        }

        // us and vs store the sample position in level 0 pixel coordinates
        float u = (us[k] / (1<<l));
        float v = (vs[k] / (1<<l));

        float chi_sq_previous = 0.f;

        //Now do some Gauss-Newton iterations for flow
        for (int iters = 0; iters < 5; iters++)
        {
          float JTe_x = 0;  //accumulators for Jac transposed times error
          float JTe_y = 0;

          uint8_t *base2 = image2 + lvl_off[l] + (uint16_t)v * iwidth + (uint16_t)u;

          //extract bilinearly filtered pixel values for the current location in image2
          float dX = u - floorf(u);
          float dY = v - floorf(v);
          float fMixTL = (1.f - dX) * (1.f - dY);
          float fMixTR = (dX) * (1.f - dY);
          float fMixBL = (1.f - dX) * (dY);
          float fMixBR = (dX) * (dY);

          float chi_sq = 0.f;
          int c = 0;
          for (int8_t jj = -HALF_PATCH_SIZE; jj <= HALF_PATCH_SIZE; jj++)//-4->4
          {
            uint8_t *left1 = base1 + jj*iwidth;
            uint8_t *left2 = base2 + jj*iwidth;

            for (int8_t ii = -HALF_PATCH_SIZE; ii <= HALF_PATCH_SIZE; ii++)
            {
              float fPixel = fMixTL * left2[ii] + fMixTR * left2[ii+1] + fMixBL * left2[ii+iwidth] + fMixBR * left2[ii+iwidth+1];
              float fDiff = fPixel - left1[ii];
              JTe_x += fDiff * Jx[c];
              JTe_y += fDiff * Jy[c];
              chi_sq += fDiff*fDiff;
              c++;
            }
          }

          //only update if the error got smaller
          if (iters == 0 || chi_sq_previous > chi_sq)
          {
            //compute inverse of hessian
            float det = (JTJ[0]*JTJ[3]-JTJ[1]*JTJ[2]);
            if (det != 0.f)
            {
              float detinv = 1.f / det;
              float JTJinv[4];
              JTJinv[0] = detinv * JTJ[3];
              JTJinv[1] = detinv * -JTJ[1];
              JTJinv[2] = detinv * -JTJ[2];
              JTJinv[3] = detinv * JTJ[0];

              //compute update and shift current position accordingly
              float updx = JTJinv[0]*JTe_x + JTJinv[1]*JTe_y;
              float updy = JTJinv[2]*JTe_x + JTJinv[3]*JTe_y;
              float new_u = u-updx;
              float new_v = v-updy;

              //check if we drifted outside the image
              if (((int16_t)new_u < HALF_PATCH_SIZE) || (int16_t)new_u > (iwidth-HALF_PATCH_SIZE-1) || ((int16_t)new_v < HALF_PATCH_SIZE) || (int16_t)new_v > (iwidth-HALF_PATCH_SIZE-1))
              {
                break;
              }
              else
              {
                u = new_u;
                v = new_v;
              }
            }
            else
              break;
          }
          else
            break;

          chi_sq_previous = chi_sq;
        }//??????????

        if (l > 0)
        {
          us[k] = u * (1<<l);
          vs[k] = v * (1<<l);
        }
        else  //for the last, level compute the actual flow in pixels
        {
          float nx = i-u;
          float ny = j-v;
          //TODO: check if patch drifted too far - take number of pyramid levels in to account for that
          {
            meanflowx += nx;
            meanflowy += ny;
            meancount++;
          }
        }
    }
  }

  /* evaluate flow calculation */
  if (meancount > 0)
  {
    meanflowx /= meancount;
    meanflowy /= meancount;

  *pixel_flow_x = -meanflowx;
  *pixel_flow_y = -meanflowy;
}
else
{
  *pixel_flow_x = 0.0f;
  *pixel_flow_y = 0.0f;
  return 0;
}

/* return quality */
return (uint8_t)(meancount * 255 / (NUM_BLOCKS_KLT*NUM_BLOCKS_KLT));
}

//-----------------------------SAD2
#else

uint32_t zrn_uadd8(uint32_t f1, uint32_t f2)
{
    uint8_t *p1 = (uint8_t*) &f1;
    uint8_t *p2 = (uint8_t*) &f2;
    uint32_t tmp = 0;
    uint8_t *p3 = (uint8_t*) &tmp;

    p3[0] = p1[0] + p2[0];
	p3[1] = p1[1] + p2[1];
	p3[2] = p1[2] + p2[2];
	p3[3] = p1[3] + p2[3];

	return tmp;
}

uint32_t zrn_usada8(uint32_t f1, uint32_t f2, uint32_t f3)
{
	uint8_t *p1 = (uint8_t*)&f1;
	uint8_t *p2 = (uint8_t*)&f2;
	uint32_t tmp = 0;
	tmp = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2]) + abs(p1[3] - p2[3]) + f3;
	return tmp;

}


uint32_t zrn_usad8(uint32_t f1, uint32_t f2)
{
	uint8_t *p1 = (uint8_t*)&f1;
	uint8_t *p2 = (uint8_t*)&f2;
	uint32_t tmp = 0;
	tmp = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2]) + abs(p1[3] - p2[3]);
	return tmp;
}

uint32_t zrn_uhadd8(uint32_t f1, uint32_t f2)
{
	uint8_t *p1 = (uint8_t*)&f1;
	uint8_t *p2 = (uint8_t*)&f2;
	uint32_t tmp = 0;
	uint8_t *p3 = (uint8_t*)&tmp;

	p3[0] = (uint8_t)((uint32_t)(p1[0] + p2[0])) >> 1;
	p3[1] = (uint8_t)((uint32_t)(p1[1] + p2[1])) >> 1;
	p3[2] = (uint8_t)((uint32_t)(p1[2] + p2[2])) >> 1;
	p3[3] = (uint8_t)((uint32_t)(p1[3] + p2[3])) >> 1;

	return tmp;
}

/******
**@brief compute the average pixel gradient of all horizontal and vertical steps
*
**TODO
*
**@param image ...
**@param offX x coordinate of upper left corner of 8x8 pattern in image
*
**@param offY y coordinate of upper left corner of 8X8 pattern in image
*
*******/

static inline uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
    /* calculate position in image buffer */
	uint16_t off = (offY - 2) * row_size + (offX - 2); // we calc only the 4x4 pat
	uint32_t acc;

    /* calc row diff */
	acc = zrn_usad8(*((uint32_t*)&image[off + 0 + 0 * row_size]), *((uint32_t*)&image[off + 0 + 1 * row_size]));
	acc = zrn_usada8(*((uint32_t*)&image[off + 0 + 1 * row_size]), *((uint32_t*)&image[off + 0 + 2 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image[off + 0 + 2 * row_size]), *((uint32_t*)&image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 0 + 1 * row_size] << 16 | image[off + 0 + 2 * row_size] << 8 | image[off + 0 + 3 * row_size];
	uint32_t col2 = (image[off + 1 + 0 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 1 + 2 * row_size] << 8 | image[off + 1 + 3 * row_size];
	uint32_t col3 = (image[off + 2 + 0 * row_size] << 24) | image[off + 2 + 1 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 | image[off + 2 + 3 * row_size];
	uint32_t col4 = (image[off + 3 + 0 * row_size] << 24) | image[off + 3 + 1 * row_size] << 16 | image[off + 3 + 2 * row_size] << 8 | image[off + 3 + 3 * row_size];

	/* calc column diff */
	acc = zrn_usada8(col1, col2, acc);
	acc = zrn_usada8(col2, col3, acc);
	acc = zrn_usada8(col3, col4, acc);
}

static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	* calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	* every iteration is one line of the 8x8 field.
	*
	*  + - + - + - + - + - + - + - + - +
	*  |   |   |   |   |   |   |   |   |
	*  + - + - + - + - + - + - + - + - +
	*
	*
	*/

	for (uint16_t i = 0; i < 8; i++)
	{
		/*
		* first column of 4 pixels:
		*
		*  + - + - + - + - + - + - + - + - +
		*  | x | x | x | x |   |   |   |   |
		*  + - + - + - + - + - + - + - + - +
		*
		* the 8 s values are from following positions for each pixel (X):
		*  + - + - + - +
		*  +   5   7   +
		*  + - + 6 + - +
		*  +   4 X 0   +
		*  + - + 2 + - +
		*  +   3   1   +
		*  + - + - + - +
		*
		*  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		*
		*/

		/* compute average of two pixel values */
		s0 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 1 + (i + 0) * row_size])));
		s1 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 + 1 + (i + 1) * row_size])));
		s2 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 0 + (i + 1) * row_size])));
		s3 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 - 1 + (i + 1) * row_size])));
		s4 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 - 1 + (i + 0) * row_size])));
		s5 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 - 1 + (i - 1) * row_size])));
		s6 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 0 + (i - 1) * row_size])));
		s7 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 0 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 + 1 + (i - 1) * row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (zrn_uhadd8(s0, s1));
		t3 = (zrn_uhadd8(s3, s4));
		t5 = (zrn_uhadd8(s4, s5));
		t7 = (zrn_uhadd8(s7, s0));

		/*
		* finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		*  + - + - + - +
		*  |   |   |   |
		*  + - 5 6 7 - +
		*  |   4 X 0   |
		*  + - 3 2 1 - +
		*  |   |   |   |
		*  + - + - + - +
		*/

		/* fill accumulation vector */
		acc[0] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = zrn_usada8((*((uint32_t*)&image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		* same for second column of 4 pixels:
		*
		*  + - + - + - + - + - + - + - + - +
		*  |   |   |   |   | x | x | x | x |
		*  + - + - + - + - + - + - + - + - +
		*
		*/

		s0 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 5 + (i + 0) * row_size])));
		s1 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 + 5 + (i + 1) * row_size])));
		s2 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 4 + (i + 1) * row_size])));
		s3 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 1) * row_size]), *((uint32_t*)&image2[off2 + 3 + (i + 1) * row_size])));
		s4 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 3 + (i + 0) * row_size])));
		s5 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 + 3 + (i - 1) * row_size])));
		s6 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i + 0) * row_size]), *((uint32_t*)&image2[off2 + 4 + (i - 1) * row_size])));
		s7 = (zrn_uhadd8(*((uint32_t*)&image2[off2 + 4 + (i - 1) * row_size]), *((uint32_t*)&image2[off2 + 5 + (i - 1) * row_size])));

		t1 = (zrn_uhadd8(s0, s1));
		t3 = (zrn_uhadd8(s3, s4));
		t5 = (zrn_uhadd8(s4, s5));
		t7 = (zrn_uhadd8(s7, s0));

		acc[0] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = zrn_usada8((*((uint32_t*)&image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}

static inline uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	acc = zrn_usad8(*((uint32_t*)&image1[off1 + 0 + 0 * row_size]), *((uint32_t*)&image2[off2 + 0 + 0 * row_size]));
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 0 * row_size]), *((uint32_t*)&image2[off2 + 4 + 0 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 1 * row_size]), *((uint32_t*)&image2[off2 + 0 + 1 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 1 * row_size]), *((uint32_t*)&image2[off2 + 4 + 1 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 2 * row_size]), *((uint32_t*)&image2[off2 + 0 + 2 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 2 * row_size]), *((uint32_t*)&image2[off2 + 4 + 2 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 3 * row_size]), *((uint32_t*)&image2[off2 + 0 + 3 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 3 * row_size]), *((uint32_t*)&image2[off2 + 4 + 3 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 4 * row_size]), *((uint32_t*)&image2[off2 + 0 + 4 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 4 * row_size]), *((uint32_t*)&image2[off2 + 4 + 4 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 5 * row_size]), *((uint32_t*)&image2[off2 + 0 + 5 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 5 * row_size]), *((uint32_t*)&image2[off2 + 4 + 5 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 6 * row_size]), *((uint32_t*)&image2[off2 + 0 + 6 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 6 * row_size]), *((uint32_t*)&image2[off2 + 4 + 6 * row_size]), acc);

	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 0 + 7 * row_size]), *((uint32_t*)&image2[off2 + 0 + 7 * row_size]), acc);
	acc = zrn_usada8(*((uint32_t*)&image1[off1 + 4 + 7 * row_size]), *((uint32_t*)&image2[off2 + 4 + 7 * row_size]), acc);

	return acc;
}

/****************************** compensate rotation ***********************************************/
float com_rotation(int8_t sumx, int8_t sumy, uint16_t i, uint16_t j)
{
	float dis = (i-32)*(i-32) + (j-32)*(j-32);
	float rotate = sumx*(j - 32) - sumy * (i - 32);
	return rotate / dis;
}

u8 first_flag_low_pass = 1;
float flow_x_last = 0, flow_y_last = 0;
#define LOW_PASS (float)80
uint8_t compute_flow1(uint8_t *image1, uint8_t *image2, float *pixel_flow_x, float *pixel_flow_y)
{
	/* constants */
	const int16_t winmin = -SEARCH_SIZE1;
	const int16_t winmax = SEARCH_SIZE1;

	/* variables */
    	uint16_t pixLo = SEARCH_SIZE1 + 1;
    	uint16_t pixHi_x = IMAGE_COLS - (SEARCH_SIZE1 + 1) - 2*SEARCH_SIZE1;
    	uint16_t pixHi_y = IMAGE_ROWS - (SEARCH_SIZE1 + 1) - 2*SEARCH_SIZE1;
   	uint16_t pixStep_x = (pixHi_x - pixLo) / (NUM_BLOCKS1-1);
   	uint16_t pixStep_y = (pixHi_y - pixLo) / (NUM_BLOCKS1-1);
	uint16_t i, j;
	uint32_t acc[8]; // subpixels

	int8_t  dirsx[64]; // shift directions in x
	int8_t  dirsy[64]; // shift directions in y
	uint8_t  subdirs[64]; // shift directions of best subpixels
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;


	/* iterate over all patterns
	 */
	for (j = pixLo; j < pixHi_y; j += pixStep_y)
	{
		for (i = pixLo; i < pixHi_x; i += pixStep_x)
		{
			/* test pixel if it is suitable for flow tracking */
			uint32_t diff = compute_diff(image1, i, j, IMAGE_COLS);
			if (diff < CORNER_THRE)
			{
				continue;
			}

			uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
			int8_t sumx = 0;
			int8_t sumy = 0;
			int8_t ii, jj;

			for (jj = winmin; jj <= winmax; jj++)
			{
				for (ii = winmin; ii <= winmax; ii++)
				{
					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, IMAGE_COLS);
					if (temp_dist < dist)
					{
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			if (dist < SAD_THRE)
			{
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, IMAGE_COLS);
				uint32_t mindist = dist; // best SAD until now
				uint8_t mindir = 8; // direction 8 for no direction
				for(uint8_t k = 0; k < 8; k++)
				{
					if (acc[k] < mindist)
					{
						// SAD becomes better in direction k
						mindist = acc[k];
						mindir = k;
					}
				}
				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
				subdirs[meancount] = mindir;
				meancount++;

				image1[(j+sumy) * IMAGE_COLS +sumx+ i] = 0;
				image1[(j+sumy) * IMAGE_COLS +sumx+ i+1] = 0;
				image1[(j+sumy+1) * IMAGE_COLS +sumx+ i] = 0;
				image1[(j+sumy+1) * IMAGE_COLS +sumx+ i+1] = 0;
			}
			image1[(j) * IMAGE_COLS + i] = 255;
			image1[(j) * IMAGE_COLS + i+1] = 255;
			image1[(j+1) * IMAGE_COLS + i] = 255;
			image1[(j+1) * IMAGE_COLS + i+1] = 255;
		}
	}

	//printf("meancount: %d\n",meancount );
	/* create flow image if needed (image1 is not needed anymore)
	 * -> can be used for debugging purpose
	 */
/*	for (j = pixLo; j < pixHi; j += pixStep)
	{
		for (i = pixLo; i < pixHi; i += pixStep)
		{
			uint32_t diff = compute_diff(image1, i, j, FRAME_SIZE);
			if (diff > CORNER_THRE)
			{
				image1[(j) * FRAME_SIZE + i] = 255;
				image1[(j) * FRAME_SIZE + i+1] = 255;
				image1[(j+1) * FRAME_SIZE + i] = 255;
				image1[(j+1) * FRAME_SIZE + i+1] = 255;
			}
		}
	}
*/

	/* evaluate flow calculation */
	if (meancount > 10)
	{
		meanflowx /= meancount;
		meanflowy /= meancount;

		/* use average of accepted flow values */
		uint32_t meancount_x = 0;
		uint32_t meancount_y = 0;

		for (uint8_t h = 0; h < meancount; h++)
		{
			float subdirx = 0.0f;
			if (subdirs[h] == 0 || subdirs[h] == 1 || subdirs[h] == 7) subdirx = 0.5f;
			if (subdirs[h] == 3 || subdirs[h] == 4 || subdirs[h] == 5) subdirx = -0.5f;
			histflowx += (float)dirsx[h] + subdirx;
			meancount_x++;

			float subdiry = 0.0f;
			if (subdirs[h] == 5 || subdirs[h] == 6 || subdirs[h] == 7) subdiry = -0.5f;
			if (subdirs[h] == 1 || subdirs[h] == 2 || subdirs[h] == 3) subdiry = 0.5f;
			histflowy += (float)dirsy[h] + subdiry;
			meancount_y++;
		}

		histflowx /= meancount_x;
		histflowy /= meancount_y;

		*pixel_flow_x = histflowx;
		*pixel_flow_y = histflowy;

		/******************************* compensate rotation here ***********************/



		/********************************************************************************/


        //  low_pass stuff //
		if(first_flag_low_pass == 1)
		{
			flow_x_last = *pixel_flow_x;
			flow_y_last = *pixel_flow_y;
			first_flag_low_pass = 0;
		}
		else
		{
			*pixel_flow_x = *pixel_flow_x * LOW_PASS / 100. + (1-LOW_PASS / 100.)* flow_x_last;
			*pixel_flow_y = *pixel_flow_y * LOW_PASS / 100. + (1-LOW_PASS / 100.)* flow_y_last;
			flow_x_last = *pixel_flow_x;
			flow_y_last = *pixel_flow_y;
		}
	}
	else
	{
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}

	uint8_t qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS1*NUM_BLOCKS1));

	return qual;
}

#endif

//-----------------
u8 flow_fps;
#include "optic_flow_ardrone.h"
double pixel_flow_x = 0.0f;
double pixel_flow_y = 0.0f;
double pixel_flow_x_klt = 0.0f;
double pixel_flow_y_klt = 0.0f;
double pixel_flow_x_sad = 0.0f;
double pixel_flow_y_sad = 0.0f;
double pixel_flow_x_sadt = 0.0f;
double pixel_flow_y_sadt = 0.0f;
int px_x,px_y;
int new_x,  new_y;
u8 en_klt=1;
u8 qual[2];
float k_sad=0.76;

u8 flow_task(uint8_t * current_image,uint8_t * previous_image ,uint8_t * current_imaget,uint8_t * previous_imaget ,float get_time_between_images)
{

	static float pos_acc_flow[2],pos_flow[2],spd[2];	
	static float apr[3];
	static u8 init,led;
	static float hc_acc_i,wz_speed_0,wz_speed_1,wz_speed_old;
	float pixel_flow_x_t2,pixel_flow_y_t2;
	float f_com[2];
	float tempx,tempy,tempx1,tempy1;
  //if(!en_klt)
#if USE_SAD	
	#if SAD2
	qual[0] = compute_flow1(previous_image, current_image, &tempx, &tempy );
	#else
	qual[0] = compute_flow(previous_image, current_image, 0, 0,0, &tempx, &tempy , get_time_between_images);
	#endif
	pixel_flow_x_sad=firstOrderFilter(	tempx ,&firstOrderFilters[FLOW_LOWPASS_X],get_time_between_images);
	pixel_flow_y_sad=firstOrderFilter(	tempy ,&firstOrderFilters[FLOW_LOWPASS_Y],get_time_between_images);
	#if SAD2
	qual[0] = compute_flow1(previous_imaget, current_imaget, &tempx, &tempy );
	#else
	qual[0] = compute_flow(previous_imaget, current_imaget, 0, 0,0, &tempx1, &tempy1 , get_time_between_images);
	#endif
	pixel_flow_x_sadt=firstOrderFilter(	tempx1 ,&firstOrderFilters[ACC_LOWPASS_X],get_time_between_images);
	pixel_flow_y_sadt=firstOrderFilter(	tempy1 ,&firstOrderFilters[ACC_LOWPASS_Y],get_time_between_images);
	pixel_flow_x=pixel_flow_x_sad*0.5+pixel_flow_y_sadt*0.5;
	pixel_flow_y=pixel_flow_y_sad*0.5+pixel_flow_x_sadt*0.5;
	//int status;
	//opticFlowLK( current_image,previous_image, &px_x,&px_y, 64, 64,64, & new_x,  &new_y,  &status,HALF_PATCH_SIZE, 5);

#else
	#if !KLT2
	qual[1] = compute_klt(previous_image, current_image, 0, 0,0, &tempx1, &tempy1 );
	#else
	qual[1] =check_for_frame(previous_image, current_image, 0, 0,0, &tempx1, &tempy1 );
	#endif
	pixel_flow_x_klt=1.45*1*firstOrderFilter(	-tempx1 ,&firstOrderFilters[ACC_LOWPASS_X],get_time_between_images);
	pixel_flow_y_klt=1.45*1*firstOrderFilter(	-tempy1 ,&firstOrderFilters[ACC_LOWPASS_Y],get_time_between_images);
#endif	
	
//	pixel_flow_x=(float)qual[0]/255*k_sad*pixel_flow_x_sad+(1-k_sad*(float)qual[0]/255)*pixel_flow_x_klt;
//	pixel_flow_y=(float)qual[0]/255*k_sad*pixel_flow_y_sad+(1-k_sad*(float)qual[0]/255)*pixel_flow_y_klt;
return	qual[0]/2+qual[1]/2;
}

#if KLT2
//----------------------------------------KLT 2222---------------------------
static flow_klt_image flow_klt_images[2] __attribute__((section(".ccm")));


void klt_preprocess_image(uint8_t *image, flow_klt_image *klt_image) {
	uint16_t i, j;

	klt_image->image = image;
	
	/*
	 * compute image pyramid for current frame
	 * there is 188*120 bytes per buffer, we are only using 64*64 per buffer,
	 * so just add the pyramid levels after the image
	 */
	//first compute the offsets in the memory for the pyramid levels
	uint8_t *lvl_base[PYR_LVLS];
	uint16_t frame_size = (uint16_t)(64+0.5);
	uint16_t s = frame_size / 2;
	uint16_t off = 0;
	lvl_base[0] = image;
	for (int l = 1; l < PYR_LVLS; l++)
	{
		lvl_base[l] = klt_image->preprocessed + off;
		off += s*s;
		s /= 2;
	}

	//then subsample the images consecutively, no blurring is done before the subsampling (if someone volunteers, please go ahead...)
	for (int l = 1; l < PYR_LVLS; l++)
	{
		uint16_t src_size = frame_size >> (l-1);
		uint16_t tar_size = frame_size >> l;
		uint8_t *source = lvl_base[l-1]; //pointer to the beginning of the previous level
		uint8_t *target = lvl_base[l];   //pointer to the beginning of the current level
		for (j = 0; j < tar_size; j++) {
			for (i = 0; i < tar_size; i+=2)
			{
				//subsample the image by 2, use the halving-add instruction to do so
				uint32_t l1 = (__UHADD8(*((uint32_t*) &source[(j*2+0)*src_size + i*2]), *((uint32_t*) &source[(j*2+0)*src_size + i*2+1])));
				uint32_t l2 = (__UHADD8(*((uint32_t*) &source[(j*2+1)*src_size + i*2]), *((uint32_t*) &source[(j*2+1)*src_size + i*2+1])));
				uint32_t r = __UHADD8(l1, l2);

				//the first and the third byte are the values we want to have
				target[j*tar_size + i+0] = (uint8_t) r;
				target[j*tar_size + i+1] = (uint8_t) (r>>16);
			}
		}
	}
}

float get_flow_klt_capability() {
	uint16_t topPyrStep = 1 << (PYR_LVLS - 1);
  return topPyrStep * 2;
}

uint16_t compute_klt1(flow_klt_image *image1, flow_klt_image *image2, float x_rate, float y_rate, float z_rate,
					 flow_raw_result *out, uint16_t max_out)
{
	/* variables */
	uint16_t i, j;

	float chi_sum = 0.0f;
	uint8_t chicount = 0;

	uint16_t max_iters = 5;

	/*
	 * compute image pyramid for current frame
	 * there is 188*120 bytes per buffer, we are only using 64*64 per buffer,
	 * so just add the pyramid levels after the image
	 */
	//first compute the offsets in the memory for the pyramid levels
	uint8_t *lvl_base1[PYR_LVLS];
	uint8_t *lvl_base2[PYR_LVLS];
	uint16_t frame_size = (uint16_t)(64);
	uint16_t s = frame_size / 2;
	uint16_t off = 0;
	lvl_base1[0] = image1->image;
	lvl_base2[0] = image2->image;
	for (int l = 1; l < PYR_LVLS; l++)
	{
		lvl_base1[l] = image1->preprocessed + off;
		lvl_base2[l] = image2->preprocessed + off;
		off += s*s;
		s /= 2;
	}

	//need to store the flow values between pyramid level changes
	float us[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];
	float vs[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];
	uint16_t is[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];
	uint16_t js[NUM_BLOCKS_KLT*NUM_BLOCKS_KLT];

	//initialize flow values with the pixel value of the previous image
	uint16_t topPyrStep = 1 << (PYR_LVLS - 1);

    /* 
     * if the gyro change (x_rate & y_rate) is more than the maximum pixel
     * difference that can be detected between two frames (depends on PYR_LVLS)
     * we can't calculate the flow. So return empty flow.
     */
    if(fabsf(x_rate) > (float)(2 * topPyrStep) || fabsf(y_rate) > (float)(2 * topPyrStep)){
        return 0;
    }

	uint16_t pixStep = frame_size / (NUM_BLOCKS_KLT + 1);
	uint16_t pixLo = pixStep;
	/* align with topPyrStep */
	pixStep = ((uint16_t)(pixStep                 )) & ~((uint16_t)(topPyrStep - 1));	// round down
	pixLo   = ((uint16_t)(pixLo + (topPyrStep - 1))) & ~((uint16_t)(topPyrStep - 1));	// round up
	//uint16_t pixHi = pixLo + pixStep * (NUM_BLOCK_KLT - 1);

	j = pixLo;
	for (int y = 0; y < NUM_BLOCKS_KLT; y++, j += pixStep)
	{
		i = pixLo;
		for (int x = 0; x < NUM_BLOCKS_KLT; x++, i += pixStep)
		{
			uint16_t idx = y*NUM_BLOCKS_KLT+x;
			if (0){//[PARAM_KLT_GYRO_ASSIST]
				/* use the gyro measurement to guess the initial position in the new image */
				us[idx] = i + x_rate; //position in new image at level 0
				vs[idx] = j + y_rate;
				if ((int16_t)us[idx] < HALF_PATCH_SIZE) 				 us[idx] = HALF_PATCH_SIZE;
				if ((int16_t)us[idx] > frame_size - HALF_PATCH_SIZE - 1) us[idx] = frame_size - HALF_PATCH_SIZE - 1;
				if ((int16_t)vs[idx] < HALF_PATCH_SIZE) 				 vs[idx] = HALF_PATCH_SIZE;
				if ((int16_t)vs[idx] > frame_size - HALF_PATCH_SIZE - 1) vs[idx] = frame_size - HALF_PATCH_SIZE - 1;
			} else {
				us[idx] = i; //position in new image at level 0
				vs[idx] = j;
			}
			is[idx] = i;	//position in previous image at level 0
			js[idx] = j;
			/* init output vector */
			if (idx < max_out) {
				out[idx].x = 0;
				out[idx].y = 0;
				out[idx].quality = 0;
				out[idx].at_x = i;
				out[idx].at_y = j;
			}
		}
	}

	//for all pyramid levels, start from the smallest level
	for (int l = PYR_LVLS-1; l >= 0; l--)
	{
		//iterate over all patterns
		for (int k = 0; k < NUM_BLOCKS_KLT*NUM_BLOCKS_KLT; k++)
		{
			i = is[k] >> l;  //reference pixel for the current level
			j = js[k] >> l;

			uint16_t iwidth = frame_size >> l;
			uint8_t *base1 = lvl_base1[l] + j * iwidth + i;

			float JTJ[4];   //the 2x2 Hessian
			JTJ[0] = 0;
			JTJ[1] = 0;
			JTJ[2] = 0;
			JTJ[3] = 0;
			int c = 0;

			//compute jacobians and the hessian for the patch at the current location
			uint8_t min_val = 255;
			uint8_t max_val = 0;
			for (int8_t jj = -HALF_PATCH_SIZE; jj <= HALF_PATCH_SIZE; jj++)
			{
				uint8_t *left = base1 + jj*iwidth;
				for (int8_t ii = -HALF_PATCH_SIZE; ii <= HALF_PATCH_SIZE; ii++)
				{
					uint8_t val = left[ii];
					if (val > max_val) max_val = val;
					if (val < min_val) min_val = val;
					const float jx = ((uint16_t)left[ii+1] - (uint16_t)left[ii-1]) * 0.5f;
					const float jy = ((uint16_t)left[ii+iwidth] - (uint16_t)left[ii-iwidth]) * 0.5f;
					Jx[c] = jx;
					Jy[c] = jy;
					JTJ[0] += jx*jx;
					JTJ[1] += jx*jy;
					JTJ[2] += jx*jy;
					JTJ[3] += jy*jy;
					c++;
				}
			}

			//compute inverse of hessian
			float det = (JTJ[0]*JTJ[3]-JTJ[1]*JTJ[2]);
			float dyn_range = (float)(max_val - min_val) + 1;
			float trace = (JTJ[0] + JTJ[3]);
			float M_c = det - 0.06 * trace * trace;//PARAM_ALGORITHM_CORNER_KAPPA
			if (fabsf(det) > 90 * dyn_range && M_c > 0.0f)//PARAM_KLT_DET_VALUE_MIN
			{
				float detinv = 1.f / det;
				float JTJinv[4];
				JTJinv[0] = detinv * JTJ[3];
				JTJinv[1] = detinv * -JTJ[1];
				JTJinv[2] = detinv * -JTJ[2];
				JTJinv[3] = detinv * JTJ[0];

				// us and vs store the sample position in level 0 pixel coordinates
				float u = (us[k] / (1<<l));
				float v = (vs[k] / (1<<l));

				float chi_sq_previous = 0.f;

				u8 result_good = 1;

				//Now do some Gauss-Newton iterations for flow
				for (int iters = 0; iters < max_iters; iters++)
				{
					float JTe_x = 0;  //accumulators for Jac transposed times error
					float JTe_y = 0;

					uint8_t *base2 = lvl_base2[l] + (uint16_t)v * iwidth + (uint16_t)u;

					//extract bilinearly filtered pixel values for the current location in image2
					float dX = u - floorf(u);
					float dY = v - floorf(v);
					float fMixTL = (1.f - dX) * (1.f - dY);
					float fMixTR = (dX) * (1.f - dY);
					float fMixBL = (1.f - dX) * (dY);
					float fMixBR = (dX) * (dY);

					float chi_sq = 0.f;
					c = 0;
					for (int8_t jj = -HALF_PATCH_SIZE; jj <= HALF_PATCH_SIZE; jj++)
					{
						uint8_t *left1 = base1 + jj*iwidth;
						uint8_t *left2 = base2 + jj*iwidth;

						for (int8_t ii = -HALF_PATCH_SIZE; ii <= HALF_PATCH_SIZE; ii++)
						{
							float fPixel = fMixTL * left2[ii] + fMixTR * left2[ii+1] + fMixBL * left2[ii+iwidth] + fMixBR * left2[ii+iwidth+1];
							float fDiff = fPixel - left1[ii];
							JTe_x += fDiff * Jx[c];
							JTe_y += fDiff * Jy[c];
							chi_sq += fDiff*fDiff;
							c++;
						}
					}

					//only update if the error got smaller
					if (iters == 0 || chi_sq_previous > chi_sq)
					{
						//compute update and shift current position accordingly
						float updx = JTJinv[0]*JTe_x + JTJinv[1]*JTe_y;
						float updy = JTJinv[2]*JTe_x + JTJinv[3]*JTe_y;
						float new_u = u-updx;
						float new_v = v-updy;

						//check if we drifted outside the image
						if (((int16_t)new_u < HALF_PATCH_SIZE) || (int16_t)new_u > (iwidth-HALF_PATCH_SIZE-1) || ((int16_t)new_v < HALF_PATCH_SIZE) || (int16_t)new_v > (iwidth-HALF_PATCH_SIZE-1))
						{
							result_good = 0;
							break;
						}
						else
						{
							u = new_u;
							v = new_v;
						}
					}
					else
					{
						chi_sum += chi_sq_previous;
						chicount++;
						break;
					}
					chi_sq_previous = chi_sq;
				}
				if (l > 0)
				{
					// TODO: evaluate recording failure at each level to calculate a final quality value
					us[k] = u * (1<<l);
					vs[k] = v * (1<<l);
				}
				else  //for the last level compute the actual flow in pixels
				{
					if (result_good && k < max_out) {
						if (0) {
							/* compute flow and compensate gyro */
							out[k].x = u - i - x_rate;
							out[k].y = v - j - y_rate;
						} else {
							out[k].x = u - i;
							out[k].y = v - j;
						}
						if (fabsf(out[k].x) < (float)(2 * topPyrStep) &&
						    fabsf(out[k].y) < (float)(2 * topPyrStep)) {
							out[k].quality = 1.0f;
						} else {
							/* drifted too far */
							out[k].x = 0;
							out[k].y = 0;
							out[k].quality = 0;
						}
					}
				}
			}
		}
	}
	return NUM_BLOCKS_KLT * NUM_BLOCKS_KLT < max_out ? NUM_BLOCKS_KLT * NUM_BLOCKS_KLT : max_out;
}

struct flow_res_dim_value {
	float value;
	uint16_t idx;
};

static int flow_res_dim_value_compare(const void* elem1, const void* elem2)
{
	float v1 = ((const struct flow_res_dim_value *)elem1)->value;
	float v2 = ((const struct flow_res_dim_value *)elem2)->value;
    if(v1 < v2)
        return -1;
    return v1 > v2;
}
uint16_t flow_rslt_count = 0;
uint8_t flow_extract_result(flow_raw_result *in, uint16_t result_count, float *px_flow_x, float *px_flow_y,
				float accuracy_p, float accuracy_px)
{
	/* extract all valid results: */
	struct flow_res_dim_value xvalues[30];
	struct flow_res_dim_value yvalues[30];
	uint16_t valid_c = 0;
	for (int i = 0; i < result_count; i++) {
		if (in[i].quality > 0) {
			xvalues[valid_c].value = in[i].x;
			xvalues[valid_c].idx   = i;
			yvalues[valid_c].value = in[i].y;
			yvalues[valid_c].idx   = i;
			valid_c++;
		}
	}
	if (valid_c < (result_count + 2) / 3 || valid_c < 3) {
		*px_flow_x = 0;
		*px_flow_y = 0;
		return 0;
	}
	struct flow_res_dim_value *axes[2] = {xvalues, yvalues};
	float *output[2] = {px_flow_x, px_flow_y};
	float max_spread_val[2] = {0};
	float spread_val[2] = {0};
	uint16_t total_avg_c = 0;
	for (int i = 0; i < 2; ++i) {
		struct flow_res_dim_value *axis = axes[i];
		/* sort them */
		qsort(axis, valid_c, sizeof(struct flow_res_dim_value), flow_res_dim_value_compare);
		spread_val[i] = axis[valid_c * 3 / 4].value - axis[valid_c / 4].value;
		/* start with one element */
		uint16_t s = valid_c / 2;
		uint16_t e = valid_c / 2 + 1;
		uint16_t s_res;
		uint16_t e_res;
		float avg_sum = axis[s].value;
		uint16_t avg_c = 1;
		float avg;
		while (1) {
			s_res = s;
			e_res = e;
			/* calculate average and maximum spread to throw away outliers */
			avg = avg_sum / avg_c;
			float max_spread = fabsf(avg) * accuracy_p;
			max_spread = accuracy_px * accuracy_px / (accuracy_px + max_spread) + max_spread;
			max_spread_val[i] = max_spread;
			/* decide on which side to add new data-point (to remain centered in the sorted set) */
			if (s > valid_c - e && s > 0) {
				s--;
				avg_sum += axis[s].value;
			} else if (e < valid_c) {
				e++;
				avg_sum += axis[e - 1].value;
			} else {
				break;
			}
			avg_c++;
			/* check maximum spread */
			if (axis[e - 1].value - axis[s].value > max_spread) break;
			/* its good. continue .. */
		}
		if (e_res - s_res > 1) {
			/* we have a result */
			*output[i] = avg;
			total_avg_c += e_res - s_res;
		} else {
			*px_flow_x = 0;
			*px_flow_y = 0;
			return 0;
		}
	}

	total_avg_c = total_avg_c / 2;
	return (total_avg_c * 255) / result_count;
}

uint8_t check_for_frame(uint8_t *image1,uint8_t *image2,float x_rate,float y_rate,float z_rate,float *px,float *py) {
	/* new gyroscope data */		
		u8 use_klt = 1;
		
		flow_klt_image *klt_images[2] ;
		
			/* make sure that the new images get the correct treatment */
			/* this algorithm will still work if both images are new */
			int i;
			u8 used_klt_image[2] = {0, 0};
			
			for (i = 0; i < 2; ++i) {

					// the image has the preprocessing already applied.
					if (use_klt) {
						int j;
						/* find the klt image that matches: */
						for (j = 0; j < 2; ++j) {
							if (0){//flow_klt_images[j].meta == frames[i]->frame_number) {
								used_klt_image[j] = 1;
								klt_images[i] = &flow_klt_images[j];
							}
						}
					}
				
			}
			
			if (use_klt) {
				/* only for KLT: */
				/* preprocess the images if they are not yet preprocessed */
				for (i = 0; i < 2; ++i) {
//					if (klt_images[i] == 0) {
//						// need processing. find unused KLT image:
//						int j;
//						for (j = 0; j < 2; ++j) {
//							if (!used_klt_image[j]) {
//								used_klt_image[j] = 1;
//								klt_images[i] = &flow_klt_images[j];
//								break;
//							}
//						}
						if(i==1)
						klt_preprocess_image(image1, klt_images[i]);
						else
						klt_preprocess_image(image2, klt_images[i]);
					}
			//	}
			}
		
		
//		float frame_dt   = calculate_time_delta_us(frames[0]->timestamp, frames[1]->timestamp)           * 0.000001f;
//		float dropped_dt = calculate_time_delta_us(frames[1]->timestamp, last_processed_frame_timestamp) * 0.000001f;
//		last_processed_frame_timestamp = frames[0]->timestamp;

		/* calculate focal_length in pixel */
		//const float focal_length_px = 15;//(global_data.param[PARAM_FOCAL_LENGTH_MM]) / 
									 // ((float)frames[0]->param.p.binning * 0.006f);	// pixel-size: 6um
	  float focal_length_px = (16) / (4.0f * 6.0f) * 1000.0f;
		/* extract the raw flow from the images: */
		flow_raw_result flow_rslt[32];

		/* make sure both images are taken with same binning mode: */

	  flow_rslt_count =  compute_klt1(klt_images[1], klt_images[0],0, 0, 0, flow_rslt, sizeof(flow_rslt) / sizeof(flow_rslt[0]));

		/* determine velocity capability: */
		float flow_mv_cap;
		flow_mv_cap = get_flow_klt_capability();


		/* calculate flow value from the raw results */
		float pixel_flow_x;
		float pixel_flow_y;
		float outlier_threshold = 0.015;//global_data.param[PARAM_ALGORITHM_OUTLIER_THR_RATIO];
		float min_outlier_threshold = 0;

		min_outlier_threshold = 0.2;//global_data.param[PARAM_ALGORITHM_OUTLIER_THR_KLT];
		
		return  flow_extract_result(flow_rslt, flow_rslt_count, px, py, outlier_threshold,  min_outlier_threshold);

}
#endif