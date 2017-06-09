#ifndef __FLOW_H
#define __FLOW_H	 
#include "stm32f4xx.h" 
#include "math.h" 
#define USE_30FPS 1

u8 RGB_GRAY(u32 RGB,u8 SEL);
u8 RGB_GRAY_16(u16 RGB);
u16 GRAY_RGB(u8 gray);
unsigned short read_xy_rgb(u32 *jpeg_buf_i, unsigned int x,unsigned int y,int W);
u8 flow_task(uint8_t * current_image,uint8_t * previous_image ,uint8_t * current_imaget,uint8_t * previous_imaget ,float get_time_between_images);
extern double pixel_flow_x,pixel_flow_y;
extern double pixel_flow_x_klt,pixel_flow_y_klt;
extern double pixel_flow_x_sad,pixel_flow_y_sad;
extern double pixel_flow_x_sadt,pixel_flow_y_sadt;
extern u8 en_hist_filter;
extern u8 en_gro_filter;
extern u8 en_klt;
typedef struct
{
 uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
 uint32_t integration_time_us; ///< Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 float integrated_x; ///< Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 float integrated_y; ///< Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 float h_x_pix;
 float h_y_pix;
 float integrated_xgyro; ///< RH rotation around X axis (rad)
 float integrated_ygyro; ///< RH rotation around Y axis (rad)
 float integrated_zgyro; ///< RH rotation around Z axis (rad)
	
 float x_kf[3];
 float y_kf[3];	
 uint32_t time_delta_distance_us; ///< Time in microseconds since the distance was sampled.
 float distance; ///< Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
 int16_t temperature; ///< Temperature * 100 in centi-degrees Celsius
 uint8_t sensor_id; ///< Sensor ID
 uint8_t quality; ///< Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}FLOW_RAD;

extern FLOW_RAD flow;

extern float flow_per_out[4];//光流最终输出为数组中 [2] [3]
//----------参数---------
extern float k_flow_devide;//光流输出增益  按实际波形和运动情况进行调节
extern float flow_module_offset_y,flow_module_offset_x;//光流安装偏移/m
void flow_pertreatment_oldx( FLOW_RAD *flow_in ,float flow_height);//光流数据预处理



typedef struct _flow_raw_result {
	float x;		///< The flow in x direction
	float y;		///< The flow in y direction
	float quality;	///< The quality of this result. 0 = bad
	uint8_t at_x;	///< The mid-position of the patch that was used to calculate the flow.
	uint8_t at_y;	///< The mid-position of the patch that was used to calculate the flow.
} flow_raw_result;

typedef struct _flow_klt_image {
	uint8_t *image;
	uint8_t preprocessed[(64 * 64) / 2];
	uint32_t meta;
} flow_klt_image;

uint8_t check_for_frame(uint8_t *image1,uint8_t *image2,float x_rate,float y_rate,float z_rate,float *px,float *py) ;
#endif











