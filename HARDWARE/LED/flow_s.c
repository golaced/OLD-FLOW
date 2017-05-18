#include "flow.h"
#define LIMIT_FLOW( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
float rate_threshold=1.62;
float flow_per_out[4];
float yaw_comp[2];
float flow_module_offset_x,flow_module_offset_y;
void flow_pertreatment_oldx( FLOW_RAD *flow_in ,float flow_height){
float flow_gyrospeed[3];	
	  flow_gyrospeed[0] = (float)flow_in->integrated_xgyro ;
		flow_gyrospeed[1] = (float)flow_in->integrated_ygyro ;
		flow_gyrospeed[2] = (float)flow_in->integrated_zgyro ;
	  static  unsigned char n_flow;
	  static float gyro_offset_filtered[3],att_gyrospeed_filtered[3],flow_gyrospeed_filtered[3];
		float flow_ang[2];

		if (fabs(flow_gyrospeed[0]) < rate_threshold) {  
		flow_ang[0] = flow_in->integrated_x ;//for now the flow has to be scaled (to small)  
		}  
		else {  
		//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)  
		flow_ang[0] = ((flow_in->integrated_x - LIMIT_FLOW(flow_in->integrated_xgyro,-fabs(flow_in->integrated_x),fabs(flow_in->integrated_x)))) ;//for now the flow has to be scaled (to small)  
		}  
		if (fabs(flow_gyrospeed[1]) < rate_threshold) {  
		flow_ang[1] = flow_in->integrated_y;//for now the flow has to be scaled (to small)  
		}  
		else {  
		flow_ang[1] = ((flow_in->integrated_y- LIMIT_FLOW(flow_in->integrated_ygyro,-fabs(flow_in->integrated_y),fabs(flow_in->integrated_y))));//for now the flow has to be scaled (to small)  
		}  
	  
		yaw_comp[0] = - flow_module_offset_y * (flow_gyrospeed[2]);  
		yaw_comp[1] = flow_module_offset_x * (flow_gyrospeed[2]);  
		/* flow measurements vector */  


		if (fabs(flow_gyrospeed[2]) < rate_threshold) {
    flow_per_out[2]=(flow_ang[1]);
		flow_per_out[3]=(flow_ang[0]);
		} else {
		flow_per_out[2]=(flow_ang[1]) - yaw_comp[1] ;//1
		flow_per_out[3]=(flow_ang[0]) - yaw_comp[0] ;//0
		}

}