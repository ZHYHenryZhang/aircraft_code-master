#include "include.h"
#include "height_ctrl.h"
#include "mymath.h"
#include "ultrasonic.h"  
#include "mpu6050.h"

#define ULTRA_MAX_HEIGHT 1000  //最大高度为1000mm

_st_height_pid ultra_pid;
_st_height_pid_v ultra_ctrl;

float exp_height; //理想期望高度
float height_gap=500;  //调整的范围-500~+500
float ultra_ctrl_out;

void Ultra_PID_Init()
{
	ultra_pid.kp = 1.0*pid_setup.groups.hc_height.kp;
	ultra_pid.kd = 1.0*pid_setup.groups.hc_height.kd;
	ultra_pid.ki = 1.0*pid_setup.groups.hc_height.ki;
}

void IncrementalPID(_st_height_pid *ultra_pid,float target,float measure)
{ 
  ultra_pid->error=target-measure;
  ultra_pid->output=ultra_pid->kp*(ultra_pid->error-ultra_pid->preerror)+ultra_pid->ki*ultra_pid->error+ultra_pid->kd*(ultra_pid->error-2*ultra_pid->preerror+ultra_pid->prepreerror);
  ultra_pid->preerror=ultra_pid->error;
  ultra_pid->prepreerror=ultra_pid->preerror;
}

void Ultrasonic_Ctrl(float T,float thr)
{
	 if(exp_height>ULTRA_MAX_HEIGHT)
		 exp_height=ULTRA_MAX_HEIGHT;
	 
	 
   if(ABS(exp_height-ultra_distance)>20)
	 {
		 IncrementalPID(&ultra_pid,exp_height,ultra_distance);
		 ultra_pid.output=LIMIT(ultra_pid.output,-50,200);
		 ultra_ctrl_out=thr+ultra_pid.output;
	 } 
}

/*void My_Height_Ctrl(float T,float thr)
{	
	static float wz_speed_t;
	static u8 height_ctrl_start_f;
	static u16 hc_start_delay;
	static u8 hs_ctrl_cnt;
	
	switch( height_ctrl_start_f )
	{
		case 0:
	
		if( mpu6050.Acc.z > 4000 )
		{
			height_ctrl_start_f = 1;
		}	
		else if( ++hc_start_delay > 500 )
		{
			height_ctrl_start_f = 1;
		}
		
		break;
		
		case 1:
		
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *( (reference_v.z *mpu6050.Acc.z + reference_v.x *mpu6050.Acc.x + reference_v.y *mpu6050.Acc.y - 4096 ) - wz_acc );
		
		wz_speed_t += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *(0.4f*(thr-500) - wz_speed_t);
				
		if( height_ctrl_mode == 2)
		{
			hs_ctrl_cnt++;
			hs_ctrl_cnt = hs_ctrl_cnt%10;
			if(hs_ctrl_cnt == 0)
			{
				height_speed_ctrl(0.02f,thr,0.4f*ultra_ctrl_out,ultra_speed);
			}
			
			if( ultra_start_f == 0 )
			{	
				
				Ultra_Ctrl(0.1f,thr);//超声波周期100ms
				ultra_start_f = -1;
			}
		}
		
		if(height_ctrl_mode)
		{	
			//height_ctrl_out = wz_speed_pid_v.pid_out;//原程序	
  		height_ctrl_out = ultra_ctrl_out;//我的程序
		}
		else
		{
			height_ctrl_out = thr;
		}
		
		break; //case 1
		
		default: break;
		
	} //switch
}*/
