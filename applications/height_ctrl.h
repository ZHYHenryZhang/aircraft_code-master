#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "stm32f4xx.h"

typedef struct
{
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;

}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;
	float error;
	float preerror;
	float prepreerror;
	float intergral;  //����
	float output;
	float lastoutput;

}_st_height_pid;


void Ultra_PID_Init(void);

//void IncrementalPID(_st_height_pid *ultra_pid,float target,float measure);
void PositionPID(_st_height_pid *ultra_pid_p,float target,float measure); 

void My_Height_Ctrl(float T,float thr);

//void Height_Ctrl(float T,float thr);

//void WZ_Speed_PID_Init(void);

//void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed);

//void Ultra_Ctrl(float T,float thr);

//void PID_Incremental(_st_height_pid *ultra_pid,float target,float measure);
	
extern float ultra_ctrl_out;

extern float height_ctrl_out;

extern float ultra_speed,wz_speed;

extern float exp_height; //�����߶�����Ϊȫ�ֱ���

extern u8 height_ctrl_flag_star;//���߱�־λ

extern  float ultra_pid_display_p,ultra_pid_display_i,ultra_pid_display_d,ultra_pid_display_pid;//���ֵ
#endif

