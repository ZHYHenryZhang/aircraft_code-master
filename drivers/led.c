/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��led.c
 * ����    ��LED����
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/

#include "led.h"
#include "include.h"

void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOG,ENABLE);	//ʹ��GPIOC������ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	//ѡ��Ҫ�õ�GPIO����		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��������Ϊ��ͨ���ģʽ		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//��������Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//���������ٶ�Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//��������Ϊ���� 		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);//���ÿ⺯������ʼ��GPIO	
}

#include "rc.h"
#include "ak8975.h"

u8 flag=1;
void LED_Duty()
{
	if(flag==0){
		LED0_ON;
		flag=1;
	}else{
		LED0_OFF;
		flag=0;
	}
}

void LED_MPU_Err(void)
{	
	while(1)
	{
		LED0_ON;
	}
}

void LED_Mag_Err(void)
{
	while(1)
	{
		LED0_ON;
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

