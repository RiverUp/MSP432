/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//***************************************************************************************
//  Blink the LED Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP432P4xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  E. Chen
//  Texas Instruments, Inc
//  March 2015
//  Built with Code Composer Studio v6
//***************************************************************************************

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "rtthread.h"
#include "HC-SR04.h"
#include "Delay.h"
#include "Serial.h"
#include "HC-05.h"
#include "Encoder.h"
#include "oled.h"
#include "Motor.h"
#include "TCRT5000.h"
#include "Semaphore.h"
#include "Filter.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

//主时钟信号为12Mhz，在system_msp432p401r.c中配置
//rt_thread_t blink_thread = RT_NULL;

//信号量
//rt_sem_t AbleToConvert;
//extern uint32_t countValue;

float Voltage_filted;


//红灯闪烁的线程
static void blink_entry()
{
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
	while(1)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
		rt_thread_mdelay(500);
	}
}

//hcsr采样的线程
static void hcsr_entry()
{
	init_hc_sr04();
	char text[20];
	AbleToConvert=rt_sem_create("AbleToConvert",0,RT_IPC_FLAG_PRIO);
	while(1)
	{
		Interrupt_enableInterrupt(INT_PORT2);
		trigger_measure();
		//获得信号量，将其通过串口发送；最多等待50tick，防止错过触发中断导致没获得信号一直卡死
		if(rt_sem_take(AbleToConvert,50)==RT_EOK)
		{
			sprintf(text,"%f\r\n",read_hc_sr04(countValue));
			sendText(text);
			
		}
	}
}

//处理蓝牙信号的线程
static void hc05_entry()
{
	init_hc05();
	ProcessBtData=rt_sem_create("ProcessBtData",0,RT_IPC_FLAG_PRIO);
	bool connected=false;
	while(1)
	{
		#if MASTER_CAR
		while(!connected){
			sendMsgByBlueTooth("connecting");
		}
		#endif
		if(rt_sem_take(ProcessBtData,RT_WAITING_FOREVER)==RT_EOK)
		{
			//从机收到主机的连接信号
			if(!strcmp(btdata,"connecting"))
			{
				sendMsgByBlueTooth("connected\r\n");
				connected=true;
			}
			//主机收到从机的连接确认 
			if(!strcmp(btdata,"connected"))
			{
				connected=true;
				//发送给电脑告知蓝牙连接完毕
				sendText("bluetooth connected\r\n");
			}
		}
	}
}

void sendEncoderBack()
{
	int encoder_left,encoder_right;
	encoder_left=read_encoder(0);
	encoder_right=read_encoder(1);
	char text[30];
	sprintf(text,"right: %d;left: %d\r\n",encoder_right,encoder_left);
	sendText(text);
}

static void encoder_entry()
{
	init_encoder_left();
	init_encoder_right();
	rt_timer_t EncoderTimer=rt_timer_create("EncoderTimer",sendEncoderBack,RT_NULL,100,RT_TIMER_FLAG_PERIODIC);
	if(EncoderTimer!=RT_NULL)
	{
		rt_timer_start(EncoderTimer);
	}
	
}


static void motor_entry()
{
	init_motor();
	rt_timer_t MotorTimer=rt_timer_create("MotorTimer",set_pwm_trail,RT_NULL,10,RT_TIMER_FLAG_PERIODIC);
	if(MotorTimer!=RT_NULL)
	{
		rt_timer_start(MotorTimer);
	}
}

static void oled_entry()
{
	init();
	OLED_Init();
	OLED_Clear();
	while(1)
	{
    delay_ms(5);
    OLED_ShowString(0,0,(unsigned char *)"  2021  8.4");
    OLED_ShowString(0,2,(unsigned char *)" NUEDC Contest ");
  	delay_ms(25);
	}
}



void sampleVoltage()
{
	read_TCRT();
	if(processTCRTADCData->value==0)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
		rt_sem_release(processTCRTADCData);
	}
}

static void tcrt_entry()
{
	init();
	OLED_Init();
	char text[20];
	init_TCRT();
	processTCRTADCData=rt_sem_create("processTCRTADCData",0,RT_IPC_FLAG_PRIO);
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
	rt_timer_t ADCSampleTimer=rt_timer_create("ADCSampleTimer",sampleVoltage,RT_NULL,100,RT_TIMER_FLAG_PERIODIC);
	if(ADCSampleTimer!=RT_NULL)
	{
		rt_timer_start(ADCSampleTimer);
	}
	while(1)
	{
		if(rt_sem_take(processTCRTADCData,RT_WAITING_FOREVER)==RT_EOK)
		{
			Voltage=Voltage/BITS*3.3;
			Voltage_filted=Kalman_TCRT(Voltage);
			sprintf(text,"tcrt:%.2f     ",Voltage_filted);
			sendText(text);
//			if(Voltage_filted<THRESHOLD1)
//			{
//				state=White;
//			}
//			else if(Voltage_filted>THRESHOLD2)
//			{
//				state=Off_Ground;
//			}
//			else
//			{
//				state=Black;
//			}
//			
//			if(SendDatSem->value==0)
//			{
//				rt_sem_release(SendDatSem);
//			}
//			
			
		}	
	}

}


static void control_trail_entry()
{
	init_encoder_left();
	init_encoder_right();
	init_motor();
	rt_timer_t MotorTimer=rt_timer_create("MotorTimer",set_pwm_trail,RT_NULL,10,RT_TIMER_FLAG_PERIODIC);
	if(MotorTimer!=RT_NULL)
	{
		rt_timer_start(MotorTimer);
	}
}


static void display_entry()
{
	init();
	OLED_Init();
	OLED_Clear();
	char text1[20];
	char text2[20];
	init_encoder_left();
	init_encoder_right();
	
	while(1)
	{
		int encoder_left,encoder_right;
		encoder_left=read_encoder(0);
		encoder_right=read_encoder(1);
		sprintf(text1,"r:%2d ",encoder_right);
		sprintf(text2,"l:%2d ",encoder_left);
		OLED_ShowString(0,0,(unsigned char *)text1);
		OLED_ShowString(0,2,(unsigned char *)text2);
//		sprintf(text1,"dat:%.2f",Voltage_filted);
//		OLED_ShowString(0,0,(unsigned char *)text1);
	}
	
}

void control()
{
//	rt_sem_release(clearEncoder);
//	int encoder_left,encoder_right;
//	encoder_left=read_encoder(1);
//	encoder_right=read_encoder(0);
	
	GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
	
//	countTrail++;
//	
//	if(countTrail==100)
//	{
//		GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
//		countTrail=0;
//	}
	

//	int pwma=velocity_left(encoder_left)+turn();
//	int pwmb=velocity_right(encoder_right)-turn();
//	pwma=limit_pwm(pwma,8000,-8000);
//	pwmb=limit_pwm(pwmb,8000,-8000);
//	set_pwm(pwma,pwmb);
}

static void control_entry()
{
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
	GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
	rt_timer_t control_timer=rt_timer_create("Control",control,RT_NULL,100,RT_TIMER_FLAG_PERIODIC);
	if(control_timer!=RT_NULL)
	{
		rt_timer_start(control_timer); 
	}
}




int main(void)
{
	WDT_A_hold(WDT_A_BASE);
	Interrupt_enableMaster();
	
//	Timer32_initModule(TIMER32_BASE,TIMER32_PRESCALER_1,TIMER32_32BIT,TIMER32_PERIODIC_MODE);
//	Interrupt_enableInterrupt(INT_T32_INT1);
//	Timer32_setCount(TIMER32_BASE,120000);//10ms
//  Timer32_enableInterrupt(TIMER32_BASE);
//  Timer32_startTimer(TIMER32_BASE, false);
	
	Delay_Init();
  initSerial();
	
	
	init();
	OLED_Init();
	char text[20];
	init_TCRT();
	init_hc_sr04();
	
	
	while(1)
	{
		Interrupt_enableInterrupt(INT_PORT2);
			trigger_measure();
		sprintf(text,"%f\r\n",read_hc_sr04(countValue));
		sendText(text);
		OLED_ShowString(0,0,(unsigned char *)text);
		
//		Voltage=adcV/BITS*3.3;
//		sprintf(text,"v:%.2f     ",Voltage);
//		OLED_ShowString(0,0,(unsigned char *)text);
	}
//	rt_thread_t control_thread=rt_thread_create("Control",control_entry,RT_NULL,1024,25,25); 
//	if(control_thread!=RT_NULL)
//	{
//		rt_thread_startup(control_thread);
//	}
	
	
//	rt_thread_t control_trail_thread=rt_thread_create("control",control_trail_entry,RT_NULL,1024,25,50);
//	if(control_trail_thread!=RT_NULL)
//	{
//		rt_thread_startup(control_trail_thread);
//	}

//	rt_thread_t tcrt_thread=rt_thread_create("tcrt",tcrt_entry,RT_NULL,1024,24,5);
//	if(tcrt_thread!=RT_NULL)
//	{
//		rt_thread_startup(tcrt_thread);
//	}
//	rt_thread_t display_thread=rt_thread_create("display",display_entry,RT_NULL,1024,25,50);
//	if(display_thread!=RT_NULL)
//	{
//		rt_thread_startup(display_thread);
//	}
	//创建并运行Oled线程
//	rt_thread_t oled_thread=rt_thread_create("OLED",oled_entry,RT_NULL,1024,25,50);
//	if(oled_thread!=RT_NULL)
//	{
//		rt_thread_startup(oled_thread);
//	}
	
	
	//创建并运行motor线程
//	rt_thread_t motorTrail_thread=rt_thread_create("MotorTrail",motor_entry,RT_NULL,1024,25,50);	
//	if(motorTrail_thread!=RT_NULL)
//	{
//		rt_thread_startup(motorTrail_thread);
//	}
	
	
  //创建并运行encoder线程
//	rt_thread_t encoder_thread=rt_thread_create("Encoder",encoder_entry,RT_NULL,1024,25,50);
//	if(encoder_thread!=RT_NULL)
//	{
//		rt_thread_startup(encoder_thread);
//	}
	
	
	//创建并运行hcsr线程
//	rt_thread_t hcsr_thread=rt_thread_create("HC-SR04",hcsr_entry,RT_NULL,1024,25,50);
//	if(hcsr_thread!=RT_NULL)
//	{
//		rt_thread_startup(hcsr_thread);
//	}
	
	
//	rt_thread_t blink_thread=rt_thread_create("blink",blink_entry,RT_NULL,1024,25,5);
//	if(blink_thread!=RT_NULL)
//	{
//		rt_thread_startup(blink_thread);
//	}
}