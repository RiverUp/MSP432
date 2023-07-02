#include "Motor.h"
#include "math.h"
#include <stdlib.h>

Timer_A_PWMConfig PWMAConfig=
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	10000,
	TIMER_A_CAPTURECOMPARE_REGISTER_1,
	TIMER_A_OUTPUTMODE_RESET_SET,
	7000
};

Timer_A_PWMConfig PWMBConfig=
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	10000,
	TIMER_A_CAPTURECOMPARE_REGISTER_3,
	TIMER_A_OUTPUTMODE_RESET_SET,
	1000
};

void init_motor()
{
	//初始化p2.4，p2.6为PWM输出
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN4|GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN7);
	GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN5);
	GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4);
	GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN5);
	// Timer_A_generatePWM(TIMER_A0_BASE,&PWMAConfig);
	GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN4);
	GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN4);
}

void set_pwm_trail()
{
	GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7);
	GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN5);
	Timer_A_generatePWM(TIMER_A0_BASE,&PWMAConfig);
}

void set_pwm(int motor_left,int motor_right)
{
	int pwma,pwmb;
	if(motor_left>0)
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN5);
		GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN4);
		GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN5);
	}
	if(motor_right>0)
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN5);
		GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7);
		GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN5);
	}	
	
	pwma=abs(motor_right);
	pwmb=abs(motor_left);
	
	PWMAConfig.dutyCycle=pwma;
	PWMBConfig.dutyCycle=pwmb;
	
	Timer_A_generatePWM(TIMER_A0_BASE,&PWMAConfig);
	Timer_A_generatePWM(TIMER_A0_BASE,&PWMBConfig);
}
