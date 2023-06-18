#include "rtthread.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "HC-SR04.h"

//��⵽�����ش����ж�
void PORT2_IRQHandler(void)
{
	uint32_t status=GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
	GPIO_clearInterrupt(GPIO_PORT_P2,status);
	
	if(status&GPIO_PIN5)
	{
		//ʹ��TA0�жϣ��ر�PORT2�ж�
		Interrupt_enableInterrupt(INT_TA0_N);
		Interrupt_disableInterrupt(INT_PORT2);
		//�������Ź���
		GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
		//��ʼ����
		Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_CONTINUOUS_MODE);
	}
}

//��⵽�½��ش����ж�
void TA0_N_IRQHandler(void)
{
	//����жϱ�־λ
	Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//��ü���ֵ
	countValue=Timer_A_getCaptureCompareCount(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//�������Ź���Ϊ������������
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2,GPIO_PIN5);
	//�������ֵ��ֹͣ����
	Timer_A_clearTimer(TIMER_A0_BASE);
	Timer_A_stopTimer(TIMER_A0_BASE);
	Interrupt_disableInterrupt(INT_TA0_N);
	if(AbleToConvert->value==0)
	{
		//�����ź�������֪�̻߳�ò���
		rt_sem_release(AbleToConvert);
	}
}