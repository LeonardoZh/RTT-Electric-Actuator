/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-08     shelton      first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_gpio.h"


/**
 * @addtogroup AT32F403AVGT7
 */

/*@{*/

/* �����߳̿�ջʱҪ��RT_ALIGN_SIZE���ֽڶ��� */
ALIGN(RT_ALIGN_SIZE)


/* defined the LED2 pin: PD13 */
#define LED2_PIN    GET_PIN(D, 13)
/* defined the LED3 pin: PD14 */
#define LED3_PIN    GET_PIN(D, 14)
/* defined the LED4 pin: PD15 */
#define LED4_PIN    GET_PIN(D, 15)

#defien DIR_PIN     GET_PIN(B, 3)
#defien RESET_PIN     GET_PIN(B, 4)
#defien RESET_PIN_FSH     GET_PIN(B, 5)
#defien START_PIN     GET_PIN(B, 6)
#defien SYS_RESET_PIN     GET_PIN(B, 7)
    uint32_t Speed = 200;
uint16_t PressureData[4096];
/*
*************************************************************************
*                             ��������
*************************************************************************
*/


static void led1_thread_entry(void* parameter);
static void led2_thread_entry(void* parameter);
  /* �����߳̿��ƿ� */
  static struct rt_thread led1_thread;
static struct rt_thread led2_thread;

/* �����߳�ջ */
static rt_uint8_t rt_led1_thread_stack[1024];
static rt_uint8_t rt_led2_thread_stack[1024];

/* �߳�ʾ�� */
int Task_Init(void)
{
        /* set LED2 pin mode to output */
        rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
        /* set LED3 pin mode to output */
        rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
        /* set LED4 pin mode to output */
        rt_pin_mode(LED4_PIN, PIN_MODE_OUTPUT);
        //��ʼ�����򣬸����������
        rt_pin_mode(DIR_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(RESET_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(RESET_PIN_FSH, PIN_MODE_INPUT);
        rt_pin_mode(RESET_PIN_FSH, PIN_MODE_INPUT);
        rt_pin_mode(START_PIN, PIN_MODE_INPUT);
        
//��̬�߳�
	rt_thread_init(&led1_thread,                 /* �߳̿��ƿ� */
                   "led1",                       /* �߳����� */
                   led1_thread_entry,            /* �߳���ں��� */
                   RT_NULL,                      /* �߳���ں������� */
                   &rt_led1_thread_stack[0],     /* �߳�ջ��ʼ��ַ */
                   sizeof(rt_led1_thread_stack), /* �߳�ջ��С */
                   3,                            /* �̵߳����ȼ� */
                   20);                          /* �߳�ʱ��Ƭ */
   rt_thread_startup(&led1_thread);             /* �����̣߳��������� */	
	
	
	rt_thread_init(&led2_thread,                 /* �߳̿��ƿ� */
                   "led2",                       /* �߳����� */
                   led2_thread_entry,            /* �߳���ں��� */
                   RT_NULL,                      /* �߳���ں������� */
                   &rt_led2_thread_stack[0],     /* �߳�ջ��ʼ��ַ */
                   sizeof(rt_led2_thread_stack), /* �߳�ջ��С */
                   3,                            /* �̵߳����ȼ� */
                   20);                          /* �߳�ʱ��Ƭ */
   rt_thread_startup(&led2_thread);             /* �����̣߳��������� */	
	
									 
	return 0;
}

int main(void)
{
    Task_Init(); 
    while (1)
    {
    }
}
/*
*************************************************************************
*                             �̶߳���
*************************************************************************
*/
static void led1_thread_entry(void* parameter)
{	
    while (1)
    {
#if 0
        LED1_ON;
        rt_thread_delay(500);   /* ��ʱ500��tick */
        
        LED1_OFF;     
        rt_thread_delay(500);   /* ��ʱ500��tick */		 		
#endif
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(Speed);
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_mdelay(Speed);
      
			rt_kprintf("LED1 IS OK\r\n");
    }
}
static void led2_thread_entry(void* parameter)
{
	#define PWM_DEV_NAME        "tmr1pwm1"  /* PWM�豸���� */
	#define PWM_DEV_CHANNEL     1       /* PWMͨ�� */
	
	#define ADC_DEV_NAME        "adc1"  /* ADC �豸���� */
	#define ADC_DEV_CHANNEL     5       /* ADC ͨ�� */
	#define REFER_VOLTAGE       330         /* �ο���ѹ 3.3V,���ݾ��ȳ���100����2λС��*/
	#define CONVERT_BITS        (1 << 12)   /* ת��λ��Ϊ12λ */

	struct rt_device_pwm *pwm_dev;		/* PWM�豸��� */	
	rt_uint32_t period, pulse;
	
	rt_adc_device_t adc_dev;			/* ADC �豸��� */	
	static rt_uint32_t value;	
	period = 1000000;	/* ����Ϊ0.5ms����λΪ����ns */
	pulse = 500000;			/* PWM������ֵ����λΪ����ns */
	/* �����豸 */
	pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
	/* ����PWM���ں������� */
	rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
//	/* ʹ���豸 */
//	rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
//	/* �ر��豸ͨ�� */
//	rt_pwm_disable(pwm_dev,PWM_DEV_CHANNEL);


	/* �����豸 */
	adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
	/* ʹ���豸 */
	rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);
//	/* ��ȡ����ֵ */
//	value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
//	/* ת��Ϊ��Ӧ��ѹֵ */
//	vol = value * REFER_VOLTAGE / CONVERT_BITS;
//	rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);
  uint8_t status = 0;
  uint8_t start_gether;
  uint32_t counter;
    while (1)
    {        
        switch(status)
        {
            
        case 0: // ��ʼ��״̬ �ȴ��ź�
          if(rt_pin_read(START_PIN) == PIN_LOW)
          {
            rt_pin_write(DIR_PIN, PIN_LOW);// ����
              /* ʹ���豸 */
            rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);// ��ʼ�˶�
            status = 1;
            counter = 0;
          }
          if(rt_pin_read(SYS_RESET_PIN) == PIN_LOW)
          {
            status = 2;
          }
          break;
          case 1://�˶�״̬
             PressureData[counter++] = value; // �ɼ�����
             if(value > = 2048) // ����ѹ����ֵ
             {
                /* �ر��豸ͨ�� */
                rt_pwm_disable(pwm_dev,PWM_DEV_CHANNEL); // �رյ��
                status = 2;
             }  
            rt_thread_mdelay(1);// 1mS һ������
          break;
          case 2: // ��������
             rt_pin_write(RESET_PIN, PIN_LOW);
             while(rt_pin_read(RESET_PIN_FSH) == PIN_LOW) // �������
             {
                status = 0; // ���س�ʼ״̬
             }
          break;
        default:
          break;
        }
        
    }
}

/*---------------------------------------------------------------end of line--*/
