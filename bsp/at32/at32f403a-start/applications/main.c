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

/* 定义线程控栈时要求RT_ALIGN_SIZE个字节对齐 */
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
*                             函数声明
*************************************************************************
*/


static void led1_thread_entry(void* parameter);
static void led2_thread_entry(void* parameter);
  /* 定义线程控制块 */
  static struct rt_thread led1_thread;
static struct rt_thread led2_thread;

/* 定义线程栈 */
static rt_uint8_t rt_led1_thread_stack[1024];
static rt_uint8_t rt_led2_thread_stack[1024];

/* 线程示例 */
int Task_Init(void)
{
        /* set LED2 pin mode to output */
        rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
        /* set LED3 pin mode to output */
        rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
        /* set LED4 pin mode to output */
        rt_pin_mode(LED4_PIN, PIN_MODE_OUTPUT);
        //初始化方向，复归完成引脚
        rt_pin_mode(DIR_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(RESET_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(RESET_PIN_FSH, PIN_MODE_INPUT);
        rt_pin_mode(RESET_PIN_FSH, PIN_MODE_INPUT);
        rt_pin_mode(START_PIN, PIN_MODE_INPUT);
        
//静态线程
	rt_thread_init(&led1_thread,                 /* 线程控制块 */
                   "led1",                       /* 线程名字 */
                   led1_thread_entry,            /* 线程入口函数 */
                   RT_NULL,                      /* 线程入口函数参数 */
                   &rt_led1_thread_stack[0],     /* 线程栈起始地址 */
                   sizeof(rt_led1_thread_stack), /* 线程栈大小 */
                   3,                            /* 线程的优先级 */
                   20);                          /* 线程时间片 */
   rt_thread_startup(&led1_thread);             /* 启动线程，开启调度 */	
	
	
	rt_thread_init(&led2_thread,                 /* 线程控制块 */
                   "led2",                       /* 线程名字 */
                   led2_thread_entry,            /* 线程入口函数 */
                   RT_NULL,                      /* 线程入口函数参数 */
                   &rt_led2_thread_stack[0],     /* 线程栈起始地址 */
                   sizeof(rt_led2_thread_stack), /* 线程栈大小 */
                   3,                            /* 线程的优先级 */
                   20);                          /* 线程时间片 */
   rt_thread_startup(&led2_thread);             /* 启动线程，开启调度 */	
	
									 
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
*                             线程定义
*************************************************************************
*/
static void led1_thread_entry(void* parameter)
{	
    while (1)
    {
#if 0
        LED1_ON;
        rt_thread_delay(500);   /* 延时500个tick */
        
        LED1_OFF;     
        rt_thread_delay(500);   /* 延时500个tick */		 		
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
	#define PWM_DEV_NAME        "tmr1pwm1"  /* PWM设备名称 */
	#define PWM_DEV_CHANNEL     1       /* PWM通道 */
	
	#define ADC_DEV_NAME        "adc1"  /* ADC 设备名称 */
	#define ADC_DEV_CHANNEL     5       /* ADC 通道 */
	#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
	#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

	struct rt_device_pwm *pwm_dev;		/* PWM设备句柄 */	
	rt_uint32_t period, pulse;
	
	rt_adc_device_t adc_dev;			/* ADC 设备句柄 */	
	static rt_uint32_t value;	
	period = 1000000;	/* 周期为0.5ms，单位为纳秒ns */
	pulse = 500000;			/* PWM脉冲宽度值，单位为纳秒ns */
	/* 查找设备 */
	pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
	/* 设置PWM周期和脉冲宽度 */
	rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
//	/* 使能设备 */
//	rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
//	/* 关闭设备通道 */
//	rt_pwm_disable(pwm_dev,PWM_DEV_CHANNEL);


	/* 查找设备 */
	adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
	/* 使能设备 */
	rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);
//	/* 读取采样值 */
//	value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
//	/* 转换为对应电压值 */
//	vol = value * REFER_VOLTAGE / CONVERT_BITS;
//	rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);
  uint8_t status = 0;
  uint8_t start_gether;
  uint32_t counter;
    while (1)
    {        
        switch(status)
        {
            
        case 0: // 初始化状态 等待信号
          if(rt_pin_read(START_PIN) == PIN_LOW)
          {
            rt_pin_write(DIR_PIN, PIN_LOW);// 向下
              /* 使能设备 */
            rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);// 开始运动
            status = 1;
            counter = 0;
          }
          if(rt_pin_read(SYS_RESET_PIN) == PIN_LOW)
          {
            status = 2;
          }
          break;
          case 1://运动状态
             PressureData[counter++] = value; // 采集曲线
             if(value > = 2048) // 到达压力阈值
             {
                /* 关闭设备通道 */
                rt_pwm_disable(pwm_dev,PWM_DEV_CHANNEL); // 关闭电机
                status = 2;
             }  
            rt_thread_mdelay(1);// 1mS 一次数据
          break;
          case 2: // 结束复归
             rt_pin_write(RESET_PIN, PIN_LOW);
             while(rt_pin_read(RESET_PIN_FSH) == PIN_LOW) // 复归完成
             {
                status = 0; // 返回初始状态
             }
          break;
        default:
          break;
        }
        
    }
}

/*---------------------------------------------------------------end of line--*/
