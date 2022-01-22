#ifndef INC_BSP_BSP_INIT_H_
#define INC_BSP_BSP_INIT_H_

#include "main.h"
#include "m_config.h"

//IAP升级
#define 	APP_START_ADDR     		0x08080000    	//应用程序1起始地址

#define UPDATE_0 (1<<0)     //事件位
#define TIMEOUT_1 (1<<1)

#define EVENTBIT_ALL ( UPDATE_0 |TIMEOUT_1)
#define  ON  1
#define  OFF  0
#define LED1(n)		(n?HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET))
extern TaskHandle_t AppTaskCreate_Handle;      				//创建线程句柄
extern TIM_HandleTypeDef htim6;
extern EventGroupHandle_t   EventGroupHandle;   //事件标志组句柄

/*bootJumpToAppTask
 * bsp_init.h
 *
 *  Created on: Jan 21, 2022
 *      Author: Administrator
 */

void   bootJumpToAppTask(TimerHandle_t xTimer);
void Bsp_Init(void);

void AppTaskCreate(void);

void Test_Task(void);
void  ledTogle_Task(void);

void MX_TIM6_Init(uint16_t arr, uint16_t psc);

#endif
