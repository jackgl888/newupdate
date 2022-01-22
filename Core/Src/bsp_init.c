#include "bsp_init.h"


TaskHandle_t Test_Task_Handle		  	   = NULL;		//测试线程句柄

TaskHandle_t ledTogle_Task_Handle          =NULL;

SemaphoreHandle_t Ledtogle_Bin_Handle = NULL;		//反馈处理二值信号量

EventGroupHandle_t   EventGroupHandle = NULL;   //程序运行事件标志组句柄

TaskHandle_t  Boot_JumpApp_Handle          = NULL ;     //boot跳转句柄


TIM_HandleTypeDef htim6;
/*
 * bsp_init.c
 *
 *  Created on: Jan 21, 2022
 *      Author: Administrator
 */


void AppTaskCreate(void)
{



	Ledtogle_Bin_Handle=  xSemaphoreCreateBinary();
	EventGroupHandle=  xEventGroupCreate();

	BaseType_t xReturn = pdPASS;
	taskENTER_CRITICAL(); //进入临界区
	xReturn = xTaskCreate((TaskFunction_t) Test_Task,           			//任务入口函数
			(const char*   ) "ADC_Sampling_Task",         			//任务名字
			(uint16_t      ) 128,                     				//任务栈大小
			(void*         ) NULL,                    				//任务入口函数参数
			(UBaseType_t   ) 2,                       				//任务的优先级
			(TaskHandle_t* ) &Test_Task_Handle);  			//任务控制块指针
	if (pdPASS == xReturn)
		;
	else __NOP();



	xReturn = xTaskCreate((TaskFunction_t)  ledTogle_Task,           					//任务入口函数
			(const char*   ) "Test_Task",         					//任务名字
			(uint16_t      ) 128,                     				//任务栈大小
			(void*         ) NULL,                    				//任务入口函数参数
			(UBaseType_t   ) 5,                       				//任务的优先级
			(TaskHandle_t* ) &ledTogle_Task_Handle);  					//任务控制块指针
	if (pdPASS == xReturn)
		;
	else __NOP();


	xReturn = xTaskCreate((TaskFunction_t)	bootJumpToAppTask,								 //任务入口函数
						   (const char*   ) "bootJump",							 //任务名字
						   (uint16_t	  ) 128,									 //任务栈大小
						   (void*		  ) NULL,									 //任务入口函数参数
						   (UBaseType_t   ) 3,										 //任务的优先级
						   (TaskHandle_t* ) & Boot_JumpApp_Handle  );					 //任务控制块指针
	 if (pdPASS == xReturn)
;
	 else __NOP();





	vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
	taskEXIT_CRITICAL(); //退出临界区
}








/*-----------------------------------------------------------------------------
 * @name  : Test_Task
 * @brief : 测试任务，乱写一通去测即可
 * @param : None
 * @retval: None
 * @date  : 2021/09/10
 * @note  :
 * ---------------------------------------------------------------------------*/
void Test_Task(void)
{
	uint8_t a=0;

	while(1)
	{
		a++;
		if(a==5)
		{
			//xSemaphoreGive(Ledtogle_Bin_Handle); //释放信号量
			xEventGroupSetBits(EventGroupHandle,UPDATE_0);
			a= 0;
		}

		vTaskDelay(1000);
	}
}






void  ledTogle_Task(void)
{


	xSemaphoreTake(Ledtogle_Bin_Handle, portMAX_DELAY);


	while(1)
	{

		LED1(ON);
		vTaskDelay(500);
		LED1(OFF);
		vTaskDelay(500);
	}

}
/*-----------------------------------------------------------------------------
 * @name  :  APP_Enter
 * @brief :  跳转动作
 * @param :
 * @retval:
 * @date  :
 * @note  :
 * ---------------------------------------------------------------------------*/
void bootJumpApp(void)
{
	typedef void(*iapfun)(void);

	iapfun 	jump2app;

	__disable_irq();



	jump2app=(iapfun)*(__IO uint32_t*)(APP_START_ADDR+4);
	//用户代码区第二个字为程序开始地址(复位地址)
	__set_PSP(*(__IO uint32_t*)APP_START_ADDR);
	__set_CONTROL(0);
	__set_MSP(*(__IO uint32_t*)APP_START_ADDR); //初始化APP堆栈指针(用户代码区的第一个字用

	//于存放栈顶地址)
	jump2app();         //跳转到 APP.


}


/*-----------------------------------------------------------------------------
 * @name  :  bootJumpToApp
 * @brief :  boot跳转到app去执行
 * @param :
 * @retval:
 * @date  :
 * @note  :
 * ---------------------------------------------------------------------------*/
void   bootJumpToAppTask(TimerHandle_t xTimer)
{

	while(1)
	{

		if(EventGroupHandle!=NULL)
		{


			//等待事件组中的相应事件位
			//等待事件组中的相应事件位
			xEventGroupWaitBits((EventGroupHandle_t )EventGroupHandle,
					(EventBits_t ) EVENTBIT_ALL,
					(BaseType_t )pdTRUE,
					(BaseType_t )pdTRUE,
					(TickType_t )portMAX_DELAY);
			//定时时间到并且不在升级状态

			bootJumpApp();

		}
		vTaskDelay(50);
	}






}




/*-----------------------------------------------------------------------------
 * @name  : HAL_TIM_Base_MspInit
 * @brief : 定时器硬件抽象初始化
 * @param : None
 * @retval: None
 * @date  : 2021/09/10
 * @note  :
 * ---------------------------------------------------------------------------*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
}



/*-----------------------------------------------------------------------------
 * @name  : HAL_TIM_Base_MspDeInit
 * @brief : 定时器硬件反抽象初始化
 * @param : None
 * @retval: None
 * @date  : 2021/09/10
 * @note  :
 * ---------------------------------------------------------------------------*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
}

void MX_TIM6_Init(uint16_t arr, uint16_t psc)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = psc;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = arr;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim6); //开启定时器中断
}
