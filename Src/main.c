/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include "../drivers/ssd1306/oled.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LENGTH 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint8_t AdcConvEnd = 0;
uint16_t adcBuff[FFT_LENGTH];
uint8_t display_mode=2;
float fft_inputbuf[FFT_LENGTH * 2];  
float fft_outputbuf[FFT_LENGTH]; 
	 float votage,JIPING;
uint32_t ADCTIME=1000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	
	/************Adc************************/
	OLED_Init();
	Gram_clear();
	OLED_Refresh_Gram(); 
	htim2.Init.Period = ADCTIME;
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, FFT_LENGTH);	
	//HAL_ADC_Stop_DMA(&hadc1);
	//OLED_ShowStr(0, 0, "hello world", 2);//显示字符
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, FFT_LENGTH);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch,FILE *f){
	HAL_UART_Transmit(&huart1,((uint8_t*)&ch),1,2);
	return ch;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//HAL_ADC_Stop_DMA(&hadc1);
	for (int i = 0; i < FFT_LENGTH; i++){
			fft_inputbuf[i * 2] = adcBuff[i] * 3.3 / 4096-1.65;//实部3.3 / 4096是为了将ADC采集到的值转换成实际电压 后面加窗hamming*0.54-0.46*cos(2*3.14159*i/(FFT_LENGTH-1)))
			fft_inputbuf[i * 2 + 1] = 0;//虚部赋，固定0.
	}
	arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);
	arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH); 
	fft_outputbuf[0] /= FFT_LENGTH;
	for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波�??
	{
			fft_outputbuf[i] /= FFT_LENGTH/2;
	}
	//printf("FFT Result:\r\n");
	
	if(display_mode==0){
	
			gui_draw_axis();
			for(uint8_t i=0;i<=127;i++)
			{
			OLED_DrawPoint(i,64-(int)(adcBuff[i *(FFT_LENGTH/128)]*3.3/4096/3.3*64),1);          //显示前面的采集忼，形成曲�??
				//printf("%d\n",(int)(adcBuff[i *(FFT_LENGTH/128)]*3.3/4096/3.3*64));
			}						
			OLED_Refresh_Gram();                   //更新显存
			Gram_clear();                          //将OLED_GRAM数组清空，下次再赋忼（清空但不影响显示＿	
		}
	if(display_mode==1){
			OLED_CLS();
			unsigned char display_str[15];
			sprintf((char*)display_str,"MAX HZ:%dk",30000/htim2.Init.Period);
			OLED_ShowStr(0,0,display_str,2);	
			for (int i = 0; i < FFT_LENGTH; i++)//输出各次谐波
			{
				float temp=0;
				temp=fft_outputbuf[i];
				if(temp>votage){
					votage=temp;
					JIPING=(float)i*30000000/ADCTIME/FFT_LENGTH;
				}
				//printf("%d:\t%.2f\r\n", i, fft_outputbuf[i]);
			//	printf("%fhz:\t%.2f\r\n",(float)i*30000000/ADCTIME/FFT_LENGTH, fft_outputbuf[i]); //adc频率
			}
			unsigned char display_str1[15];
			sprintf((char*)&display_str1,"V:%.2f",votage);
			printf("%.2f\n",votage);
			unsigned char display_str2[15];
			sprintf((char*)&display_str2,"F:%.2f",JIPING);
			
			votage=0;//清除变量
			OLED_ShowStr(0,2,display_str1,2);
			OLED_ShowStr(0,4,display_str2,2);				
			
		}
	// 	if(display_mode==2){
	// 		float temp[128];
	// 		//gui_draw_axis();
	// 		for(uint8_t i=0;i<=127;i++)
	// 		{
	// 			for (int j = 0; j < FFT_LENGTH/128; j++)//输出各次谐波
	// 			{
	// 				 temp[i]=temp[i]+fft_outputbuf[i*FFT_LENGTH/128+j]; //归一化每32合一
	// 			}
	// 			temp[i]=temp[i]/32;
	// 			gui_draw_vline(i,127-(int)temp[i]/3.3*64,127);
	// 			printf("原始%.2f计算%d\n",temp[0],(int)((float)temp[0]*64/3.3));
	// 		}						
	// 		OLED_Refresh_Gram();                   //更新显存
	// 		Gram_clear();                          //将OLED_GRAM数组清空，下次再赋忼（清空但不影响显示＿	
		
	// }
		if(display_mode==2){
			//gui_draw_axis();
			for(uint8_t i=0;i<=127;i++)
			{
				
				gui_draw_vline(i,63-(int)(fft_outputbuf[i*32]/3.3*64),63);
				//printf("0:%.2f计算%d\n",fft_outputbuf[0],(int)(fft_outputbuf[0]/3.3*64));
			}						
			OLED_Refresh_Gram();                   //更新显存
			Gram_clear();                          //将OLED_GRAM数组清空，下次再赋忼（清空但不影响显示＿	
		
	}
		//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, FFT_LENGTH);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t ADCTIME_ARRY[]={30000000/1000,30000000/10000,30000000/100000,30000000/1000000,15};
	                         //1k          10k            100k            1000k            2000k
	static uint8_t i=0;
	HAL_Delay(5);
	printf("延时结束\n");
	switch(GPIO_Pin){//左按�??
		case GPIO_PIN_0:
					//左按�?
				
				if(0==HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)){
					i++;
					if(i>=5)i=0;
					htim2.Init.Period = ADCTIME_ARRY[i];
					printf("ADCTIME:%d\r",ADCTIME_ARRY[i]);
					HAL_TIM_Base_Init(&htim2);
					HAL_TIM_Base_Start(&htim2);
			}
				break;
		case GPIO_PIN_3:
				if(0==HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)){
					i--;
					if(i<=0||i>=5)i=0;
					htim2.Init.Period = ADCTIME_ARRY[i];
					printf("ADCTIME:%d\r",ADCTIME_ARRY[i]);
					HAL_TIM_Base_Init(&htim2);
					HAL_TIM_Base_Start(&htim2);
				}
				break;
		case GPIO_PIN_7:	
			printf("判断电平\n");
			if(0==HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)){
				printf("已触发\n");
							display_mode++;
							if(display_mode>=3)display_mode=0;
							printf("模式�?%d\n",display_mode);
							if(display_mode==0){
					
							gui_draw_axis();
							for(uint8_t i=0;i<=127;i++)
							{
							OLED_DrawPoint(i,64-(int)(adcBuff[i *(FFT_LENGTH/128)]*3.3/4096/3.3*64),1);          //显示前面的采集忼，形成曲�??
								//printf("%d\n",(int)(adcBuff[i *(FFT_LENGTH/128)]*3.3/4096/3.3*64));
							}						
							OLED_Refresh_Gram();                   //更新显存
							Gram_clear();                          //将OLED_GRAM数组清空，下次再赋忼（清空但不影响显示＿	
						}
					if(display_mode==1){
							OLED_CLS();
							unsigned char display_str[15];
							sprintf((char*)display_str,"MAX HZ:%dk",30000/htim2.Init.Period);
							OLED_ShowStr(0,0,display_str,2);	
							for (int i = 0; i < FFT_LENGTH; i++)//输出各次谐波
							{
								float temp=0;
								temp=fft_outputbuf[i];
								if(temp>votage){
									votage=temp;
									JIPING=(float)i*30000000/ADCTIME/FFT_LENGTH;
								}
								//printf("%d:\t%.2f\r\n", i, fft_outputbuf[i]);
							//	printf("%fhz:\t%.2f\r\n",(float)i*30000000/ADCTIME/FFT_LENGTH, fft_outputbuf[i]); //adc频率
							}
							unsigned char display_str1[15];
							sprintf((char*)&display_str1,"V:%.2f",votage);
							printf("%.2f\n",votage);
							unsigned char display_str2[15];
							sprintf((char*)&display_str2,"F:%.2f",JIPING);
							
							votage=0;//清除变量
							OLED_ShowStr(0,2,display_str1,2);
							OLED_ShowStr(0,4,display_str2,2);				
							
						}if(display_mode==2){
							//gui_draw_axis();
							for(uint8_t i=0;i<=127;i++)
							{
								
								gui_draw_vline(i,63-(int)(fft_outputbuf[i*32]/3.3*64),63);
								//printf("0:%.2f计算%d\n",fft_outputbuf[0],(int)(fft_outputbuf[0]/3.3*64));
							}						
							OLED_Refresh_Gram();                   //更新显存
							Gram_clear();                          //将OLED_GRAM数组清空，下次再赋忼（清空但不影响显示＿	
					}
				}
				break;
		default:
			break;

	}
	//HAL_Delay(50);
__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
