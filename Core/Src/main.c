/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy62.h"
#include "zigbee.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RIGHT 1
#define Left -1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int abs(int x){
  if(x<0)return (0-x);
  return x;
}
float max(float a,float b){
  return a>b?a:b;
}
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
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); 
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  jy62_Init(&huart3);
  SetBaud(115200);
  SetHorizontal();
  InitAngle();
  Calibrate();

  zigbee_Init(&huart2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//0.86 0.013 10
float kp=0.86,ki=0.015,kd=10;
float integral[4]={0},prev[4]={0};
int setspeed=30;
float distance=0;
int delay=11;
float angle;
int pwm1,pwm2,pwm3,pwm4;
//cm/s
float pid(float setspeed,float actualspeed,int num){
  integral[num]+=setspeed-actualspeed;
  float out=actualspeed+kp*(setspeed-actualspeed)+ki*integral[num]+kd*(prev[num]-actualspeed);
  prev[num]=actualspeed;        
  if(out>=169)out=169;
  if(out<=0)out=0;
  return out;
}
void rotate(float speed1,float speed2,float speed3,float speed4,int lr){
  float s_speed1=setspeed*(1+lr);
  float s_speed3=setspeed*(1+lr);
  float s_speed2=setspeed*0.1;
  float s_speed4=setspeed*0.1;
  pwm1=(int)(pid(s_speed1,speed1,1)*5.88);
  pwm2=(int)(pid(s_speed2,speed2,2)*5.88);
  pwm3=(int)(pid(s_speed3,speed3,3)*5.88);
  pwm4=(int)(pid(s_speed4,speed4,4)*5.88);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm1); 
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2); 
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm3); 
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm4); 

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)  //refresh PID value every 10ms
{
    if (htim->Instance == TIM6) {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);

        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);

        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);

        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
        
        int count1=__HAL_TIM_GetCounter(&htim1);
        int count2=__HAL_TIM_GetCounter(&htim3);
        int count3=__HAL_TIM_GetCounter(&htim4);
        int count4=__HAL_TIM_GetCounter(&htim8);
        __HAL_TIM_SetCounter(&htim1,0);
        __HAL_TIM_SetCounter(&htim3,0);
        __HAL_TIM_SetCounter(&htim4,0);
        __HAL_TIM_SetCounter(&htim8,0);
        float speed1=(float)count1*100/1040*3.1416*6.5;//cm/s
        float speed2=(float)count2*100/1040*3.1416*6.5;
        float speed3=(float)count3*100/1040*3.1416*6.5;
        float speed4=(float)count4*100/1040*3.1416*6.5;
        float avgspeed=(speed1+speed2+speed3+speed4)/4.0;
        distance+=avgspeed*0.0001;
        if(delay==1){
           angle=GetYaw();
           delay=5;
        }else delay--;

        
       // u1_printf("%d    %d\n",(int)angle,(int)(10*distance));
        if(distance>=0.2){
          rotate(speed1,speed2,speed3,speed4,RIGHT);
          if(49<angle && angle<180){
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); 
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); 
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); 
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); 
              if(max(max(speed1,speed2),max(speed3,speed4))<10){
                distance=0;
                InitAngle();
                for(int i=0;i<4;i++){
                  integral[i]=0;
                  prev[i]=0;
                }
                goto s;
              }
           }
           return;
        }
       s:
        pwm1=(int)(pid(setspeed,speed1,1)*5.88);
        pwm2=(int)(pid(setspeed,speed2,2)*5.88);
        pwm3=(int)(pid(setspeed,speed3,3)*5.88);
        pwm4=(int)(pid(setspeed,speed4,4)*5.88);
        //300  ---- 43cm/s
        //1000 ---- 170
        //6.2x ---- x
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm1); 
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2); 
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm3); 
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm4); 

    }
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
