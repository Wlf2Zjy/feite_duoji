/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
extern UART_HandleTypeDef huart2;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rs485_rxBuffer[RX_BUFFER_SIZE];
uint16_t rs485_rxIndex = 0;
uint8_t commandReceived = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void RS485_SendEnable(void) //发送使能函数
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

// 接收使能函数
void RS485_ReceiveEnable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

// 发送数据函数
void RS485_SendData(uint8_t *pData, uint16_t Size)
{
    RS485_SendEnable();
    HAL_UART_Transmit(&huart1, pData, Size, 1000);
    HAL_Delay(1);  // 确保数据发送完成
    RS485_ReceiveEnable();
}

// 接收数据函数
// 非阻塞接收函数
HAL_StatusTypeDef RS485_ReceiveData(uint8_t *pData, uint16_t Size) {
    // 确保在接收模式
    RS485_ReceiveEnable();
    
    // 检查是否有数据
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        HAL_UART_Receive(&huart1, pData, Size, 0);
        return HAL_OK;
    }
    return HAL_TIMEOUT;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart2_send_buf(uint8_t *buf, uint16_t len) {
    HAL_UART_Transmit(&huart2, buf, len, 1000);
}

uint8_t calc_checksum(uint8_t *buf, uint8_t len) {
    uint16_t sum = 0;
    for(uint8_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return ~(sum & 0xFF);
}
/**
  * @brief  The application entry point.
  * @retval int
  */
// 角度限制在0~360度之间
uint16_t angle_to_position(float angle) {
    if(angle < 0) angle = 0;
    if(angle > 360) angle = 360;
    return (uint16_t)((angle / 360.0f) * 4095);
}

void feetech_servo_rotate(uint8_t id, float angle, uint16_t speed) {
    uint16_t pos = angle_to_position(angle);//转指令码
    uint8_t buf[12];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 0x09;
    buf[4] = 0x03;
    buf[5] = 0x2A;

    buf[6] = (pos & 0xff);
    buf[7] = (pos >> 8);

    buf[8] = 0x00;
    buf[9] = 0x00;

    buf[10] = speed & 0xFF;
    buf[11] = (speed >> 8);

    uint8_t checksum = 0;
    for(int i = 2; i < 12; i++) {
        checksum += buf[i];
    }
    checksum = ~checksum;

    uint8_t packet[13];
    memcpy(packet, buf, 12);
    packet[12] = checksum;

    uart2_send_buf(packet, 13);
}

// 指令解析函数
void parse_command(uint8_t *cmd) {
    int servo_id;
    float angle;
    
    // 尝试解析指令格式 "#ID,角度"
    if (sscanf((char*)cmd, "#%d,%f", &servo_id, &angle) == 2) {
        // 验证参数范围
        if (servo_id >= 1 && servo_id <= 254 && angle >= 0.0f && angle <= 360.0f) {
            char msg[50];
            sprintf(msg, "Moving servo %d to %.1f degrees\r\n", servo_id, angle);
            uart2_send_buf((uint8_t*)msg, strlen(msg));
            
            // 控制舵机转动
            feetech_servo_rotate(servo_id, angle, 5000);
        } else {
            uart2_send_buf((uint8_t*)"Invalid parameters\r\n", 20);
        }
    } else {
        uart2_send_buf((uint8_t*)"Invalid command format\r\n", 23);
    }
}


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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
	
  /* USER CODE BEGIN 2 */
  //uint8_t sendData[] = "Hello, RS485!";//试验
  //uint8_t receiveData[20];
  RS485_ReceiveEnable(); // 确保485处于接收模式
  uart2_send_buf((uint8_t*)"RS485 Servo Controller Ready\r\n", 30);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//HAL_UART_Transmit(&huart2, (uint8_t *)"hello windows!\r\n", 16 , 0xffff);
		 //HAL_Delay(1000);  //延
		/*
		      // 使舵机转到90度，以速度500(单位:步/s)
        feetech_servo_rotate(1, 90.0f, 5000);
        HAL_Delay(2000);

        // 使舵机转到180度，以速度300
        feetech_servo_rotate(1, 180.0f, 5000);
        HAL_Delay(2000);

        // 使舵机转回0度，以默认速度
        feetech_servo_rotate(1, 0.0f, 5000);
        HAL_Delay(2000);*/
		// 发送数据
        /*RS485_SendData(sendData, sizeof(sendData));

        // 接收数据
        if (RS485_ReceiveData(receiveData, sizeof(receiveData)) == HAL_OK)
        {
            // 处理接收到的数据
        }

        HAL_Delay(1000);*/
				uint8_t byte;
        
        // 尝试接收一个字节
        if (RS485_ReceiveData(&byte, 1) == HAL_OK) {
            // 缓冲区未满时存储数据
            if (rs485_rxIndex < RX_BUFFER_SIZE - 1) {
                rs485_rxBuffer[rs485_rxIndex++] = byte;
                
                // 检测到换行符表示指令结束
                if (byte == '\n') {
                    rs485_rxBuffer[rs485_rxIndex] = '\0'; // 添加字符串结束符
                    commandReceived = 1;
                }
            } else {
                // 缓冲区溢出，重置
                rs485_rxIndex = 0;
            }
        }
        
        // 处理接收到的完整指令
        if (commandReceived) {
            parse_command(rs485_rxBuffer);
            rs485_rxIndex = 0;
            commandReceived = 0;
        }
        
        HAL_Delay(1); // 短暂延时，减少CPU占用
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
#ifdef USE_FULL_ASSERT
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
