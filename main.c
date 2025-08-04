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
#include <stdarg.h>

extern UART_HandleTypeDef huart2;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint8_t header;     // 指令头 (0xA5)
    uint8_t servo_id;   // 舵机ID
    uint16_t position;  // 位置值 (0-4095)
} ServoCommand;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define RX_BUFFER_SIZE 4        // 4字节指令长度
#define COMMAND_HEADER 0xA5     // 指令头标识
#define MAX_SERVO_ID 254        // 最大舵机ID

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 添加接收缓冲区
uint8_t uart_rxByte; // 串口接收的单个字节
uint8_t uart_rxBuffer[RX_BUFFER_SIZE];   // 接收缓冲区
uint8_t uart_rxIndex = 0;                // 接收索引
volatile uint8_t commandReceived = 0;     // 命令接收完成标志
ServoCommand currentCommand;              // 当前解析的命令

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void RS485_SendEnable(void) //发送使能函数
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

// 接收使能函数
void RS485_ReceiveEnable(void)
{
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

// 发送数据函数
void RS485_SendData(uint8_t *pData, uint16_t Size)
{
    RS485_SendEnable();
    HAL_UART_Transmit(&huart1, pData, Size, 1000);
    HAL_Delay(1);  // 确保数据发送完成
    RS485_ReceiveEnable();
}

// 接收函数为非阻塞模式
HAL_StatusTypeDef RS485_ReceiveData(uint8_t *pData, uint16_t Size)
{
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

// 添加调试信息发送函数
void debug_printf(char *format, ...) {
    char buffer[128];
		RS485_SendEnable();
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // 使用USART1发送调试信息
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
		RS485_ReceiveEnable();
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

    buf[6] = (pos & 0xff); //低
    buf[7] = (pos >> 8); //高

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

    // 使用USART2发送到舵机
    RS485_SendEnable(); // 切换到发送模式
    HAL_UART_Transmit(&huart2, packet, 13, 1000); // huart2
    HAL_Delay(1); // 确保数据发送完成
    RS485_ReceiveEnable(); // 切换回接收模式
}

// 指令解析函数
void parse_command(uint8_t *data) {
    currentCommand.header = data[0];
    currentCommand.servo_id = data[1];
    currentCommand.position = (data[3] << 8) | data[2]; // 小端序解析
    
    // 验证指令头
    if(currentCommand.header != COMMAND_HEADER) {
        return;
    }
    
    // 验证舵机ID范围
    if(currentCommand.servo_id == 0 || currentCommand.servo_id > MAX_SERVO_ID) {
        return;
    }
    
    // 验证位置范围 (0-4095)
    if(currentCommand.position > 4095) {
        return;
    }
    
    // 执行舵机控制
    feetech_servo_rotate(currentCommand.servo_id, currentCommand.position, 500);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        // 将接收到的字节存入缓冲区
        uart_rxBuffer[uart_rxIndex++] = uart_rxByte;
        
        // 检查是否收到完整指令
        if(uart_rxIndex >= RX_BUFFER_SIZE) {
            commandReceived = 1; // 设置命令接收标志
            uart_rxIndex = 0;    // 重置索引
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &uart_rxByte, 1);
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
  RS485_ReceiveEnable();

// 发送启动消息


 // 启动串口接收中断
  HAL_UART_Receive_IT(&huart1, &uart_rxByte, 1);
  
  // 初始测试：转动舵机到中间位置
  feetech_servo_rotate(1, 2048, 500);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		    // 检查命令接收标志
    if(commandReceived) {
        // 解析并执行命令
        parse_command(uart_rxBuffer);
        
        // 重置标志
        commandReceived = 0;
			
			// 清除缓冲区
        memset((void*)uart_rxBuffer, 0, RX_BUFFER_SIZE);
		}
		HAL_Delay(1);
		


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
