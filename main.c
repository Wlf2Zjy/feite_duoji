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
extern UART_HandleTypeDef huart1;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 接收状态枚举
typedef enum {
    RX_STATE_WAIT_HEADER1,      // 等待帧头第一个字节
    RX_STATE_WAIT_HEADER2,      // 等待帧头第二个字节
    RX_STATE_WAIT_LENGTH,       // 等待长度字节
	  RX_STATE_WAIT_ID,           // 新增：等待舵机ID字节
    RX_STATE_WAIT_CMD,          // 等待指令字节
    RX_STATE_WAIT_CONTENT,      // 等待内容字节
    RX_STATE_WAIT_END           // 等待结束字节
} RxState;

// 舵机反馈解析状态
typedef enum {
    SERVO_RX_WAIT_HEADER1,
    SERVO_RX_WAIT_HEADER2,
    SERVO_RX_WAIT_ID,
    SERVO_RX_WAIT_LENGTH,
    SERVO_RX_WAIT_ERROR,
    SERVO_RX_WAIT_PARAM_LOW,
    SERVO_RX_WAIT_PARAM_HIGH,
    SERVO_RX_WAIT_CHECKSUM
} ServoRxState;

uint8_t rx_id;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_BUFFER_SIZE 64       // 接收缓冲区大小
#define FRAME_HEADER_1 0xFE     // 帧头第一个字节
#define FRAME_HEADER_2 0xFE     // 帧头第二个字节
#define FRAME_END 0xFA          // 帧结束字节
#define FIXED_SERVO_ID 1        // 固定舵机ID

// 舵机指令定义
#define SERVO_HEADER1 0xFF
#define SERVO_HEADER2 0xFF
#define SERVO_READ_DATA 0x02
#define SERVO_WRITE_DATA 0x03 //重置指令
#define SERVO_POSITION_ADDR 0x38
#define SERVO_POSITION_LEN 0x02  // 定义零位数据长度
#define SERVO_RESET_ADDR 0x28 //重置指令
#define SERVO_RESET_VALUE 0x80 //重置指令

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 添加接收缓冲区
uint8_t uart_rxByte;            // 串口接收的单个字节
uint8_t uart2_rxByte;           // 串口接收的单个字节2
volatile RxState rxState = RX_STATE_WAIT_HEADER1; // 接收状态
volatile RxState rxState2 = RX_STATE_WAIT_HEADER1; // 接收状态2

// 协议帧变量
uint8_t rx_cmd;                 // 接收到的指令
uint8_t rx_len;                 // 接收到的长度
uint8_t rx_content[64];         // 接收到的内容
uint8_t rx_content_index;       // 内容索引
volatile uint8_t frameReceived = 0; // 帧接收完成标志

uint8_t rx2_cmd;                // 接收到的指令2
uint8_t rx2_len;                // 接收到的长度2
uint8_t rx2_content[64];        // 接收到的内容2
uint8_t rx2_content_index;      // 内容索引2
volatile uint8_t frameReceived2 = 0; // 帧接收完成标志2

// 舵机位置读取相关变量
volatile uint16_t servo_position = 0; // 舵机当前位置
volatile uint8_t servo_position_updated = 0; // 位置更新标志

// 舵机反馈解析相关变量
volatile ServoRxState servo_rx_state = SERVO_RX_WAIT_HEADER1;
uint8_t servo_rx_id;
uint8_t servo_rx_length;
uint8_t servo_rx_error;
volatile uint8_t servo_rx_complete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void RS485_SendEnable(void) //发送使能函数
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

// 接收使能函数
void RS485_ReceiveEnable(void)
{
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
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // 使用USART1发送调试信息
    RS485_SendEnable();
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

// 发送响应帧
void send_response_frame(uint8_t cmd, uint8_t return_len, uint8_t *return_content) {
    uint8_t frame[64];
    uint8_t index = 0;
    
    // 帧头
    frame[index++] = FRAME_HEADER_1;
    frame[index++] = FRAME_HEADER_2;
    
    // 返回长度 (ID1字节 + 指令1字节 + 内容n字节 + 结束符1字节)
    frame[index++] = return_len + 3; //长度计算
    
    // 舵机ID (使用接收到的ID)  <--- 关键修改
    frame[index++] = rx_id;
    
    // 返回指令
    frame[index++] = cmd;
    
    // 返回内容
    if (return_content != NULL && return_len > 0) {
        memcpy(&frame[index], return_content, return_len);
        index += return_len;
    }
    
    // 结束位
    frame[index++] = FRAME_END;
    
    // 发送响应
    RS485_SendData(frame, index);
}

// 角度限制在0~360度之间
uint16_t angle_to_position(float angle) {
    if(angle < 0) angle = 0;
    if(angle > 360) angle = 360;
    return (uint16_t)((angle / 360.0f) * 4095);
}

// 位置转角度
float position_to_angle(uint16_t position) {
    return (position / 4095.0f) * 360.0f;
}

void feetech_servo_rotate(uint8_t servo_id, float angle, uint16_t speed) {
    uint16_t pos = angle_to_position(angle);
    uint8_t buf[12];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = servo_id;  // 使用接收到的ID  
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

    HAL_UART_Transmit(&huart2, packet, 13, 1000);
}

// 舵机重置零位
void servo_reset_zero(void) {
    uint8_t reset_cmd[8] = {
        SERVO_HEADER1,
        SERVO_HEADER2,
        FIXED_SERVO_ID,
        0x04,           // 长度
        SERVO_WRITE_DATA,// 写指令(0x03)
        SERVO_RESET_ADDR, // 重置寄存器地址
        SERVO_RESET_VALUE, // 重置值
        0x00            // 校验位(临时)
    };
    
    // 计算校验和
    uint8_t checksum = 0;
    for(int i = 2; i < 7; i++) { // ID到最后一个参数
        checksum += reset_cmd[i];
    }
    reset_cmd[7] = ~checksum;
    
    // 发送重置指令
    HAL_UART_Transmit(&huart2, reset_cmd, sizeof(reset_cmd), 100);
    HAL_Delay(50); // 确保舵机有足够时间处理
}


// 读取舵机位置 (阻塞式等待)
uint16_t read_servo_position(void) {
    uint8_t read_cmd[8] = {
        SERVO_HEADER1,
        SERVO_HEADER2,
        FIXED_SERVO_ID,
        0x04,           // 长度
        SERVO_READ_DATA,// 读取指令(0x02)
        SERVO_POSITION_ADDR, // 位置寄存器地址
        SERVO_POSITION_LEN,  // 读取长度
        0x00            // 校验位(临时)
    };
    
    // 计算校验和
    uint8_t checksum = 0;
    for(int i = 2; i < 7; i++) { // ID到最后一个参数
        checksum += read_cmd[i];
    }
    read_cmd[7] = ~checksum;
    
    // 重置舵机接收状态
    servo_rx_state = SERVO_RX_WAIT_HEADER1;
    servo_rx_complete = 0;
    
    // 发送读取指令
    HAL_UART_Transmit(&huart2, read_cmd, sizeof(read_cmd), 100);
    
    // 等待舵机响应 (最多20ms)
    uint32_t start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < 20) {
        if(servo_rx_complete) {
            return servo_position;
        }
    }
    
    return 0xFFFF; // 读取失败
}

// 处理接收到的协议帧
void process_protocol_frame(void) {
    uint8_t response_content[2] = {0}; // 位置响应内容
    uint8_t response_len = 0;
    
    // 舵机控制指令 (0x01)
    if (rx_cmd == 0x01 && rx_content_index >= 2) {
        // 指令格式: [位置高8位, 位置低8位]
        uint16_t position = (rx_content[0] << 8) | rx_content[1];
        
        // 使用接收到的ID控制舵机
        feetech_servo_rotate(rx_id, (position / 4095.0f) * 360.0f, 5000);
        
        // 发送响应
        response_content[0] = 0x01; // 成功
        response_len = 1;
        send_response_frame(0x01, response_len, response_content);
    }
		
    // 读取位置指令 (0x02)
    else if (rx_cmd == 0x02) {
        // 读取舵机当前位置
        uint16_t current_pos = read_servo_position();
        
        if(current_pos != 0xFFFF) {
            // 准备响应内容 [位置高8位, 位置低8位]
            response_content[0] = (current_pos >> 8) & 0xFF; // 高字节
            response_content[1] = current_pos & 0xFF;        // 低字节
            response_len = 2;
        } else {
            // 读取失败
            response_content[0] = 0xFF; // 错误标记
            response_content[1] = 0xFF; 
            response_len = 2;
        }
        
        // 发送响应
        send_response_frame(0x02, response_len, response_content);
    }
		
    // 重置舵机零位指令 (0x03)
    else if (rx_cmd == 0x03) {
        // 执行舵机重置
        servo_reset_zero();
        
        // 发送响应
        response_content[0] = 0x01; // 成功
        response_len = 1;
        send_response_frame(0x03, response_len, response_content);
    }
    
    // 重置接收状态
    rx_content_index = 0;
}


// 处理接收到的协议帧
void process_protocol_frame2(void) {
    // 舵机控制指令 (0x00)
    if (rx2_cmd == 0x00 && rx2_content_index >= 2) {
        // 指令格式: [位置高8位, 位置低8位]
        uint16_t position = (rx2_content[0] << 8) | rx2_content[1];
        RS485_SendEnable();
        HAL_UART_Transmit(&huart1, &rx2_content[0], 1, 100);
        HAL_UART_Transmit(&huart1, &rx2_content[1], 1, 100);
        RS485_ReceiveEnable();
    }
    
    // 重置接收状态
    rx2_content_index = 0;
}


// 串口接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        switch (rxState) {
            case RX_STATE_WAIT_HEADER1:
                if (uart_rxByte == FRAME_HEADER_1) {
                    rxState = RX_STATE_WAIT_HEADER2;
                }
                break;
                
            case RX_STATE_WAIT_HEADER2:
                if (uart_rxByte == FRAME_HEADER_2) {
                    rxState = RX_STATE_WAIT_LENGTH;
                } else {
                    rxState = RX_STATE_WAIT_HEADER1;
                }
                break;
                
            case RX_STATE_WAIT_LENGTH:
                rx_len = uart_rxByte;
                rx_content_index = 0;
                
                if (rx_len >= 3) { // 至少包含ID、指令和结束符
                    rxState = RX_STATE_WAIT_ID;  // 新增状态
                } else {
                    rxState = RX_STATE_WAIT_HEADER1; // 无效长度
                }
                break;
                
            // 新增：等待ID状态
            case RX_STATE_WAIT_ID:
                rx_id = uart_rxByte;  // 存储接收到的ID
                rxState = RX_STATE_WAIT_CMD;
                break;
                
            case RX_STATE_WAIT_CMD:
                rx_cmd = uart_rxByte;
                if (rx_len > 3) { // 接收 (长度-ID-指令-结束符)
                    rxState = RX_STATE_WAIT_CONTENT;
                } else {
                    rxState = RX_STATE_WAIT_END;
                    frameReceived = 1;
                }
                break;
                
            case RX_STATE_WAIT_CONTENT:
                if (rx_content_index < sizeof(rx_content)) {
                    rx_content[rx_content_index++] = uart_rxByte;
                }
                
                // 检查是否接收完所有内容 (长度 = ID1字节 + 指令1字节 + 内容n字节 + 结束符1字节)
                if (rx_content_index >= (rx_len - 3)) {
                    rxState = RX_STATE_WAIT_END;
                }
                break;
                
            case RX_STATE_WAIT_END:
                if (uart_rxByte == FRAME_END) {
                    // 完整帧接收完成
                    frameReceived = 1;
                }
                rxState = RX_STATE_WAIT_HEADER1;
                break;
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &uart_rxByte, 1);
    }
    else if (huart->Instance == USART2) {
        // 舵机协议解析
        switch (servo_rx_state) {
            case SERVO_RX_WAIT_HEADER1:
                if (uart2_rxByte == SERVO_HEADER1) {
                    servo_rx_state = SERVO_RX_WAIT_HEADER2;
                }
                break;
                
            case SERVO_RX_WAIT_HEADER2:
                if (uart2_rxByte == SERVO_HEADER2) {
                    servo_rx_state = SERVO_RX_WAIT_ID;
                } else {
                    servo_rx_state = SERVO_RX_WAIT_HEADER1;
                }
                break;
                
            case SERVO_RX_WAIT_ID:
                servo_rx_id = uart2_rxByte;
                if (servo_rx_id == FIXED_SERVO_ID) {
                    servo_rx_state = SERVO_RX_WAIT_LENGTH;
                } else {
                    servo_rx_state = SERVO_RX_WAIT_HEADER1; // 非目标ID，丢弃
                }
                break;
                
            case SERVO_RX_WAIT_LENGTH:
                servo_rx_length = uart2_rxByte;
                // 位置反馈帧长度应为4 (错误码+位置低+位置高+校验和)
                if (servo_rx_length == 0x04) {
                    servo_rx_state = SERVO_RX_WAIT_ERROR;
                } else {
                    servo_rx_state = SERVO_RX_WAIT_HEADER1; // 长度不符，丢弃
                }
                break;
                
            case SERVO_RX_WAIT_ERROR:
                servo_rx_error = uart2_rxByte;
                servo_rx_state = SERVO_RX_WAIT_PARAM_LOW;
                break;
                
            case SERVO_RX_WAIT_PARAM_LOW:
                // 位置低字节 (舵机返回的是低字节在前)
                servo_position = uart2_rxByte;
                servo_rx_state = SERVO_RX_WAIT_PARAM_HIGH;
                break;
                
            case SERVO_RX_WAIT_PARAM_HIGH:
                // 位置高字节 (舵机返回的是高字节在后)
                servo_position |= (uart2_rxByte << 8);
                servo_rx_state = SERVO_RX_WAIT_CHECKSUM;
                break;
                
            case SERVO_RX_WAIT_CHECKSUM:
                // 校验和验证 (简单跳过)
                servo_rx_complete = 1; // 标记完成
                servo_rx_state = SERVO_RX_WAIT_HEADER1;
                break;
                
            default:
                servo_rx_state = SERVO_RX_WAIT_HEADER1;
                break;
        }
        
        // 重新启动USART2接收
        HAL_UART_Receive_IT(&huart2, &uart2_rxByte, 1);
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

  // 启动串口接收中断
  HAL_UART_Receive_IT(&huart1, &uart_rxByte, 1);
  HAL_UART_Receive_IT(&huart2, &uart2_rxByte, 1);
  
  // 初始测试：转动舵机到中间位置
  //feetech_servo_rotate(180.0f, 500);
  //HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      // 检查帧接收标志
      if(frameReceived) {
          // 处理协议帧
          process_protocol_frame();
          
          // 重置标志
          frameReceived = 0;
      }
      if(frameReceived2) {
          // 处理协议帧
          process_protocol_frame2();
          
          // 重置标志
          frameReceived2 = 0;
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
