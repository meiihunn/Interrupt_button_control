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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
///*! \brief If Modbus Master ASCII support is enabled. */
//#define MB_MASTER_ASCII_ENABLED					( 0 )
///*! \brief If Modbus Master RTU support is enabled. */
//#define MB_MASTER_RTU_ENABLED					( 1 )
///*! \brief If Modbus Master TCP support is enabled. */
//#define MB_MASTER_TCP_ENABLED					( 0 )
///*! \brief If Modbus Slave ASCII support is enabled. */
//#ifndef INC_MODBUS_CRC_H_
//#define INC_MODBUS_CRC_H_
//uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
//bool check_crc16(uint8_t *buffer, uint16_t length);
//#endif



#define COL		GPIOA
#define COL1	GPIO_PIN_3
#define COL2	GPIO_PIN_4
#define COL3	GPIO_PIN_6
#define COL4	GPIO_PIN_7
#define ROW		GPIOA
#define ROW1	GPIO_PIN_0
#define ROW2	GPIO_PIN_1
#define ROW3	GPIO_PIN_2

int a[3][4]={
		{'1','2','3','4'},
		{'5','6','7','8'},
		{'9','#','0','*'},
};
char scan_keypad();
volatile uint8_t g_row_flag =0;
volatile uint8_t data_received_flag=0;

char key='\0';
char last_key ='\0';
char button_input[4]= {}; //chuoi nhap phim tu ban phim
volatile uint8_t button_count = 0;
volatile uint8_t rx_count = 0;
//volatile uint8_t rx_data = 0;
//volatile uint8_t tx_buffer[256];
//volatile uint8_t rx_buffer[256];
uint8_t tx_buffer[256];
uint8_t rx_buffer[256];



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length_data) { // length
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length_data; i++) {
        crc ^= buffer[i];  // XOR voi du lieu dau vao: crc = crc^buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
//                crc = (crc >> 1) ^ 0xA001;  // Đa thức CRC-16 Modbus
            	crc >>= 1;
            	crc = crc ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
// ham gui du lieu modbus qua uart
// neu khong can truyen du lieu thi truyen data = NULL va length_data = 0


//MODBUS_NHAN_1_PHIM
//void Send_Modbus(UART_HandleTypeDef *huart, uint8_t slaveID, uint16_t address, uint8_t data, uint8_t length_data) {
////    uint8_t tx_buffer[256];
////    if(length_data > 250) return;
//	uint8_t tx_buffer[8]={0};
//    tx_buffer[0] = slaveID;                 // dia chi Slave
//    tx_buffer[1] = 0x06;                     // Function Code: Read Holding Register
//    tx_buffer[2] = (address >> 8) & 0xFF;    // address High
//    tx_buffer[3] = address & 0xFF;           // address Low
//    tx_buffer[4] = (data >> 8) & 0xFF;   // write Data Hi
//    tx_buffer[5] = data & 0xFF;          //write Data Lo
//
//    uint16_t crc = Modbus_CRC16(tx_buffer, 6 );
//    tx_buffer[6 ] = crc & 0xFF;      // CRC Low byte
//    tx_buffer[7 ] = (crc >> 8) & 0xFF; // CRC High byte
//
//    HAL_UART_Transmit(huart, tx_buffer, sizeof(tx_buffer), 100);
//}

////MODBUS_NHAP_NHIEU_PHIM
void Send_Modbus(UART_HandleTypeDef *huart, uint8_t slaveID,
		uint16_t address,uint16_t quantity,uint16_t byte_count
		,uint8_t * data, uint8_t length_data) {
//    uint8_t tx_buffer[256];
//    if(length_data > 250) return;
//	uint8_t tx_buffer[256]={0};
    tx_buffer[0] = slaveID;                 // dia chi Slave
    tx_buffer[1] = 0x10;                     // Function Code: Read Holding Register
    tx_buffer[2] = (address >> 8) & 0xFF;    // address High
    tx_buffer[3] = address & 0xFF;           // address Low
    tx_buffer[4] = (quantity >> 8) & 0xFF;   // quantity Hi
    tx_buffer[5] = quantity & 0xFF;          //quantity Lo
//    tx_buffer[6] = byte_count & 0xFF; //byte_count
    tx_buffer[6] = (quantity *2) & 0xFF; //byte_count

    for (int i = 0; i < quantity; i++) {
        tx_buffer[7 + (i*2 )] = (data[i] >> 8) & 0xFF; // High byte
        tx_buffer[8 + (i*2 )] = (data[i]) & 0xFF;        // Low byte
    }
    uint16_t crc = Modbus_CRC16(tx_buffer, 7 + (quantity*2)); // 7 + length_data / sizeof(data)????
    tx_buffer[7 + (quantity*2)] = crc & 0xFF;      // CRC Low byte
    tx_buffer[8 + (quantity*2)] = (crc >> 8) & 0xFF; // CRC High byte

    HAL_UART_Transmit(huart, tx_buffer, /*sizeof(tx_buffer)*/ 9 + (quantity * 2), 100);
}


void Received_Modbus (UART_HandleTypeDef *huart, uint8_t slaveID,uint8_t function,
		uint16_t address, uint16_t quantity){
	uint8_t byte_count = quantity *2;

//	HAL_UART_Receive_IT(huart, rx_buffer,8); // nhan o cho khac -> duwjng cowf -> nhay vao received_modbus
	// tach frane -> so sanh -> neu giong thi done

	uint16_t received_crc = (rx_buffer[8 - 1] << 8) | rx_buffer[8 - 2];
	uint16_t calculated_crc = Modbus_CRC16(rx_buffer, 8 - 2);
//	HAL_UART_Receive(huart, rx_buffer, 8, 100);
	if (rx_buffer[0] == tx_buffer[0] && rx_buffer[1] == tx_buffer[1]) {
		if(rx_buffer [2] == tx_buffer[2] && rx_buffer[3] == tx_buffer[3]){
			if (received_crc == calculated_crc){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				printf ("Du lieu hop le: ");
				for (int i=0; i<8; i++){
					printf("%02X ", rx_buffer[i]);
				}
			}
			else{
				printf("Loi CRC!");
			}
		}
		else{
			printf("Sai Address hoac Quantity!");
		}
	}
	else{
		printf("Sai Slave ID hoac Function Code!");
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HAL_Delay(10);
	if(GPIO_Pin == GPIO_PIN_0){
		g_row_flag=1;
		key = scan_keypad();
	}
	else if(GPIO_Pin == GPIO_PIN_1)
	{
		g_row_flag=2;
		key = scan_keypad();
	}
	else if (GPIO_Pin == GPIO_PIN_2)
	{
		g_row_flag=3;
		key = scan_keypad();
	}
}
char scan_keypad(void){
	key='\0';
	int selected_col=0;
	for (int col =0; col <4; col++){

//		HAL_GPIO_WritePin(COL, COL1 | COL2 | COL3 | COL4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL, COL1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL, COL2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL, COL3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL, COL4, GPIO_PIN_SET);

		if(col == 0) HAL_GPIO_WritePin(COL, COL1, GPIO_PIN_RESET);
		if(col == 1) HAL_GPIO_WritePin(COL, COL2, GPIO_PIN_RESET);
		if(col == 2) HAL_GPIO_WritePin(COL, COL3, GPIO_PIN_RESET);
		if(col == 3) HAL_GPIO_WritePin(COL, COL4, GPIO_PIN_RESET);

		if (/*g_row_flag==1 &&*/(HAL_GPIO_ReadPin(ROW, ROW1) == GPIO_PIN_RESET)){
			selected_col = col;
			key = a[0][selected_col];
		}
		else if(/*g_row_flag==2 &&*/(HAL_GPIO_ReadPin(ROW, ROW2) == GPIO_PIN_RESET)){
			selected_col = col;
			key = a[1][selected_col];
		}
		else if(/*g_row_flag==3 && */(HAL_GPIO_ReadPin(ROW, ROW3) == GPIO_PIN_RESET)){
			selected_col = col;
			key = a[2][selected_col];
		}
		if(key != '\0') break;
	}

	HAL_GPIO_WritePin(COL, COL1, 0);
	HAL_GPIO_WritePin(COL, COL2, 0);
	HAL_GPIO_WritePin(COL, COL3, 0);
	HAL_GPIO_WritePin(COL, COL4, 0);

	g_row_flag = 0;
	return key;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_data=0;
//uint8_t tx_data[6]="hello\n";
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
//		for (int i=0; i<8; i++){
		rx_buffer[rx_count++]= rx_data;
//			rx_count++;
//		}
//		HAL_UART_Transmit(&huart1,&rx_data,sizeof(rx_data), 100);

        if (rx_count >= 8) {
            data_received_flag = 1; // Dung co nhan du data
            rx_count = 0;           // Reset bien diem
        }

	}
}
// HIEN THI PHIM (UART)
//void display_key(char key) {
//    HAL_UART_Transmit(&huart1, (uint8_t*)&key, 4, 100);
//    char newline = '\n';
//    HAL_UART_Transmit(&huart1, (uint8_t*)&newline, 1, 100);
//}

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
//  uint16_t registers[4] = {1, 2, 3, 4}; // Giả sử nhập "1234"
//  Send_Modbus(&huart1, 0x11, 0x0001,0x0002,0x04, registers, 4);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Transmit(&huart1,tx_data,sizeof(tx_data),100);
  HAL_UART_Receive_IT(&huart1,&rx_data, 1);
//  Send_Modbus();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      HAL_Delay(100);
	  if(key != '\0' ){
		  if(button_count <4){
			  button_input[button_count]= key;
			  button_count++;
//			  Send_Modbus(&huart1, 0x11, 0x0001,0x0002,0x04, (uint8_t )*button_input, sizeof(button_input));
	//		  Send_Modbus(&huart1, 0x01, 0x0001, (uint8_t )key, 1);

		  key ='\0';
		  }
		  if (button_count == 4){
//			  Send_Modbus(&huart1, 0x01, 0x0001, (uint8_t *)button_input, 1);
			  Send_Modbus(&huart1, 0x11, 0x0001,4,8, (uint8_t *)button_input, sizeof(button_input));

			  button_count =0;
		  }
	  }

	  if(data_received_flag){
//		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		  HAL_Delay (1000);
		  Received_Modbus(&huart1,0x11,0x0001, rx_buffer, 4);

//		  HAL_UART_Transmit(&huart1, &rx_data, sizeof(rx_data), 100);
		  data_received_flag=0;
	  }
  }
}


  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
	static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */


  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
