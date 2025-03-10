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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



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
char button_input[16]= {}; //chuoi nhap phim tu ban phim
volatile uint8_t button_count = 0;
volatile uint8_t rx_count = 0;
volatile uint16_t count_time_send_func16 =0;
//volatile uint16_t count_time_send_func03 =0;

volatile uint16_t count_time_received =0;
volatile uint8_t count_time_send_flag =0;
volatile uint8_t count_time_received_flag =0;
// khai bao cho Func03
volatile uint8_t read_request_flag =0;
volatile uint16_t count_time_read =0;

uint8_t tx_buffer[256];
uint8_t rx_buffer[256];

//volatile=0
uint16_t last_read_value[10]={0};// luu gia tri lan doc truoc
uint8_t pc_slaveID = 0x01;		// Slave ID cua PC (theo cau hinh tool modbus slave)
//uint16_t reg_address = 0x0001;  // Dia chi thanh ghi se doc
//uint8_t reg_quantity = 0x01;    // So luong thanh ghi can doc
uint16_t reg_address = 0x0000;
uint8_t reg_quantity =10;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length_data) { // length
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length_data; i++) {
        crc ^= buffer[i];  // XOR voi du lieu dau vao: crc = crc^buffer[i];
        for (uint8_t j = 0; j < 8; j++) {

            if (crc & 0x0001) {
            	crc >>= 1;
            	crc = crc ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

//MODBUS_NHAN_1_PHIM
//void Send_Modbus(_Func16)(UART_HandleTypeDef *huart, uint8_t slaveID
//		, uint16_t address, uint8_t data, uint8_t length_data) {
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

////MODBUS_NHAP_NHIEU_PHIM (FUNCTION 16_Write Multiple Registers)
void Send_Modbus_Func16(UART_HandleTypeDef *huart, uint8_t slaveID,
		uint16_t address,uint16_t quantity,uint16_t byte_count
		,uint8_t * data, uint8_t length_data) {

    tx_buffer[0] = slaveID;                 // dia chi Slave
    tx_buffer[1] = 0x10;                     // Function Code: Read Holding Register
    tx_buffer[2] = (address >> 8) & 0xFF;    // address High
    tx_buffer[3] = address & 0xFF;           // address Low
    tx_buffer[4] = (quantity >> 8) & 0xFF;   // quantity Hi
    tx_buffer[5] = quantity & 0xFF;          //quantity Lo
//    tx_buffer[6] = byte_count & 0xFF; //byte_count
    tx_buffer[6] = (quantity *2) & 0xFF; //byte_count

    for (int i = 0; i < quantity; i++) {
        tx_buffer[7 + (i*2)] = (data[i] >> 8) & 0xFF; // High byte
        tx_buffer[8 + (i*2)] = (data[i]) & 0xFF;        // Low byte
    }
    uint16_t crc = Modbus_CRC16(tx_buffer, 7 + (quantity*2)); // 7 + length_data / sizeof(data)????
    tx_buffer[7 + (quantity*2)] = crc & 0xFF;      // CRC Low byte
    tx_buffer[8 + (quantity*2)] = (crc >> 8) & 0xFF; // CRC High byte

    HAL_UART_Transmit(huart, tx_buffer, /*sizeof(tx_buffer)*/ 9 + (quantity * 2), 100);
}
void Send_Modbus_Func03(UART_HandleTypeDef *huart, uint8_t slaveID,
			uint16_t address, uint8_t quantity/*, uint8_t length_data*/){
	tx_buffer[0] = slaveID;
	tx_buffer[1]= 0x03; // function code 03
	tx_buffer[2]= (address >> 8) &0xFF;
	tx_buffer[3]=address & 0xFF; // dia chi bat dau doc
	tx_buffer[4]= (quantity >> 8) & 0xFF;
	tx_buffer[5]= quantity & 0xFF;

	uint16_t crc = Modbus_CRC16(tx_buffer, 6 );
	tx_buffer[6 ] = crc & 0xFF;      // CRC Low byte
	tx_buffer[7 ] = (crc >> 8) & 0xFF; // CRC High byte

//	HAL_UART_Transmit(huart, tx_buffer, sizeof(tx_buffer), 100);
	HAL_UART_Transmit (huart, tx_buffer, 8, 100);
}

void Received_Modbus (){

//	HAL_UART_Receive_IT(huart, rx_buffer,8); // nhan o cho khac -> duwjng cowf -> nhay vao received_modbus
	// tach frane -> so sanh -> neu giong thi done => DONE

	uint16_t received_crc = (rx_buffer[8 - 1] << 8) | rx_buffer[8 - 2];
	uint16_t calculated_crc = Modbus_CRC16(rx_buffer, 8 - 2);
	//FUNC_16
	if(rx_buffer[1]== 0x10){
		if (rx_buffer[0] == tx_buffer[0] && rx_buffer[1] == tx_buffer[1]) {
			if(rx_buffer [2] == tx_buffer[2] && rx_buffer[3] == tx_buffer[3]){
				if (received_crc == calculated_crc){
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					HAL_Delay(500);
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
		memset(button_input, 0, sizeof(button_input));
	}

	// FUNC_03
	if(rx_buffer[1] == 0x03){
		uint8_t byte_count = rx_buffer[2];
		uint16_t response_length = 3 + byte_count +2;
		uint16_t received_crc = (rx_buffer[response_length - 1] << 8) | rx_buffer[response_length - 2];
		uint16_t calculated_crc = Modbus_CRC16(rx_buffer, response_length -2);

		if(received_crc == calculated_crc) {
			uint16_t read_data[10];
			for (int i = 0; i < 10; i++) {
				read_data[i] = (rx_buffer[3 + (i * 2)] << 8) | rx_buffer[4 + (i * 2)];
			}

// 1 THANH GHI THAY DOI GIA TRI => LED BLINK
//				uint16_t read_data = (rx_buffer[3] << 8) | rx_buffer[4];
				// So sanh voi gia tri lan doc truoc
//				if(read_data != last_read_value) {
////					last_read_value = read_data;
//					printf("Func03: Gia tri thanh ghi thay doi: 0x%04X\r\n", read_data);
//					for(int i=0; i<3; i++){
//						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//						HAL_Delay(100);
//						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//						HAL_Delay(100);
//					}
//				}


// 10 THANH GHI, BAT KY THANH GHI NAO THAY DOI GIA TRI => LED BLINK
			if (memcmp(read_data, last_read_value, sizeof(read_data)) != 0){
				memcpy(last_read_value, read_data, sizeof(last_read_value));
				for(int i=0; i<3; i++){
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_Delay(100);
				}
			}
		}
		else {
			printf("Func03: Loi CRC!\r\n");
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
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

		HAL_GPIO_WritePin(COL, COL1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL, COL2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL, COL3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COL, COL4, GPIO_PIN_SET);

		if(col == 0) HAL_GPIO_WritePin(COL, COL1, GPIO_PIN_RESET);
		if(col == 1) HAL_GPIO_WritePin(COL, COL2, GPIO_PIN_RESET);
		if(col == 2) HAL_GPIO_WritePin(COL, COL3, GPIO_PIN_RESET);
		if(col == 3) HAL_GPIO_WritePin(COL, COL4, GPIO_PIN_RESET);

		if ((HAL_GPIO_ReadPin(ROW, ROW1) == GPIO_PIN_RESET)){
			selected_col = col;
			key = a[0][selected_col];
		}
		else if((HAL_GPIO_ReadPin(ROW, ROW2) == GPIO_PIN_RESET)){
			selected_col = col;
			key = a[1][selected_col];
		}
		else if((HAL_GPIO_ReadPin(ROW, ROW3) == GPIO_PIN_RESET)){
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
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_data=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
		rx_buffer[rx_count++]= rx_data;
		count_time_received = 0;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM2) {

    	count_time_send_func16++;
    	if(count_time_send_func16 > 1000){
    		count_time_send_flag =1;
    	}

    	count_time_received++;
    	if(count_time_received > 500 && rx_count>0){
    		count_time_received_flag =1;
    	}

    	count_time_read++;
    	if(count_time_read >= 2500){
    		read_request_flag = 1;
    		count_time_read =0;
//    		Send_Modbus_Func03();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1,&rx_data, 1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	  if(key != '\0' ){
		  button_input[button_count]= key;
		  button_count++;
		  count_time_send_func16 =0;
		  count_time_send_flag=0;
		  key ='\0';
		  }
	  if (count_time_send_flag && button_count >0){
		  Send_Modbus_Func16(&huart1, 0x11, 0x0001,button_count,button_count*2,(uint8_t *)button_input, sizeof(button_input));
		  count_time_send_flag = 0;

		  button_count =0;
		  }

	  if(count_time_received_flag ){
		  Received_Modbus();

		  rx_count = 0;

		  memset(rx_buffer, 0, sizeof(rx_buffer));
		  memset(tx_buffer, 0, sizeof(tx_buffer));

		  count_time_received_flag=0;

	  }
	  if(read_request_flag){
		  Send_Modbus_Func03(&huart1, 0x11, reg_address, reg_quantity);
		  read_request_flag =0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
