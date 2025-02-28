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
#include <string.h>
//#include  <HardwareSerial.h>
//#ifndef MYBOOLEAN_H
//#define MYBOOLEAN_H
//
//#define false 0
//#define true 1
//typedef int bool; // or #define bool int

//#endif
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*! \brief If Modbus Master ASCII support is enabled. */
#define MB_MASTER_ASCII_ENABLED					( 0 )
/*! \brief If Modbus Master RTU support is enabled. */
#define MB_MASTER_RTU_ENABLED					( 1 )
/*! \brief If Modbus Master TCP support is enabled. */
#define MB_MASTER_TCP_ENABLED					( 0 )
/*! \brief If Modbus Slave ASCII support is enabled. */
#ifndef INC_MODBUS_CRC_H_
//#define INC_MODBUS_CRC_H_
//uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
//bool check_crc16(uint8_t *buffer, uint16_t length);
#endif



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
char button_input[5]= {0}; //chuoi nhap phim tu ban phim
volatile uint8_t button_count = 0;
volatile uint8_t tx_buffer[10];
volatile uint8_t rx_buffer[10];

//uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length);
//uint8_t buffer_modbus[256];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */

//static const uint8_t table_crc_high[] = {
//    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
//    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
//    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
//    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
//    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
//    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
//    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
//    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
//    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
//    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
//    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
//    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
//    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
//    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
//    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
//    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
//    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
//    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
//    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
//    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
//    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
//    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
//    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
//    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
//    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
//    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
//};
//
///* Table of CRC values for low-order byte */
//static const uint8_t table_crc_low[] = {
//    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
//    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
//    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
//    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
//    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
//    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
//    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
//    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
//    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
//    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
//    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
//    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
//    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
//    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
//    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
//    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
//    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
//    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
//    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
//    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
//    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
//    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
//    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
//    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
//    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
//    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
//};
////ham check crc
//uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
//{
//    uint8_t crc_high = 0xFF; /* high CRC byte initialized */
//    uint8_t crc_low  = 0xFF; /* low CRC byte initialized */
//    unsigned int i; /* will index into CRC lookup */
//
//    /* pass through message buffer */
//    while (buffer_length--) {
//        i = crc_low ^ *buffer++; /* calculate the CRC  */
//        crc_low  = crc_high ^ table_crc_high[i];
//        crc_high = table_crc_low[i];
//    }
//
//    return (crc_high << 8 | crc_low);
//}
//
//bool check_crc16(uint8_t *buffer, uint16_t length)
//{
//  if (length < 2) return 0;
//
//  uint16_t computed = crc16(buffer, length - 2);
//  uint16_t expected_crc = ((uint16_t)buffer[length - 2] << 8) | (buffer[length - 1]);
//
//  return computed == expected_crc ? 1 : 0;
//}
//
//
//static uint32_t Button;
//if ((uint32_t)(HAL_GetTick() - Button)>= 100)
//{
//	static uint8_t button_transmit[8];
//	button_transmit[0]= 0x01; //address slave
//	button_transmit[1]= 0x03; //function code - read holding register
//	button_transmit[2]= 0x00; //address start high
//	button_transmit[3]= 0x00; //address start low
//	button_transmit[4]= 0x00; //length read high
//	button_transmit[5]= 0x02; //length read low
//
//	uint16_t calculate_crc = crc16(button_transmit, 6);
//	button_transmit[6]=calculate_crc & 0xFF; //CRC low
//	button_transmit[7]=(calculate_crc >> 8)& 0xFF;
//
//	Modbus_SendData(&huart1, button_transmit, 8);
//	Button = HAL_GetTick();
//
//}
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
//         tham khao
//        CRCLo = (char)crc;
//        CRCHi = (char)(crc>>8);
    }
    return crc;
}
// ham gui du lieu modbus qua uart
// neu khong can truyen du lieu thi truyen data = NULL va length_data = 0
void Send_Modbus(UART_HandleTypeDef *huart, uint8_t slaveID, uint16_t address, uint8_t data, uint8_t length_data) {
//    uint8_t tx_buffer[256];
//    if(length_data > 250) return;
	uint8_t tx_buffer[8]={0};
    tx_buffer[0] = slaveID;                 // dia chi Slave
    tx_buffer[1] = 0x06;                     // Function Code: Read Holding Register
    tx_buffer[2] = (address >> 8) & 0xFF;    // address High
    tx_buffer[3] = address & 0xFF;           // address Low
    tx_buffer[4] = (data >> 8) & 0xFF;   // write Data Hi
    tx_buffer[5] = data & 0xFF;          //write Data Lo
//    tx_buffer[6+i]=data[i];
//    for (int i =0; i< length_data; i++){
//    	tx_buffer[6+i]=data[i];
//    }
    uint16_t crc = Modbus_CRC16(tx_buffer, 6 );
    tx_buffer[6 ] = crc & 0xFF;      // CRC Low byte
    tx_buffer[7 ] = (crc >> 8) & 0xFF; // CRC High byte

    HAL_UART_Transmit(huart, tx_buffer, sizeof(tx_buffer), 100);
}
//void Modbus_ReceiveResponse(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint8_t count_bytes) {
//    HAL_UART_Receive(huart, rx_buffer, count_bytes, 100); // nhan data tu uart
//
//    //kiem tra CRC
//    uint16_t received_crc = (rx_buffer[count_bytes - 1] << 8) | rx_buffer[count_bytes - 2];
//    uint16_t calculated_crc = Modbus_CRC16(rx_buffer, count_bytes - 2);
//
//    if (received_crc == calculated_crc) {
////        printf("Du lieu hop le: ");
//        for (int i = 0; i < count_bytes; i++) {
//            printf("\n %02X ", rx_buffer[i]); //X: in ra hex, 0: fill 0 if output it hon 2 so, 2: output hien 2 so
//        }
////        printf("\n");
//    } else {
////        printf("CRC sai! \n");
//    }
//}

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
uint8_t rx_data;
uint8_t tx_data[6]="hello\n";
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
//		HAL_UART_Transmit(&huart1,&rx_data,sizeof(rx_data), 100);
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
		data_received_flag = 1;
	}
}

void display_key(char key) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&key, 1, 100);
    char newline = '\n';
    HAL_UART_Transmit(&huart1, (uint8_t*)&newline, 1, 100);
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
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1,tx_data,sizeof(tx_data),100);
  HAL_UART_Receive_IT(&huart1,&rx_data,1);
//  Send_Modbus();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  key = scan_keypad();

//	  uint16_t value = modbus_read(0x01, 0x03, 0x000A);
//	  if (value != 0xFFFF) {
//	      char msg[20];
//	      sprintf(msg, "Read Value: %u\r\n", value);
//	      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
//	  }

//	  uint8_t buf[10];
//	  buf[0] = 0x02;
//      Send_Modbus(&huart1, 0x01, 0x0000, 2, buf[0], sizeof(buf)); // gui yeu cau doc 2 thanh ghi
      HAL_Delay(100);

      uint8_t rx_buffer[9];
//      Modbus_ReceiveResponse(&huart1, rx_buffer, 9); // nhan phan hoi
      HAL_Delay(1000);
	  if(key != '\0' ){
//		  button_input[button_count]= key;
//		  button_count++;
		  Send_Modbus(&huart1, 0x01, 0x0001, (uint8_t )
, 1);
		  key ='\0';
//		  if(key != last_key){
//		  strncat(button_input, &key, 1);
//		  if(key == '#'){
//			  tx_buffer[0]=1; // slave address
//			  tx_buffer[1]=6;
//			  tx_buffer[2]=0x00;
//			  tx_buffer[3]=0x01;
//			  tx_buffer[4]=1;
//			  tx_buffer[5]=1;
//			  Send_Modbus(&huart1, 0x01, 0x0000,2, tx_buffer, 6);
////			  Send_Modbus(1,3,0,0, modbus_tx_buffer, 6);
//			  memset(button_input, 0, sizeof(button_input));
//		  }

//		  display_key(key);
//		  key = '\0';

//		  if (button_count == 4){
//			  Send_Modbus(&huart1, 0x01, 0x0001, (uint8_t *)button_input, 1);
//			  button_count =0;

//			  memset(button_input, 0, sizeof(button_input));
//void *memset(void *str, int c, size_t n) sao chép ký tự c (một unsigned char)
//tới n ký tự đầu tiên của chuỗi được trỏ tới bởi tham số str.

//			  uint16_t key = (button_input[0]-'0')*1000 + // value
//					  (button_input[1]-'0')*100 +
//					  (button_input[2]-'0')*10 +
//					  (button_input[3]-'0');
//			  Send_Modbus(&huart1, 0x01, 0x0000, key, buf[0],sizeof(buf)); //value

//		  }
//		  }
	  }
//	  last_key = key;
//	  if(data_received_flag){
//		  HAL_UART_Transmit(&huart1, &rx_data, sizeof(rx_data), 100);
//		  data_received_flag=0;
//	  }
//	  HAL_Delay(100);
//	  key = '\0';
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

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
