/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#define WAIT_HOST  0
#define IDLE       1
#define PAGE_PROG  2

typedef void (*pFunction)(void);

// Flash configuration
#define MAIN_PROGRAM_START_ADDRESS (uint32_t)0x08004000
#define MAIN_PROGRAM_PAGE_NUMBER   16
#define NUM_OF_PAGES               (64 - MAIN_PROGRAM_PAGE_NUMBER)

// CAN identifiers
#define DEVICE_CAN_ID              0x78E
#define CMD_HOST_INIT              0x01
#define CMD_PAGE_PROG              0x02
#define CMD_BOOT                   0x03

#define CAN_RESP_OK                0x01
#define CAN_RESP_ERROR             0x02

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static CAN_TxHeaderTypeDef can_tx_header;
static CAN_RxHeaderTypeDef can_rx_header;
static uint8_t can_rx_data[8];
static uint8_t can_tx_data[8];
static FLASH_EraseInitTypeDef eraseInitStruct;

pFunction                     JumpAddress;
uint8_t                       PageBuffer[FLASH_PAGE_SIZE];
volatile int                  PageBufferPtr;
uint8_t                       PageIndex;
int                           PageCRC;

volatile uint8_t              blState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void JumpToApplication()
{
  JumpAddress = *(__IO pFunction*)(MAIN_PROGRAM_START_ADDRESS + 4);
  __set_MSP(*(__IO uint32_t*) MAIN_PROGRAM_START_ADDRESS);
  HAL_DeInit();
  JumpAddress();
}

void TransmitResponsePacket(uint8_t response)
{
	uint32_t mailbox;

	can_tx_header.StdId = DEVICE_CAN_ID;
    can_tx_header.DLC = 1;
    can_tx_data[0] = response;
    HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_data, &mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* CanHandle)
{
	HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &can_rx_header, can_rx_data);

	if (can_rx_header.StdId != DEVICE_CAN_ID)
	{
        return;
    }

    if (blState == PAGE_PROG)
    {
        memcpy(&PageBuffer[PageBufferPtr], can_rx_data, can_rx_header.DLC);
        PageBufferPtr += can_rx_header.DLC;

        if (PageBufferPtr == FLASH_PAGE_SIZE)
        {
            HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
            uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)PageBuffer, FLASH_PAGE_SIZE / 4);

            if (crc == PageCRC && PageIndex <= NUM_OF_PAGES)
            {
                HAL_FLASH_Unlock();

                uint32_t PageError = 0;

                eraseInitStruct.TypeErase = TYPEERASE_PAGES;
                eraseInitStruct.PageAddress = MAIN_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE;
                eraseInitStruct.NbPages = 1;

                HAL_FLASHEx_Erase(&eraseInitStruct, &PageError);

                for (int i = 0; i < FLASH_PAGE_SIZE; i += 4)
                {
                    HAL_FLASH_Program(TYPEPROGRAM_WORD, MAIN_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE + i, *(uint32_t*)&PageBuffer[i]);
                }

                HAL_FLASH_Lock();

                TransmitResponsePacket(CAN_RESP_OK);
            }
            else
            {
                TransmitResponsePacket(CAN_RESP_ERROR);
            }

            blState = IDLE;

            HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }

    return;
    }

    switch(can_rx_data[0])
    {
        case CMD_HOST_INIT:
            blState = IDLE;
            TransmitResponsePacket(CAN_RESP_OK);
            break;

        case CMD_PAGE_PROG:
            if (blState == IDLE)
            {
                memset(PageBuffer, 0, FLASH_PAGE_SIZE);
                memcpy(&PageCRC, &can_rx_data[2], sizeof(int));
                PageIndex = can_rx_data[1];
                blState = PAGE_PROG;
                PageBufferPtr = 0;
            }
        break;

        case CMD_BOOT:
            TransmitResponsePacket(CAN_RESP_OK);
            JumpToApplication();
        break;

        default:
        break;
    }
}
/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_CRC_Init();

    /* USER CODE BEGIN 2 */

    CAN_FilterTypeDef canFilterConfig;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
    canFilterConfig.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_Delay(2000);

    // Timed out waiting for host
    if (blState == WAIT_HOST)
    {
        JumpToApplication();
    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
