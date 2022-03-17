/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_ioe.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"

#include "ps2_spi.h"
#include "mbcrc.h"

/* Exported typedef -----------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Exported define ------------------------------------------------------------*/
#define TIMEOUT_PS2                     500          // unit=systick

/* Uncomment the line below if you will use the USART in Transmitter Mode */
 #define USART_TRANSMITTER 
/* Uncomment the line below if you will use the USART in Receiver Mode */
/* #define USART_RECEIVER   */
 
#define USARTx                           USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler
 
#define USARTx_TX_PIN                    GPIO_Pin_9                
#define USARTx_TX_GPIO_PORT              GPIOA                       
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource9
#define USARTx_TX_AF                     GPIO_AF_USART1
 
#define USARTx_RX_PIN                    GPIO_Pin_10                
#define USARTx_RX_GPIO_PORT              GPIOA                    
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource10
#define USARTx_RX_AF                     GPIO_AF_USART1

#define USARTx_DE_PIN                    GPIO_Pin_8                
#define USARTx_DE_GPIO_PORT              GPIOC                    
#define USARTx_DE_GPIO_CLK               RCC_AHB1Periph_GPIOC

#define	USARTx_TX_EN	                   GPIO_SetBits(USARTx_DE_GPIO_PORT, USARTx_DE_PIN)	 	// PC8 高电平
#define	USARTx_RX_EN	                   GPIO_ResetBits(USARTx_DE_GPIO_PORT, USARTx_DE_PIN)	 // PC8 低电平



#define BUFFERSIZE                        20
#define MAX_RX_ERROR                      10      // maximum time of slave response error
#define MAX_WHEEL_SPEED                   (uint16_t)500     // maximum speed of wheel(0~1000),unit=0.1hz
#define MIN_WHEEL_SPEED                   (uint16_t)65036      // minimum speed of wheel(64536~65536),unit=0.1hz
#define MAX_TURN_POSITION                 (int32_t)83968           // soft limit for steering motor position
#define MIN_STEER_POSITION_POS            (int32_t)7680          // minimum steer motor position for set outside fast speed
#define MIN_TURN_POSITION                 (int32_t)-83968          // soft limit for steering motor position
#define MIN_STEER_POSITION_NEG            (int32_t)-7680          // minimum steer motor position for set outside fast speed  
#define MAX_PS2_TIMEOUT                   20                // unit= scan cycle time
#define LAST_MOVE_STOP                    PSB_NONE
#define LAST_MOVE_FORWARD                 PSB_UP
#define LAST_MOVE_BACKWARD                PSB_DOWN
#define WHEEL_ID_RF                       0x01
#define WHEEL_ID_RR                       0x02
#define WHEEL_ID_LR                       0x03
#define WHEEL_ID_LF                       0x04



/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint8_t cmd[20];
  uint8_t len;

}MbCommand_Structure;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
