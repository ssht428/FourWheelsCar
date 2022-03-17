/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main program body
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
  *  V0.3: increase turn motor step,
  *           stop moving when lost connection with PS2 joystick
  *    other
  */
#define USE_PRINTF_DEBUG    0

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */ 
  /* Public variables -which is also used in other c files------------------------*/

__IO IT_SystickTypeDef  gSystick;


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t curTick;
__IO uint16_t btnPress;
__IO uint32_t curKey = 0xFFFF;    // current ps2 key
__IO uint32_t curKeyValue = PSB_NONE;     // = curKey & 0xFFFF
__IO uint32_t curKeyState = 0;            // = curKey & 0xF0000
__IO uint32_t lastKeyValue = 0xFFFF;   // last ps2 key,include PSB_NONE
__IO uint32_t oldKeyValue = 0xFFFF;    // old valid key
__IO uint32_t lastFwdBkd = LAST_MOVE_STOP;  // store last pressed key of wheel direction
__IO uint32_t ps2KeyTimeout = 0;               // time of ps2 no button is pressed 
__IO int flagWheelRun = LAST_MOVE_STOP;                // flag of whether wheel is running

uint8_t aTxBuffer[BUFFERSIZE] = {0x00,0x06,0x00,0x43,0x00,0x60};
uint8_t aRxBuffer [BUFFERSIZE];
__IO uint8_t ubRxIndex = 0x00;
__IO uint8_t ubTxIndex = 0x00;
__IO uint8_t ubTxSize = 6;
__IO uint8_t ubRxError = 0;
__IO uint32_t TimeOut = 0x00;  
uint8_t bSend = 0;
__IO uint16_t curWheelFwdSpeed = 30;
__IO uint16_t curWheelBkdSpeed = 65506;
__IO uint16_t steerWheelFwdSpeed = 36;
__IO uint16_t steerWheelBkdSpeed = 65500;
__IO uint32_t steerMode = 0;       // 0=outside wheel run fast than inside,1=same speed of outside and inside 
__IO uint16_t curWheelSpeedLF = 30;     // left_front wheel speed
__IO uint16_t curWheelSpeedLR = 30;     // left_rear wheel speed
__IO uint16_t curWheelSpeedRF = 30;     // right_front wheel speed
__IO uint16_t curWheelSpeedRR = 30;     // right_rear wheel speed



int stopPS2 = 0;            // stop receive new ps2 key for continue current operation
static __IO int32_t curTurnPosition = 0;
static __IO uint32_t avoidButtonTwicePress = 0;


// wmd = wheel motors' driver
MbCommand_Structure wmdSpeedFwd =     // speed mode:forword
{
  {0x00,0x06,0x00,0x43,0x00,0x1E,0xF9,0xC7}, //   target speed and CRC should be changed accordingly
  8   // include crc's 2bytes
};
MbCommand_Structure wmdSpeedBkd = 
{
  {0x00,0x06,0x00,0x43,0xFF,0xE2,0xB8,0x76}, // not contain target speed and CRC
  8   // include crc's 2bytes
};
MbCommand_Structure wmdSpeedLF =                   // left front wheel speed for steering,= inner_speed+20%
{
  {0x01,0x06,0x00,0x43,0x00,0x1E,0xF8,0x16},       // change slave address accordingly 
  8   // include crc's 2bytes  
};
MbCommand_Structure wmdSpeedLR =                    // left rear wheel speed for steering,= inner_speed+10%
{
  {0x02,0x06,0x00,0x43,0x00,0x1E,0xF8,0x25},       
  8   // include crc's 2bytes  
};
MbCommand_Structure wmdSpeedRF =                    // right front wheel speed for steering,= inner_speed+20%
{
  {0x04,0x06,0x00,0x43,0x00,0x1E,0xF8,0x43},        // change slave address accordingly 
  8   // include crc's 2bytes  
};
MbCommand_Structure wmdSpeedRR =                    // right rear wheel speed for steering,= inner_speed+10%
{
  {0x03,0x06,0x00,0x43,0x00,0x1E,0xF9,0xF4},       
  8   // include crc's 2bytes  
};
MbCommand_Structure wmdStop = 
{
  {0x00,0x06,0x00,0x40,0x00,0x00,0x89,0xCF}, 
  8   // include crc's 2bytes
};
MbCommand_Structure wmdAbsPosFwd =   // abosolute position mode: forward
{
 //addr, cmd, reg addr,    reg num,   bytes, speed     , abs mode,  target position 4bytes,    CRC
  {0x00,0x10,0x00,0x44,0x00,0x04,0x08,0x00,0x32,0x00,0x00,0x00,0x00,0x00,0x64,0x95,0xA2},
  17
};
MbCommand_Structure wmdAbsPosBkd = 
{
  {0x00,0x10,0x00,0x44,0x00,0x04,0x08,0x00,0x32,0x00,0x00,0xFF,0xFF,0xFF,0x9C,0xD5,0xF4},
  17
};
MbCommand_Structure wmdRelPosFwd =   // relative position mode: forward
{
 //addr, cmd, reg addr,    reg num,   bytes, speed     , rel mode,  target position 4bytes,    CRC
  {0x00,0x10,0x00,0x44,0x00,0x04,0x08,0x00,0x32,0x00,0x01,0x00,0x00,0x00,0x64,0xA8,0x62},
  17
};
MbCommand_Structure wmdRelPosBkd = 
{
  {0x00,0x10,0x00,0x44,0x00,0x04,0x08,0x00,0x32,0x00,0x01,0xFF,0xFF,0xFF,0x9C,0xE8,0x34},
  17
};  
// smd=steering motors' driver
MbCommand_Structure steerNop =      // incase first byte isn't send out after MCU initialize 
{
  {0x02,0x03,0x60,0x41,0x00,0x01,0xCA,0x2D},
  8
};
MbCommand_Structure steerInitial = 
{
  {0x00,0x06,0x60,0x40,0x00,0x4F,0xD6,0x3B},
  8
};
MbCommand_Structure steerPosMode = 
{
  {0x00,0x06,0x60,0x60,0x00,0x01,0x57,0xC5},
  8
};
MbCommand_Structure steerSpeedAccDec = 
{
 //addr,func, reg addr,    reg num,   bytes, target speed,      acc,       dec,       CRC
  {0x00,0x10,0x60,0x81,0x00,0x04,0x08,0x00,0x00,0x01,0xF4,0x03,0xE8,0x03,0xE8,0x39,0xAD},
  17
};
MbCommand_Structure steerPosFwd =  // position mode : forward, fixed at 96pulse per movement 
{
 //addr,func, reg addr,    reg num,   bytes, relative position,     CRC
  {0x00,0x10,0x60,0x7A,0x00,0x02,0x04,0x00,0x00,0x0A,0x00,0xDE,0xAA},
  13 
};
MbCommand_Structure steerPosBkd = 
{
  {0x00,0x10,0x60,0x7A,0x00,0x02,0x04,0xFF,0xFF,0xF6,0x00,0x9F,0x8E},
  13   
};
MbCommand_Structure steerStart = 
{
  {0x00,0x06,0x60,0x40,0x00,0x5F,0xD7,0xF7},
  8
};
MbCommand_Structure steerStop = 
{
  {0x00,0x06,0x60,0x40,0x01,0x4F,0xD7,0xAB},
  8
};




/* Private function prototypes -----------------------------------------------*/
/*
 * brief: send command to motor driver.
 * pcmd: pointer to  command array,
 * return: response correct or wrong
*/
uint8_t mainMbSendCmd(MbCommand_Structure *pcmd)
{
  int i = 0;

  ubRxIndex = 0;
  for(i=0;i<BUFFERSIZE;i++)
    aRxBuffer[i] = 0;
  USARTx_TX_EN;
  HAL_Delay(0);
  for(i=0;i<pcmd->len;i++)
  {
    USART_SendData(USARTx,pcmd->cmd[i]);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){};
  }
  HAL_Delay(0);
  USARTx_RX_EN;
  HAL_Delay(5);
  return ubRxError;
}

static void TP_Config(void);

/* Private functions ---------------------------------------------------------*/
#if(USE_PRINTF_DEBUG)
  #include <stdio.h>
    /** 
     * @brief private impleted printf function,this would save a lot
     *            of rom&ram space compare to the built in printf
     *            function in the C newlib.Carefully use it inside a
     *            interrupt service routine.
     * @param fmt
     * 
     * @return int
     */
    int fputc(int ch, FILE *f)
    {
        return(ITM_SendChar(ch));
    }
#endif

/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK | USARTx_DE_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USARTx_DE_PIN; 		
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(USARTx_DE_GPIO_PORT, &GPIO_InitStructure);	
  
  USARTx_TX_EN;
  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follows:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - two Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */ 
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_2;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

/**************************************************************
  * @brief  Steering motor control,
  *        Steering have three methods: 1,one motor stop,another move;
  *        2,one forward,another backward; 3,one faster,another slower;
  *        Here use method 3.
  * @param  None
  * @retval None
 ***************************************************************/
void mainSteerControl(void)
{
  uint16_t usCRC16 = 0x0000;
  uint16_t steerSpeedFast = 30;
  
  // current car is in right steering state,change left two wheels speed faster
  if(curTurnPosition > MIN_STEER_POSITION_POS)               
  {
    if(flagWheelRun == LAST_MOVE_FORWARD)
    {
      steerSpeedFast = curWheelFwdSpeed * 1.2;
      if((curWheelSpeedLF != steerSpeedFast) || (curWheelSpeedLR != steerSpeedFast))
      {
        curWheelSpeedLF = steerSpeedFast;
        wmdSpeedLF.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedLF.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedLF.cmd[0]),wmdSpeedLF.len-2);
        wmdSpeedLF.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedLF.cmd[7] = (uint8_t)(usCRC16 >> 8);   
 
        curWheelSpeedLR = steerSpeedFast;
        wmdSpeedLR.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedLR.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedLR.cmd[0]),wmdSpeedLR.len-2);
        wmdSpeedLR.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedLR.cmd[7] = (uint8_t)(usCRC16 >> 8);   

        // avoid not synchronize of left-front and left-rear wheel
        mainMbSendCmd(&wmdSpeedLF);
        HAL_Delay(1);  
        mainMbSendCmd(&wmdSpeedLR);
        HAL_Delay(20);          
      }     
    }
    else if(flagWheelRun == LAST_MOVE_BACKWARD)
    {
      steerSpeedFast = curWheelBkdSpeed - (65536 - curWheelBkdSpeed) * 0.2;
      if((curWheelSpeedLF != steerSpeedFast) || (curWheelSpeedLR != steerSpeedFast))
      {
        curWheelSpeedLF = steerSpeedFast;
        wmdSpeedLF.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedLF.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedLF.cmd[0]),wmdSpeedLF.len-2);
        wmdSpeedLF.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedLF.cmd[7] = (uint8_t)(usCRC16 >> 8);   
 
        curWheelSpeedLR = steerSpeedFast;
        wmdSpeedLR.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedLR.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedLR.cmd[0]),wmdSpeedLR.len-2);
        wmdSpeedLR.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedLR.cmd[7] = (uint8_t)(usCRC16 >> 8);   

        // avoid not synchronize of left-front and left-rear wheel
        mainMbSendCmd(&wmdSpeedLF);
        HAL_Delay(1);  
        mainMbSendCmd(&wmdSpeedLR);
        HAL_Delay(20);          
      }    
    }   
  }
  // current car is in left steering state,change left two wheels speed faster
  else if(curTurnPosition < MIN_STEER_POSITION_NEG)               
  {
    if(flagWheelRun == LAST_MOVE_FORWARD)
    {
      steerSpeedFast = curWheelFwdSpeed * 1.2;
      if((curWheelSpeedRF != steerSpeedFast) || (curWheelSpeedRR != steerSpeedFast))
      {
        curWheelSpeedRF = steerSpeedFast;
        wmdSpeedRF.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedRF.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedRF.cmd[0]),wmdSpeedRF.len-2);
        wmdSpeedRF.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedRF.cmd[7] = (uint8_t)(usCRC16 >> 8);   
 
        curWheelSpeedRR = steerSpeedFast;
        wmdSpeedRR.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedRR.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedRR.cmd[0]),wmdSpeedRR.len-2);
        wmdSpeedRR.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedRR.cmd[7] = (uint8_t)(usCRC16 >> 8);   

        // avoid not synchronize of left-front and left-rear wheel
        mainMbSendCmd(&wmdSpeedRF);
        HAL_Delay(1);  
        mainMbSendCmd(&wmdSpeedRR);
        HAL_Delay(20);          
      }     
    }
    else if(flagWheelRun == LAST_MOVE_BACKWARD)
    {
      steerSpeedFast = curWheelBkdSpeed - (65536 - curWheelBkdSpeed) * 0.2;
      if((curWheelSpeedRF != steerSpeedFast) || (curWheelSpeedRR != steerSpeedFast))
      {
        curWheelSpeedRF = steerSpeedFast;
        wmdSpeedRF.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedRF.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedRF.cmd[0]),wmdSpeedRF.len-2);
        wmdSpeedRF.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedRF.cmd[7] = (uint8_t)(usCRC16 >> 8);   
 
        curWheelSpeedRR = steerSpeedFast;
        wmdSpeedRR.cmd[4] = steerSpeedFast >> 8;
        wmdSpeedRR.cmd[5] = steerSpeedFast & 0x00FF;
        usCRC16 = usMBCRC16(&(wmdSpeedRR.cmd[0]),wmdSpeedRR.len-2);
        wmdSpeedRR.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
        wmdSpeedRR.cmd[7] = (uint8_t)(usCRC16 >> 8);   

        // avoid not synchronize of left-front and left-rear wheel
        mainMbSendCmd(&wmdSpeedRF);
        HAL_Delay(1);  
        mainMbSendCmd(&wmdSpeedRR);
        HAL_Delay(20);           
      }    
    }   
  }

}
void mainRefreshWheelSpeed(MbCommand_Structure *pcmd)
{
  uint16_t usCRC16 = 0x0000;
  
  wmdSpeedLF.cmd[4] = pcmd->cmd[4];
  wmdSpeedLF.cmd[5] = pcmd->cmd[5];
  usCRC16 = usMBCRC16(&(wmdSpeedLF.cmd[0]),wmdSpeedLF.len-2);
  wmdSpeedLF.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
  wmdSpeedLF.cmd[7] = (uint8_t)(usCRC16 >> 8);     
  wmdSpeedLR.cmd[4] = pcmd->cmd[4];
  wmdSpeedLR.cmd[5] = pcmd->cmd[5];
  usCRC16 = usMBCRC16(&(wmdSpeedLR.cmd[0]),wmdSpeedLR.len-2);
  wmdSpeedLR.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
  wmdSpeedLR.cmd[7] = (uint8_t)(usCRC16 >> 8);    
  wmdSpeedRF.cmd[4] = pcmd->cmd[4];
  wmdSpeedRF.cmd[5] = pcmd->cmd[5];
  usCRC16 = usMBCRC16(&(wmdSpeedRF.cmd[0]),wmdSpeedRF.len-2);
  wmdSpeedRF.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
  wmdSpeedRF.cmd[7] = (uint8_t)(usCRC16 >> 8);    
  wmdSpeedRR.cmd[4] = pcmd->cmd[4];
  wmdSpeedRR.cmd[5] = pcmd->cmd[5];
  usCRC16 = usMBCRC16(&(wmdSpeedRR.cmd[0]),wmdSpeedRR.len-2);
  wmdSpeedRR.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
  wmdSpeedRR.cmd[7] = (uint8_t)(usCRC16 >> 8);    


}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  uint16_t usCRC16 = 0x0000;
  int      stateInit = 0;
  int      initializing = 1;
  uint16_t linenum = 0;
  static TP_STATE* TP_State; 
  
 /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files before to branch to application main.
       To reconfigure the default setting of SystemInit() function, 
       refer to system_stm32f4xx.c file */
  HAL_Init();     

  /* LCD initialization */
  LCD_Init();
  
  /* LCD Layer initialization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
  
  /* Touch Panel configuration */
  TP_Config();

  /*  PS2 joystick remote controller initialize  ----*/
  PS2_Init();			   

  /* Tamper Button Configuration -------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDOff(LED4);

  /* USART configuration and initialize ----------*/
  USART_Config();
  /* Enable the Rx buffer not empty interrupt */
  USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
  USART_ClearITPendingBit(USARTx, USART_IT_TC);
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); 
  HAL_Delay(100);
  STM_EVAL_LEDOn(LED4);   

  /* Wait until Tamper Button is pressed */
  //while (STM_EVAL_PBGetState(BUTTON_USER)==RESET);  

  while(PS2_GetKey() != PSB_START);
  /* Automatic Initialization */
  while(initializing)
  {
    /*******************      50systick(500ms) scan cycle task ***********/
    if(1 == SYSTICK_TIMER50)        
    {
      switch(stateInit)
      {
        case 0:   // send nop command incase first byte is droped by UART
            HAL_Delay(10);
            mainMbSendCmd(&steerNop);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED4);
            stateInit = 1;
        break;
        case 1:   /* Initialize steering motor driver    */
            HAL_Delay(10);
            mainMbSendCmd(&steerInitial);
            HAL_Delay(100);
            STM_EVAL_LEDOn(LED4);
            stateInit = 2;
        break;
        case 2:
            HAL_Delay(10);
            mainMbSendCmd(&steerPosMode);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED4);
            stateInit = 3;
        break;
        case 3:
            HAL_Delay(10);
            mainMbSendCmd(&steerSpeedAccDec);
            HAL_Delay(100);
            STM_EVAL_LEDOn(LED4);
            stateInit = 4;
        break;
        case 4:
            HAL_Delay(10);
            mainMbSendCmd(&steerPosFwd);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED4);
            stateInit = 5;
        break;
        case 5:
          STM_EVAL_LEDOn(LED3);
          stateInit = 0;
          initializing = 0;
        break;
        default:

        break;
      }
      
      SYSTICK_TIMER50 = 0;
    }
  }

  /* Infinite loop */
  while (1)
  {
    
    /*******************      10systick(100ms) scan cycle task ***********/
    if(1 == SYSTICK_TIMER10)        
    {
      //if(stopPS2 == 0)
      {
        curKey = PS2_KeyScan();
        curKeyValue = curKey & PS2_KEY_VALUE_MASK;
        curKeyState = curKey & PS2_KEY_STATE_MASK;
        lastKeyValue = curKeyValue;
      }
      if(curKey == PSB_NONE)        // stop when lost connection/no button pressed of PS2 joystick
      {
        if(ps2KeyTimeout++ > MAX_PS2_TIMEOUT)
        {
          ps2KeyTimeout = 0;
          if((flagWheelRun == LAST_MOVE_FORWARD) || (flagWheelRun == LAST_MOVE_BACKWARD))
          {
            mainMbSendCmd(&wmdStop);
            HAL_Delay(10);
            flagWheelRun = LAST_MOVE_STOP;
          }
        }
      }
      else
    	{
        #if(USE_PRINTF_DEBUG)
          printf("  \n   %d  is  pressed.  \n",curKey);
        #endif
        ps2KeyTimeout = 0;

          switch(curKeyValue)
          {
            case PSB_UP:                    // forward
              mainMbSendCmd(&wmdSpeedFwd);
              flagWheelRun = LAST_MOVE_FORWARD;
              mainRefreshWheelSpeed(&wmdSpeedFwd);
              
            break;
            case PSB_DOWN:                  // backward
              mainMbSendCmd(&wmdSpeedBkd);
              flagWheelRun = LAST_MOVE_BACKWARD;
              mainRefreshWheelSpeed(&wmdSpeedBkd);
            break;
            case PSB_RIGHT:                 // Turn right
              if((curTurnPosition > MAX_TURN_POSITION) || (curKeyState == PS2_KEY_HOLD_MASK))
                break;
              else if((curKeyState == PS2_KEY_SHORT_MASK) || (curKeyState == PS2_KEY_LONG_MASK))
              {
                mainMbSendCmd(&steerPosFwd);
                HAL_Delay(20);
                mainMbSendCmd(&steerStart);
                HAL_Delay(20);
                mainMbSendCmd(&steerInitial);
                HAL_Delay(20);  
                curTurnPosition += 0x0A00;        
                //mainSteerControl();
              }
            break;
            case PSB_LEFT:                  // Turn left
              avoidButtonTwicePress++;
              if((curTurnPosition < MIN_TURN_POSITION) || (curKeyState == PS2_KEY_HOLD_MASK))
                break;
              else if((curKeyState == PS2_KEY_SHORT_MASK) || (curKeyState == PS2_KEY_LONG_MASK))
              {           
                mainMbSendCmd(&steerPosBkd);
                HAL_Delay(20);
                mainMbSendCmd(&steerStart);
                HAL_Delay(20);
                mainMbSendCmd(&steerInitial); 
                HAL_Delay(20);
                curTurnPosition -= 0x0A00;
                //mainSteerControl();
              }
            break;
            case PSB_TRIANGLE:              // acceleration
              curWheelFwdSpeed = (wmdSpeedFwd.cmd[4] << 8) | wmdSpeedFwd.cmd[5];
              if(curWheelFwdSpeed < MAX_WHEEL_SPEED)
              {
                 curWheelFwdSpeed += 50;
              }
              wmdSpeedFwd.cmd[4] = curWheelFwdSpeed >> 8;
              wmdSpeedFwd.cmd[5] = curWheelFwdSpeed & 0x00FF;
              usCRC16 = usMBCRC16(&(wmdSpeedFwd.cmd[0]),wmdSpeedFwd.len-2);
              wmdSpeedFwd.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
              wmdSpeedFwd.cmd[7] = (uint8_t)(usCRC16 >> 8);

              curWheelBkdSpeed = (wmdSpeedBkd.cmd[4] << 8) | wmdSpeedBkd.cmd[5];
              if(curWheelBkdSpeed > MIN_WHEEL_SPEED)
              {
                 curWheelBkdSpeed -= 50;
              }
              wmdSpeedBkd.cmd[4] = curWheelBkdSpeed >> 8;
              wmdSpeedBkd.cmd[5] = curWheelBkdSpeed & 0x00FF;
              usCRC16 = usMBCRC16(&(wmdSpeedBkd.cmd[0]),wmdSpeedBkd.len-2);
              wmdSpeedBkd.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
              wmdSpeedBkd.cmd[7] = (uint8_t)(usCRC16 >> 8);
              // run in new speed
              if(flagWheelRun == LAST_MOVE_FORWARD)
              {
                mainMbSendCmd(&wmdSpeedFwd);
                mainRefreshWheelSpeed(&wmdSpeedFwd);
              }
              else if(flagWheelRun == LAST_MOVE_BACKWARD)
              {
                mainMbSendCmd(&wmdSpeedBkd);          
                mainRefreshWheelSpeed(&wmdSpeedBkd);
              }

            break;
            case PSB_CROSS:                 // Decelleration
             curWheelFwdSpeed = (wmdSpeedFwd.cmd[4] << 8) | wmdSpeedFwd.cmd[5];
              if(curWheelFwdSpeed > 101)
              {
                 curWheelFwdSpeed -= 50;
              }
              wmdSpeedFwd.cmd[4] = curWheelFwdSpeed >> 8;
              wmdSpeedFwd.cmd[5] = curWheelFwdSpeed & 0x00FF;
              usCRC16 = usMBCRC16(&(wmdSpeedFwd.cmd[0]),wmdSpeedFwd.len-2);
              wmdSpeedFwd.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
              wmdSpeedFwd.cmd[7] = (uint8_t)(usCRC16 >> 8);

              curWheelBkdSpeed = (wmdSpeedBkd.cmd[4] << 8) | wmdSpeedBkd.cmd[5];
              if(curWheelBkdSpeed < 65435)
              {
                 curWheelBkdSpeed += 50;
              }
              wmdSpeedBkd.cmd[4] = curWheelBkdSpeed >> 8;
              wmdSpeedBkd.cmd[5] = curWheelBkdSpeed & 0x00FF;
              usCRC16 = usMBCRC16(&(wmdSpeedBkd.cmd[0]),wmdSpeedBkd.len-2);
              wmdSpeedBkd.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
              wmdSpeedBkd.cmd[7] = (uint8_t)(usCRC16 >> 8);
              // run in new speed
              if(flagWheelRun == LAST_MOVE_FORWARD)
              {
                mainMbSendCmd(&wmdSpeedFwd);
                mainRefreshWheelSpeed(&wmdSpeedFwd);
              }
              else if(flagWheelRun == LAST_MOVE_BACKWARD)
              {
                mainMbSendCmd(&wmdSpeedBkd);          
                mainRefreshWheelSpeed(&wmdSpeedBkd);
              }        

            break;
            case PSB_SQUARE:                // Stop
              mainMbSendCmd(&wmdStop);
              flagWheelRun = LAST_MOVE_STOP;
              HAL_Delay(10);
              mainMbSendCmd(&steerStop);
              HAL_Delay(20);
            break;
            case PSB_CIRCLE:                // Home
              wmdSpeedFwd.cmd[4] = 0x00;
              wmdSpeedFwd.cmd[5] = 0x32;
              usCRC16 = usMBCRC16(&(wmdSpeedFwd.cmd[0]),wmdSpeedFwd.len-2);
              wmdSpeedFwd.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
              wmdSpeedFwd.cmd[7] = (uint8_t)(usCRC16 >> 8);

              wmdSpeedBkd.cmd[4] = 0xFF;
              wmdSpeedBkd.cmd[5] = 0xCE;
              usCRC16 = usMBCRC16(&(wmdSpeedBkd.cmd[0]),wmdSpeedBkd.len-2);
              wmdSpeedBkd.cmd[6] = (uint8_t)(usCRC16 & 0xFF);
              wmdSpeedBkd.cmd[7] = (uint8_t)(usCRC16 >> 8);
              
            break;
            default:

            break;
          }
          oldKeyValue = curKeyValue; 
        

      
      SYSTICK_TIMER10 = 0;
    }

    /*******************      50systick(500ms) scan cycle task ***********/
    if(1 == SYSTICK_TIMER50)        
    {
      TP_State = IOE_TP_GetState();
      
      if((TP_State->TouchDetected) && ((TP_State->Y < 245) && (TP_State->Y >= 3)))
      {
        if((TP_State->X >= 237) || (TP_State->X < 3))
        {}     
        else
        {
          LCD_DrawFullCircle(TP_State->X, TP_State->Y, 3);
        }
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 280) && (TP_State->Y >= 250) && (TP_State->X >= 5) && (TP_State->X <= 35))
      {
        LCD_SetTextColor(LCD_COLOR_BLUE2);
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 280) && (TP_State->Y >= 250) && (TP_State->X >= 40) && (TP_State->X <= 70))
      {
        LCD_SetTextColor(LCD_COLOR_CYAN); 
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 280) && (TP_State->Y >= 250) && (TP_State->X >= 75) && (TP_State->X <= 105))
      {
        LCD_SetTextColor(LCD_COLOR_YELLOW); 
      }      
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 318) && (TP_State->Y >= 288) && (TP_State->X >= 5) && (TP_State->X <= 35))
      {
        LCD_SetTextColor(LCD_COLOR_RED);
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 318) && (TP_State->Y >= 288) && (TP_State->X >= 40) && (TP_State->X <= 70))
      {
        LCD_SetTextColor(LCD_COLOR_BLUE); 
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 318) && (TP_State->Y >= 288) && (TP_State->X >= 75) && (TP_State->X <= 105))
      {
        LCD_SetTextColor(LCD_COLOR_GREEN); 
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 318) && (TP_State->Y >= 288) && (TP_State->X >= 110) && (TP_State->X <= 140))
      {
        LCD_SetTextColor(LCD_COLOR_BLACK); 
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 318) && (TP_State->Y >= 288) && (TP_State->X >= 145) && (TP_State->X <= 175))
      {
        LCD_SetTextColor(LCD_COLOR_MAGENTA); 
      }
      else if ((TP_State->TouchDetected) && (TP_State->Y <= 318) && (TP_State->Y >= 270) && (TP_State->X >= 180) && (TP_State->X <= 230))
      {
        LCD_SetFont(&Font8x8);
        for(linenum = 0; linenum < 31; linenum++)
        {
          LCD_ClearLine(LINE(linenum));
        }
      }
      else
      {
      }
      
      SYSTICK_TIMER50 = 0;
    }

   /**********************  None cycle task  *******************/


  
      
}
  }
  
}
/**
* @brief  Configure the IO Expander and the Touch Panel.
* @param  None
* @retval None
*/
static void TP_Config(void)
{
  /* Clear the LCD */ 
  LCD_Clear(LCD_COLOR_WHITE);
  
  /* Configure the IO Expander */
  if (IOE_Config() == IOE_OK)
  {   
    LCD_SetFont(&Font12x12);
    LCD_DisplayStringLine(LINE(1), (uint8_t*)"DAYE Robot Inc.");
    LCD_DisplayStringLine(LINE(2), (uint8_t*)"4Wheel Car Platform");
    LCD_SetTextColor(LCD_COLOR_BLUE2); 
    LCD_DrawFullRect(5, 250, 30, 30);
    LCD_SetTextColor(LCD_COLOR_CYAN); 
    LCD_DrawFullRect(40, 250, 30, 30);
    LCD_SetTextColor(LCD_COLOR_YELLOW); 
    LCD_DrawFullRect(75, 250, 30, 30);
    LCD_SetTextColor(LCD_COLOR_RED); 
    LCD_DrawFullRect(5, 288, 30, 30);
    LCD_SetTextColor(LCD_COLOR_BLUE); 
    LCD_DrawFullRect(40, 288, 30, 30);
    LCD_SetTextColor(LCD_COLOR_GREEN); 
    LCD_DrawFullRect(75, 288, 30, 30);
    LCD_SetTextColor(LCD_COLOR_MAGENTA); 
    LCD_DrawFullRect(145, 288, 30, 30);
    LCD_SetTextColor(LCD_COLOR_BLACK); 
    LCD_DrawFullRect(110, 288, 30, 30);
    LCD_DrawRect(180, 270, 48, 50);
    LCD_SetFont(&Font16x24);
    LCD_DisplayChar(LCD_LINE_12, 195, 0x43);
    LCD_DrawLine(0, 248, 240, LCD_DIR_HORIZONTAL);
    LCD_DrawLine(0, 284, 180, LCD_DIR_HORIZONTAL);
    LCD_DrawLine(1, 248, 71, LCD_DIR_VERTICAL);
    LCD_DrawLine(37, 248, 71, LCD_DIR_VERTICAL);
    LCD_DrawLine(72, 248, 71, LCD_DIR_VERTICAL);
    LCD_DrawLine(107, 248, 71, LCD_DIR_VERTICAL);
    LCD_DrawLine(142, 284, 36, LCD_DIR_VERTICAL);
    LCD_DrawLine(0, 319, 240, LCD_DIR_HORIZONTAL);   
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
