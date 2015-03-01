/*
  Copyright (c) 2014 HippoDevices.  All right reserved.
  Copyright (c) 2014 MakerLab.me & Andy Sze(andy.sze.mail@gmail.com)  All right reserved.
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

/*
 * Hippo-ADK Board pin           |  PORT  | Label
 * ---------------------------------+--------+-------
 *   0       Vin                          
 *   1       5V                           
 *   2       GND                        
 *   3       TIM1_CH1                 |  PA8  |pwm
 *   4       TIM8_CH3                 |  PC8  |pwm
 *   5       TIM8_CH4                 |  PC9  |pwm
 *   6                                       |  PC10|
 *   7                                       |  PC11|
 *   8       USART5_TX               |  PC12|
 *   9       USART5_RX               |  PD2 |
 *  10                                      |  PB5  |
 *  11                                      |  PB6  |
 *  12      TIM4_CH2                 |  PB7  | 
 *  13      TIM4_CH3                 |  PB8  |
 *  14      TIM4_CH4                 |  PB9  | 
 *  15                                      |  PC0  | A0
 *  16                                      |  PC1  | A1
 *  17                                      |  PC2  | A2
 *  18                                      |  PC3  | A3
 *  19       Reset
 *  20       3.3V
 *  21       GND
 *  22       TIM3_CH2,USART6_RX|  PC7  |pwm
 *  23       TIM3_CH1USART6_TX |  PC6  |pwm
 *  24       TIM12_CH2                |  PB15 |pwm 
 *  25       TIM12_CH1                |  PB14 |pwm  
 *  26                                       |  PB13 |
 *  27                                       |  PB12 |
 *  28       TIM2_CH4                 |  PB11 |pwm 
 *  29       TIM2_CH3                 |  PB10 |pwm 
 *  30       TIM3_CH4                 |  PB1  | pwm,A4
 *  31       TIM3_CH3                 |  PB0  | pwm,A5
 *  32                                       |  PA7  | A6
 *  33                                       |  PA6  | A7
 *  34                                       |  PA5  | A8
 *  35                                       |  PA4  | A9
 *  36       TIM5_CH4,USART2_RX|  PA3  | pwm,A10
 *  37       TIM5_CH3,USART2_TX |  PA2  | pwm,A11
 *  38       TIM5_CH2,UART4_RX   |  PA1  | pwm,A12
 *  39       TIM5_CH1,UART4_TX   |  PA0  | pwm,A13
 *  40                              |  PC4  |RED
 *  41                              |  PC5  |GREEN
 *  42                              |  PE10  |
 */

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Pins descriptions
 */
 
extern const PinDescription g_APinDescription[]=
{
// 0-2
{},
{},
{},
// motor 3 EN   3-5
{GPIOA, GPIO_Pin_8, RCC_AHB1Periph_GPIOA, NONE, TIM1, TIM_Channel_1, GPIO_PinSource8, GPIO_AF_TIM1},  // I2C3_SCL ?
{GPIOC, GPIO_Pin_8, RCC_AHB1Periph_GPIOC, NONE, TIM8, TIM_Channel_3, GPIO_PinSource8, GPIO_AF_TIM8},
{GPIOC, GPIO_Pin_9, RCC_AHB1Periph_GPIOC, NONE, TIM8, TIM_Channel_4, GPIO_PinSource9, GPIO_AF_TIM8},  //I2C3_SDA ?
  

// 6(USART3_TX / SPI3_SCK), 7(USARt3_RX / SPI3_MISO)
{GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC, NONE, NULL, NONE},
{GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC, NONE, NULL, NONE},

// 8(USART5_TX / SPI3_MOSI), 9(USART5_RX)
{GPIOC, GPIO_Pin_12, RCC_AHB1Periph_GPIOC, NONE, NULL, NONE, GPIO_PinSource12, GPIO_AF_UART5},
{GPIOD, GPIO_Pin_2,  RCC_AHB1Periph_GPIOD, NONE, NULL, NONE, GPIO_PinSource2,  GPIO_AF_UART5},

// 10(USART1_TX),11(USART1_RX)
{GPIOB, GPIO_Pin_5, RCC_AHB1Periph_GPIOB, NONE, NULL, NONE,          NONE,            NONE        },
{GPIOB, GPIO_Pin_6, RCC_AHB1Periph_GPIOB, NONE, TIM4, TIM_Channel_1, GPIO_PinSource6, GPIO_AF_TIM4},   // I2C1_SCL ?

// 12, 13(CAN1_RX)   14(CAN1-TX)
{GPIOB, GPIO_Pin_7, RCC_AHB1Periph_GPIOB, NONE, TIM4, TIM_Channel_2, GPIO_PinSource7, GPIO_AF_TIM4},   // I2C1_SDA ?
{GPIOB, GPIO_Pin_8, RCC_AHB1Periph_GPIOB, NONE, TIM4, TIM_Channel_3, GPIO_PinSource8, GPIO_AF_TIM4},  // 
{GPIOB, GPIO_Pin_9, RCC_AHB1Periph_GPIOB, NONE, TIM4, TIM_Channel_4, GPIO_PinSource9, GPIO_AF_TIM4},  //

// 15/16/17/18 /19     ADC(10~ 13)   
{GPIOC, GPIO_Pin_0, RCC_AHB1Periph_GPIOC, ADC_Channel_10, NULL, NONE, NONE, NONE},
{GPIOC, GPIO_Pin_1, RCC_AHB1Periph_GPIOC, ADC_Channel_11, NULL, NONE, NONE, NONE},
{GPIOC, GPIO_Pin_2, RCC_AHB1Periph_GPIOC, ADC_Channel_12, NULL, NONE, NONE, NONE},
{GPIOC, GPIO_Pin_3, RCC_AHB1Periph_GPIOC, ADC_Channel_13, NULL, NONE, NONE, NONE},
{},

// 20/21
{},
{},

//	22(USART6_RX), 23(USART6_TX)
{GPIOC, GPIO_Pin_7, RCC_AHB1Periph_GPIOC, NONE, TIM3, TIM_Channel_2, GPIO_PinSource7, GPIO_AF_TIM3},//pwm
{GPIOC, GPIO_Pin_6, RCC_AHB1Periph_GPIOC, NONE, TIM3, TIM_Channel_1, GPIO_PinSource6, GPIO_AF_TIM3},//pwm

// 24(SPI2_MOSI / TIM12_CH2),25(SPI_MISO / TIM12_CH1)
{GPIOB, GPIO_Pin_15, RCC_AHB1Periph_GPIOB, NONE, TIM12, TIM_Channel_2, GPIO_PinSource15, GPIO_AF_TIM12},
{GPIOB, GPIO_Pin_14, RCC_AHB1Periph_GPIOB, NONE, TIM12, TIM_Channel_1, GPIO_PinSource14, GPIO_AF_TIM12},

//26(SPI2_SCK / CAN2_TX),27(SPI2_NSS / CAN2_RX)
{GPIOB, GPIO_Pin_13, RCC_AHB1Periph_GPIOB, NONE, NULL, NONE, NONE, NONE},
{GPIOB, GPIO_Pin_12, RCC_AHB1Periph_GPIOB, NONE, NULL, NONE, NONE, NONE},

//28(I2C2_SDA / TIM2_CH4),29(I2C2_SCL / TIM2_CH3)
{GPIOB, GPIO_Pin_11, RCC_AHB1Periph_GPIOB, NONE, TIM2, TIM_Channel_4, GPIO_PinSource11,	GPIO_AF_TIM2},
{GPIOB, GPIO_Pin_10, RCC_AHB1Periph_GPIOB, NONE, TIM2, TIM_Channel_3, GPIO_PinSource10,	GPIO_AF_TIM2},

//30-31  motor1 EN , motor2 EN
{GPIOB, GPIO_Pin_1, RCC_AHB1Periph_GPIOB, ADC_Channel_9, TIM3,TIM_Channel_4, GPIO_PinSource1, GPIO_AF_TIM3},//motor1 en
{GPIOB, GPIO_Pin_0, RCC_AHB1Periph_GPIOB, ADC_Channel_8, TIM3,TIM_Channel_3, GPIO_PinSource0, GPIO_AF_TIM3},//motor2 en

// 32/33/34/35   SPI1_MOSI, SPI1_MISO, SPI1_SCK , SPI1_NSS
{GPIOA, GPIO_Pin_7, RCC_AHB1Periph_GPIOA, ADC_Channel_7, NULL, NONE, GPIO_PinSource7, GPIO_AF_SPI1},
{GPIOA, GPIO_Pin_6, RCC_AHB1Periph_GPIOA, ADC_Channel_6, NULL, NONE, GPIO_PinSource6, GPIO_AF_SPI1},
{GPIOA, GPIO_Pin_5, RCC_AHB1Periph_GPIOA, ADC_Channel_5, NULL, NONE, GPIO_PinSource5, GPIO_AF_SPI1},
{GPIOA, GPIO_Pin_4, RCC_AHB1Periph_GPIOA, ADC_Channel_4, NULL, NONE, GPIO_PinSource4, GPIO_AF_SPI1},

//36/37/38/39  USART2_RX,USART2_TX,UART4_RX,UART4_TX
{GPIOA, GPIO_Pin_3, RCC_AHB1Periph_GPIOA, NONE, TIM5,TIM_Channel_4, GPIO_PinSource3, GPIO_AF_TIM5},//pwm
{GPIOA, GPIO_Pin_2, RCC_AHB1Periph_GPIOA, NONE, TIM5,TIM_Channel_3, GPIO_PinSource2, GPIO_AF_TIM5},//pwm
{GPIOA, GPIO_Pin_1, RCC_AHB1Periph_GPIOA, ADC_Channel_1, TIM5,TIM_Channel_2, GPIO_PinSource1, GPIO_AF_TIM5},//Encoder
{GPIOA, GPIO_Pin_0, RCC_AHB1Periph_GPIOA, ADC_Channel_0, TIM5,TIM_Channel_1, GPIO_PinSource0, GPIO_AF_TIM5},//Encoder

// 40/41  LED
{GPIOC, GPIO_Pin_4,  RCC_AHB1Periph_GPIOC, ADC_Channel_14, NULL, NONE }, // RED
{GPIOC, GPIO_Pin_5,  RCC_AHB1Periph_GPIOC, ADC_Channel_15, NULL, NONE }, // GREEN

};

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
//RingBuffer rx_buffer1;

//UARTClass Serial(UART, UART_IRQn, id_uart, &rx_buffer1);
//void serialEvent() __attribute__((weak));
//void serialEvent() { }
//
//// IT handlers
//void UART_Handler(void)
//{
//  //Serial.IrqHandler();
//}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
RingBuffer rx_buffer1;
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;
RingBuffer rx_buffer4;

USARTClass Serial(USART3, USART3_IRQn, id_serial, &rx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }
USARTClass Serial1(USART2, USART2_IRQn, id_serial1, &rx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
//USARTClass Serial2(USART2, USART2_IRQn, id_serial2, &rx_buffer3);
//void serialEvent2() __attribute__((weak));
//void serialEvent2() { }
//USARTClass Serial3(USART3, USART3_IRQn, id_serial3, &rx_buffer4);
//void serialEvent3() __attribute__((weak));
//void serialEvent3() { }

// IT handlers
void USART1_IRQHandler(void) 
{
  Serial.IrqHandler();//USART1 must be Serial,for usart flash programming.
}

void UART4_IRQHandler(void) 
{
  Serial1.IrqHandler();
}

void USART2_IRQHandler(void) 
{
  Serial1.IrqHandler();
}

//Serial.begin() mapped to USART3(PC10,PC11)
void USART3_IRQHandler(void) 
{
  Serial.IrqHandler();
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  //if (Serial2.available()) serialEvent2();
  //if (Serial3.available()) serialEvent3();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
  SystemInit();

  // Set Systick to 1ms interval, common to all SAM3 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
    while (true);
  }
	/* Configure the SysTick Handler Priority: Preemption priority and subpriority */
	NVIC_SetPriority(SysTick_IRQn, 15);	

  // Disable watchdog
  //WDT_Disable(WDT);

  // Initialize C library
  __libc_init_array();

  // default 13pin led will off.
  pinMode(13,OUTPUT);
  digitalWrite(13, LOW);

  /*
  // Enable parallel access on PIo output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  PIOC->PIO_OWER = 0xFFFFFFFF;
  PIOD->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port U(S)Art pins
  PIO_Configure(
    g_APinDescription[PINS_UART].pport,
    g_APinDescription[PINS_UART].ulpintype,
    g_APinDescription[PINS_UART].ulpin,
    g_APinDescription[PINS_UART].ulpinconfiguration);
  digitalWrite(0, HIGH); // Enable pullup for rx0
  PIO_Configure(
    g_APinDescription[PINS_USART0].pport,
    g_APinDescription[PINS_USART0].ulpintype,
    g_APinDescription[PINS_USART0].ulpin,
    g_APinDescription[PINS_USART0].ulpinconfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART1].pport,
    g_APinDescription[PINS_USART1].ulpintype,
    g_APinDescription[PINS_USART1].ulpin,
    g_APinDescription[PINS_USART1].ulpinconfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART3].pport,
    g_APinDescription[PINS_USART3].ulpintype,
    g_APinDescription[PINS_USART3].ulpin,
    g_APinDescription[PINS_USART3].ulpinconfiguration);

  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pport,
    g_APinDescription[PINS_USB].ulpintype,
    g_APinDescription[PINS_USB].ulpin,
    g_APinDescription[PINS_USB].ulpinconfiguration);

  // Initialize CAN pins
  PIO_Configure(
    g_APinDescription[PINS_CAN0].pport,
    g_APinDescription[PINS_CAN0].ulpintype,
    g_APinDescription[PINS_CAN0].ulpin,
    g_APinDescription[PINS_CAN0].ulpinconfiguration);
  PIO_Configure(
    g_APinDescription[PINS_CAN1].pport,
    g_APinDescription[PINS_CAN1].ulpintype,
    g_APinDescription[PINS_CAN1].ulPin,
    g_APinDescription[PINS_CAN1].ulPinConfiguration);
*/

  //disable JTAG-DP,release pin 29(PB3),30(PB4),23(PA15)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  ////remap Timer4
  //GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
  ////remap USART3
  //GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
  ////remap USART2
  //GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  ////remap CAN1,to PD0,PD1
  //GPIO_PinRemapConfig(GPIO_Remap2_CAN1,ENABLE);

  // Initialize Analog Controller

	ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE);

	// Enable ADC1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit();

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_6Cycles;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;

  ADC_CommonInit(&ADC_CommonInitStructure);  


  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;

  /* ADC1 regular channel 12 configuration ************************************/
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  //ADC_InitStructure.ADC_ExternalTrigConv = ;	
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC1 reset calibration register
	//ADC_ResetCalibration(ADC1);

	// Check the end of ADC1 reset calibration register
	while(ADC_GetSoftwareStartConvStatus(ADC1));

	// Start ADC1 calibration
	//ADC_StartCalibration(ADC1);

	// Check the end of ADC1 calibration
	//while(ADC_GetCalibrationStatus(ADC1));

  // Initialize analogOutput module
  analogOutputInit();

	/* Configure the NVIC Preemption Priority Bits */
	/* 4 bits for pre-emption priority(0-15 PreemptionPriority) and 0 bits for subpriority(0 SubPriority) */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

#ifdef __cplusplus
}
#endif

