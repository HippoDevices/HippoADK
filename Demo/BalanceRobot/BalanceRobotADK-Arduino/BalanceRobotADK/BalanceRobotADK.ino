#include <Firmata.h>
//#include "stm32f4xx.h"
#include "arduino.h"
//#include "stm32f4_discovery.h"
#include "usb_bsp.h"
#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_adk_core.h"
//#include "uart_debug.h"
//__ALIGN_BEGIN USB_OTG_CORE_HANDLE           USB_OTG_Core_dev __ALIGN_END ;
//__ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END ;

#define RED 40
#define GREEN 41

int pinAMotorRight = 11;
int pinBMotorRight = 12;

int pinAMotorLeft = 39;
int pinBMotorLeft = 38;


int encoderMotorLeft  = 0;
int encoderMotorRight  = 0;

int  MotorLeftPWM  = 0; 
int  MotorRightPWM = 0;

int timerCounter  = 0;

USB_OTG_CORE_HANDLE           USB_OTG_Core_dev;
USBH_HOST                     USB_Host;

  uint8_t msg[13];
  uint8_t commandPacket_sent[4]={0};
uint8_t msg5[5]={0xD0,0x00,0x00,0x00,0x00};
uint8_t msg8[5]={0xF1,0x00,0x00,0x00,0xFF};
uint8_t init1 = 0;

static boolean output = HIGH;

void setup()
{
  uint8_t msg1[5]={0xD1,0x07,0x01,0x00,0x00};  // digitalRead   A B C D
  uint8_t msg3[5]={0}; 
  uint8_t msg4[5]={0xED,0x7B,0x00,0x00,0x00};
  uint8_t msg5[5]={0xC2,0x01,0x00,0x00,0x00};  // analogRead   G  C2
  
  uint8_t msg6[5] = {0xC3,0x01,0x00,0x00,0x00};  // analogRead   H  C3
  uint16_t analogvalue = 0;
  uint8_t initTEST = 0;
  
  uint8_t flag=0;
 RCC_ClocksTypeDef RCC_Clocks;
  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);
  attachInterrupt(pinAMotorRight, encoderCounterMotorRight, CHANGE);  //
  attachInterrupt(pinAMotorLeft, encoderCounterMotorLeft, CHANGE);  //
  
  Serial.begin(9600);
  TIM5_Timing(300,3600);
  
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  
  USBH_Init(&USB_OTG_Core_dev,
            USB_OTG_FS_CORE_ID,
            &USB_Host,
            &USBH_ADK_cb,
            &USR_Callbacks);
            
  /* Init ADK Library */
   USBH_ADK_Init("Microchip Technology Inc.", "Basic Accessory Demo", "NWtel", "2.0", "http://www.microchip.com/android",  "N/A");
   
  while (1)
  {
      byte pin, analogPin;
      USBH_Process(&USB_OTG_Core_dev , &USB_Host);    
      if( USBH_ADK_getStatus() == ADK_IDLE) { 
        USBH_ADK_read(&USB_OTG_Core_dev, msg, sizeof(msg)); 
      	Decoding(msg);           // Decoding APP message from your phone
        pinMode(17,OUTPUT);    
        pinMode(18,OUTPUT);
        pinMode(5,OUTPUT);  
        pinMode(15,OUTPUT);    
        pinMode(16,OUTPUT);
        pinMode(4,OUTPUT); 
	/* MotorLeft */
	if(MotorLeftPWM<0)	 	// Forward
	{
    	    digitalWrite(17 ,HIGH );  // PC2
  	    digitalWrite(18,LOW);     // PC3
            analogWrite(5,abs(MotorLeftPWM));   // PC9			    
	}else{
  	  digitalWrite( 17,LOW );
  	  digitalWrite( 18,HIGH);
  	  analogWrite(5,abs(MotorLeftPWM));	
	}		
	/* MotorRight */
	if(MotorRightPWM<0)	    // Forward
	{
          digitalWrite(15 ,HIGH );  // PC0
  	  digitalWrite(16,LOW);     // PC1
          analogWrite(4,abs(MotorRightPWM));   // PC8
	}else{
  	  digitalWrite( 15,LOW );
  	  digitalWrite( 16,HIGH);
  	  analogWrite(4,abs(MotorRightPWM));
	}    
    }
    delay(1);
  }
}

void loop() {
}

/*
解码 手机通过USB发送给STM32F4的电机控制指令
*/

void Decoding(uint8_t *data)
{
  uint8_t Data0 = 0;
  uint8_t Data1 = 0;
  uint8_t Data2 = 0;
  uint8_t Data3 = 0;
  uint8_t Data4 = 0;
  
  Data0 = data[0];
  Data1 = data[1];
  Data2 = data[2];
  Data3 = data[3];
  Data4 = data[4];

  if(Data0 == 0x00 && Data4 == 0x08)	
   {
	MotorLeftPWM =  data[8];  
	MotorLeftPWM <<=8;
	MotorLeftPWM |= data[7];
	MotorLeftPWM <<=8;
	MotorLeftPWM |= data[6];
	MotorLeftPWM <<=8;
	MotorLeftPWM |= data[5];

	MotorRightPWM =  data[12];  
	MotorRightPWM <<=8;
	MotorRightPWM |= data[11];
	MotorRightPWM <<=8;
	MotorRightPWM |= data[10];
	MotorRightPWM <<=8;
	MotorRightPWM |= data[9];
   } 
}

//中断0调用函数
void encoderCounterMotorLeft()
{
  //if ((millis() - time) > 3) //防抖动处理
  //      count ++; 
 // time = millis();
  int pinAMotorLeftState = digitalRead(pinAMotorLeft);
  int pinBMotorLeftState = digitalRead(pinBMotorLeft);
  if(((pinAMotorLeftState == 0)&&(pinBMotorLeftState != 0))|((pinAMotorLeftState != 0)&&(pinBMotorLeftState == 0)))  // PA0下降沿且PA1=1 或 PA0上升沿且PA1=0  
  {
    encoderMotorLeft++;	
  }else{
    encoderMotorLeft--;
  } 
  //digitalWrite(RED, output);
  //output = !output;
}

//中断0调用函数
void encoderCounterMotorRight()
{
  int pinAMotorRightState = digitalRead(pinAMotorRight);
  int pinBMotorRightState = digitalRead(pinBMotorRight);
  if(((pinAMotorRightState == 0)&&(pinBMotorRightState != 0))|((pinAMotorRightState != 0)&&(pinBMotorRightState == 0)))  // PA0下降沿且PA1=1 或 PA0上升沿且PA1=0  
  {
    encoderMotorRight++;	
  }else{
    encoderMotorRight--;
  } 
}

void TIM5_IRQHandler(void)		 
{ 
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)  
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );
		if(encoderMotorLeft < 0)   
		      commandPacket_sent[0] =1 ;  	  // 符号位	   负
		else  commandPacket_sent[0] =0 ;
	    commandPacket_sent[1]=(uint8_t)(abs(encoderMotorLeft));	 //数据位
		 
		if(encoderMotorRight < 0)   
		      commandPacket_sent[2] =1 ;  	  // 符号位	   负
		else  commandPacket_sent[2] =0 ;
	    commandPacket_sent[3]=(uint8_t)(abs(encoderMotorRight));	 //数据位

                //USBH_ADK_write(&USB_OTG_Core_dev, msg8, sizeof(msg8));
		USBH_ADK_write(&USB_OTG_Core_dev, commandPacket_sent, sizeof(commandPacket_sent));

    	encoderMotorLeft  = 0;  // 清零 
		encoderMotorRight = 0;  // 清零 

/*
          timerCounter++;
          if(timerCounter == 100){
            digitalWrite(GREEN, output);
            output = !output;
            timerCounter = 0;
            Serial.println("encoderMotorLeft:");
            Serial.print(encoderMotorLeft);
            Serial.println("\n\r");
            Serial.println("encoderMotorRight:");
            Serial.print(encoderMotorRight);
            Serial.println("\n\r");
            
          }
          */

        }


} 

