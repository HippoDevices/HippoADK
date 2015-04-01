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

USB_OTG_CORE_HANDLE           USB_OTG_Core_dev;
USBH_HOST                     USB_Host;
#define LED 40


#define MAX_QUERIES 8
#define MINIMUM_SAMPLING_INTERVAL 10
#define REGISTER_NOT_SPECIFIED -1

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned long currentMillisSerial;        // store the current value from millis()
unsigned long previousMillisSerial;       // for comparison with currentMillis
int samplingInterval = 19;          // how often to run the main loop (in ms)

  uint8_t msg5[5]={0xD0,0x00,0x00,0x00,0x00};
    uint8_t msg8[5]={0xF1,0x00,0x00,0x00,0xFF};
      uint8_t init1 = 0;
/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void outputPort(byte portNumber, byte portValue, byte forceSend)
{  
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
     //Firmata.sendDigitalPort(portNumber, portValue);
     msg5[0] = 0xD0;
     msg5[0] |= portNumber;
     msg5[1] = ((byte)portValue % 128); // Tx bits 0-6
     msg5[2] = (portValue >> 7);  // Tx bits 7-13   
     if(init1 == 0){
        //USBH_ADK_write(&USB_OTG_Core_dev, msg8, sizeof(msg8));
        init1 = 1;
      }
    byte portNumberadd;
    portNumberadd = 0xD0;
    portNumberadd |= portNumber;
    //Firmata.sendDigitalPort(portNumberadd, portValue);    
    USBH_ADK_write(&USB_OTG_Core_dev, msg5, sizeof(msg5));  // digitalRead message display 
    //Serial.print("portNumber: ");  Serial.println(portNumber);
    //Serial.print("portValue: ");   Serial.println(portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);  // pin 0-7
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);  // pin 8-15
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);  // pin 16-23
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

void setPinModeCallback(byte pin, int mode)
{
  if((pin==0x30)|(pin==0x31))   // Motor C.
  {
        pinMode(30,OUTPUT);    
        pinMode(31,OUTPUT);
        pinMode(3,OUTPUT); 
        
  	if(pin==0x30)	 	// Forward
  	{          
  	  digitalWrite(30 ,HIGH );  // PB1
  	  digitalWrite(31,LOW);     // PB0   
          digitalWrite(41,1);  // led green  
          analogWrite(3,mode);   // PA8
  			    
  	}else if(pin==0x31)				   //  reversal
  	{
  	  digitalWrite( 30,LOW );
  	  digitalWrite( 31,HIGH);
          digitalWrite(41,0);
  	  analogWrite(3,mode);	
        }
  }
 
        if((pin==0x10)|(pin==0x11))           // Motor A
    {    
        pinMode(17,OUTPUT);    
        pinMode(18,OUTPUT);
        pinMode(5,OUTPUT);       
  	if(pin==0x10)	 	// Forward
  	{
  	  digitalWrite(17 ,HIGH );  // PC2
  	  digitalWrite(18,LOW);     // PC3
          analogWrite(5,mode);   // PC9
  			    
  	}else if(pin==0x11)				   //  reversal
  	{
  	  digitalWrite( 17,LOW );
  	  digitalWrite( 18,HIGH);
  	  analogWrite(5,mode);	
        }
    }  
          if((pin==0x20)|(pin==0x21))              // Motor B
    {       
        pinMode(15,OUTPUT);    
        pinMode(16,OUTPUT);
        pinMode(4,OUTPUT);     
  	if(pin==0x20)	 	// Forward
  	{
  	  digitalWrite(15 ,HIGH );  // PC0
  	  digitalWrite(16,LOW);     // PC1
          analogWrite(4,mode);   // PC8
  			    
  	}else if(pin==0x21)				   //  reversal
  	{
  	  digitalWrite( 15,LOW );
  	  digitalWrite( 16,HIGH);
  	  analogWrite(4,mode);	
        }
    }   
  
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
      reportPINs[pin/8] = 1;  // D001,D101,D201.....
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch (mode) {
    case ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
   //       digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        }
        pinConfig[pin] = ANALOG;
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
   //     digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        pinConfig[pin] = INPUT;
   
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
   //     digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        pinConfig[pin] = OUTPUT;
        
          
      }
      break;
    case PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
  //      analogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PWM;
      }
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{  
    pinMode(PIN_TO_PWM(pin), OUTPUT);
    analogWrite(PIN_TO_PWM(pin), value);
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask = 1, pinWriteMask = 0;
   if (value & 0x01) 
{    
      //  digitalWrite(PIN_TO_DIGITAL(port*8+0), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(port*8+0), OUTPUT);
        pinConfig[port*8+0] = OUTPUT;    
}
if (value & 0x02) 
{    
     //   digitalWrite(PIN_TO_DIGITAL(port*8+1), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(port*8+1), OUTPUT);
        pinConfig[port*8+1] = OUTPUT;    
       
}
if (value & 0x04) 
{    
     //   digitalWrite(PIN_TO_DIGITAL(port*8+2), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(port*8+2), OUTPUT);
        pinConfig[port*8+2] = OUTPUT;    
       
}
if (value & 0x08) 
{    
        pinMode(PIN_TO_DIGITAL(port*8+3), OUTPUT);
   //     digitalWrite(PIN_TO_DIGITAL(port*8+3), LOW); // disable PWM
        pinConfig[port*8+3] = OUTPUT;   
              
}
 if (value == 0x10) 
{   
      //  digitalWrite(PIN_TO_DIGITAL(port*8+4), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(port*8+4), OUTPUT);
        pinConfig[port*8+4] = OUTPUT;      
}
if (value & 0x20) 
{    
      //  digitalWrite(PIN_TO_DIGITAL(port*8+5), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(port*8+5), OUTPUT);
        pinConfig[port*8+5] = OUTPUT;           
}
if (value & 0x40) 
{    
     //   digitalWrite(PIN_TO_DIGITAL(port*8+6), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(port*8+6), OUTPUT);
        pinConfig[port*8+6] = OUTPUT;           
}
if (value & 0x80) 
{    
     //   digitalWrite(PIN_TO_DIGITAL(port*8+7), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(port*8+7), OUTPUT);
        pinConfig[port*8+7] = OUTPUT;          
}

  
  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
   
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)   // C0 01
{
      byte pin;
      if(analogPin< 4)  pin = analogPin+15;
      else pin = analogPin+26;
   
      pinConfig[pin] = ANALOG;
      pinMode(pin,INPUT);
      
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport &~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)  // D0 08 00   pin=3
{ 
    byte pin;
  if(value & 0x01) {pin=port*8+0; //digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}
  if(value & 0x02) {pin=port*8+1;// digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}
  if(value & 0x04) {pin=port*8+2;// digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}
  if(value & 0x08) {pin=port*8+3;// digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}
  if(value & 0x10) {pin=port*8+4;// digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}
  if(value & 0x20) {pin=port*8+5;// digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}
  if(value & 0x40) {pin=port*8+6;// digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}
  if(value & 0x80) {pin=port*8+7; //digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                    pinMode(pin,INPUT);}  
  portConfigInputs[port] = value;  
  
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

void MotorCallback(byte dir, int value)  // F3: Motor C
{
        pinMode(30,OUTPUT);    
        pinMode(31,OUTPUT);
        pinMode(3,OUTPUT); 
         
 //      pinMode(41,OUTPUT);
 //      digitalWrite(41,1);  // led green
  	if(dir==0x00)	 	// Forward
  	{          
  	  digitalWrite(30 ,HIGH );  // PB1
  	  digitalWrite(31,LOW);     // PB0
     
          analogWrite(3,value);   // PA8
  			    
  	}else if(dir==0x01)				   //  reversal
  	{
  	  digitalWrite( 30,LOW );
  	  digitalWrite( 31,HIGH);
//          digitalWrite(41,0);
  	  analogWrite(3,value);	
        }
}

void setup()
{
  uint8_t msg[5];
  uint8_t msg1[5]={0xD1,0x07,0x01,0x00,0x00};  // digitalRead   A B C D
  uint8_t msg3[5]={0}; 
  uint8_t msg4[5]={0xED,0x7B,0x00,0x00,0x00};
  uint8_t msg5[5]={0xC2,0x01,0x00,0x00,0x00};  // analogRead   G  C2
  
  uint8_t msg6[5] = {0xC3,0x01,0x00,0x00,0x00};  // analogRead   H  C3
  uint16_t analogvalue = 0;
  uint8_t initTEST = 0;
  
  uint8_t flag=0;
 RCC_ClocksTypeDef RCC_Clocks;
  pinMode(LED,OUTPUT);
  pinMode(41,OUTPUT);
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);
  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  
  Firmata.attach(Motor_MESSAGE,MotorCallback);
  //Firmata.begin(9600);
  Firmata.begin(230400);
  
  USBH_Init(&USB_OTG_Core_dev,
            USB_OTG_FS_CORE_ID,
            &USB_Host,
            &USBH_ADK_cb,
            &USR_Callbacks);
  
  digitalWrite(LED,HIGH);

  /* Init ADK Library */
   USBH_ADK_Init("HippoDevices Inc.", "BalanceRobot", "Hippo", "2.0", "http://www.hippodevices.com",  "N/A");
   Firmata.USBProcessinput(msg1);  //digitalRead with one command
   //Firmata.USBProcessinput(msg5);  //digitalRead with one command
   //Firmata.USBProcessinput(msg4); //analogRead  with one command
   //Firmata.USBProcessinput(msg6); //analogRead  with one command
   //Firmata.USBProcessinput(msg6); //analogRead  with one command
  while (1)
  {
      byte pin, analogPin;
      /*
       if( USBH_ADK_getStatus() != ADK_IDLE)   // without using usb 
       {
      //if(init == 0){
        //USBH_ADK_write(&USB_OTG_Core_dev, msg8, sizeof(msg8));
        //init = 1;
      //}
        while (Firmata.available())
        Firmata.processInput();
        checkDigitalInputs();
        currentMillisSerial = millis();
  if (currentMillisSerial - previousMillisSerial > samplingInterval) {
    previousMillisSerial = currentMillisSerial;     
      for (pin = 0; pin < TOTAL_PINS; pin++) 
      {
        if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) 
        {           
          analogPin = PIN_TO_ANALOG(pin);          
          if (analogInputsToReport & (1 << analogPin))      
          {   
            byte analogPinadd;
            analogPinadd = 0xC0;
            analogPinadd |= analogPin;
            Firmata.sendAnalog(analogPinadd, analogRead(analogPin));
            //delay(100);  
          }
         
        }
      }
      } 
          for (pin = 0; pin < TOTAL_PINS; pin++) {
            if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {           
                analogPin = PIN_TO_ANALOG(pin);          
            if (analogInputsToReport & (1 << analogPin))  {          
            //Firmata.sendAnalog(analogPin, analogRead(analogPin)); 
            //Serial.println(analogRead(analogPin));
            //Serial.println(analogPin);
            //delay(100);     
            } 
          }
        } 
        
      }   
   */   
      USBH_Process(&USB_OTG_Core_dev , &USB_Host);    
      if( USBH_ADK_getStatus() == ADK_IDLE) { 
      //uint8_t msg8[5]={0xF1,0x00,0x00,0x00,0xFF};
      USBH_ADK_read(&USB_OTG_Core_dev, msg, sizeof(msg)); 
            //if(init1 != 80)
      //init1++;
      if(init1 == 20){
        USBH_ADK_write(&USB_OTG_Core_dev, msg8, sizeof(msg8));
        //init = 80;
      }  
      if(init1 == 40){
        USBH_ADK_write(&USB_OTG_Core_dev, msg8, sizeof(msg8));
        init1 = 80;
      }  
              checkDigitalInputs();
      Firmata.USBProcessinput(msg);
      //msg3[0] = 0xF0; 
      //msg3[4]++;
      //if(msg3[4] == 0x7F)
      //  msg3[4] = 0x00;
      //if(initTEST == 0){
      //USBH_ADK_write(&USB_OTG_Core_dev, msg3, sizeof(msg3)); 
      //delay(1);
      initTEST = 1;
      //}else{
      //USBH_ADK_write(&USB_OTG_Core_dev, msg5, sizeof(msg5));
      initTEST = 0;
      //} 
      if(msg[0]==0xF0)           // Motor C
    {     
        pinMode(30,OUTPUT);    
        pinMode(31,OUTPUT);
        pinMode(3,OUTPUT);  
  	if(msg[1]==0x00)	 	// Forward
  	{          
  	  digitalWrite(30 ,HIGH );  // PB1
  	  digitalWrite(31,LOW);     // PB0
          digitalWrite(41,1);  // led green
          analogWrite(3,(msg[2]&0x7F) + (msg[3]<<7));   // PA8
  			    
  	}else if(msg[1]==0x01)				   //  reversal
  	{
  	  digitalWrite( 30,LOW );
  	  digitalWrite( 31,HIGH);
          digitalWrite(41,0);
  	  analogWrite(3,(msg[2]&0x7F) + (msg[3]<<7));	
        }
    }  
          if(msg[0]==0xF1)           // Motor A
    {    
        pinMode(17,OUTPUT);    
        pinMode(18,OUTPUT);
        pinMode(5,OUTPUT);       
  	if(msg[1]==0x00)	 	// Forward
  	{
  	  digitalWrite(17 ,HIGH );  // PC2
  	  digitalWrite(18,LOW);     // PC3
          analogWrite(5,msg[2]);   // PC9
  			    
  	}else if(msg[1]==0x01)				   //  reversal
  	{
  	  digitalWrite( 17,LOW );
  	  digitalWrite( 18,HIGH);
  	  analogWrite(5,msg[2]);	
        }
    }  
          if(msg[0]==0xF2)              // Motor B
    {       
        pinMode(15,OUTPUT);    
        pinMode(16,OUTPUT);
        pinMode(4,OUTPUT);     
  	if(msg[1]==0x00)	 	// Forward
  	{
  	  digitalWrite(15 ,HIGH );  // PC0
  	  digitalWrite(16,LOW);     // PC1
          analogWrite(4,msg[2]);   // PC8
  			    
  	}else if(msg[1]==0x01)				   //  reversal
  	{
  	  digitalWrite( 30,LOW );
  	  digitalWrite( 31,HIGH);
  	  analogWrite(4,msg[2]);	
        }
    }  
    
    
          
     currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis = currentMillis;     
      for (pin = 0; pin < TOTAL_PINS; pin++) 
      {
        if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) 
        {           
          analogPin = PIN_TO_ANALOG(pin);          
          if (analogInputsToReport & (1 << analogPin))      
          {   
            analogvalue = analogRead(analogPin);
            msg3[0] = 0xC0; 
            msg3[0] = 0xC0|analogPin;       
            msg3[1]=(uint8_t)((analogvalue>>7)&(B01111111));  // MSB
            msg3[2]=(uint8_t)((analogvalue)&(B01111111));    // LSB 
            USBH_ADK_write(&USB_OTG_Core_dev, msg3, sizeof(msg3));   
          }
         
        }
      }
      }    	
    }
    delay(1);
  }
}

void loop() {
}


