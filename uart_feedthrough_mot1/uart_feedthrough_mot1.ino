// set this to the hardware serial port you wish to use



typedef unsigned char uint8_t;
typedef long int32_t;
typedef unsigned long uint32_t;
typedef short int16_t;
typedef unsigned short uint16_t;


#include <FlexiTimer2.h>



const int led_pin = 13;      // default to pin 13







void ControllerStep()
{  
  static boolean output=HIGH;
  digitalWrite(led_pin, output);
  output = !output;
}


void serialEvent1()
{
   int incomingByte;
     
   if (Serial1.available() > 0) 
   {
      incomingByte = Serial1.read();
      
      Serial2.write(incomingByte);     
             
   }
}


void serialEvent2()
{
   int incomingByte;

  
   if (Serial2.available() > 0) 
   {
      incomingByte = Serial2.read();
      Serial1.write(incomingByte);
   }
}


void setup() {  
  pinMode(led_pin, OUTPUT);  
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial2.setRX(7);
  Serial2.setTX(8);
  Serial3.setRX(15);
  Serial3.setTX(14);
  Serial4.setRX(16);
  Serial4.setTX(17);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  FlexiTimer2::set(50000, 1.0/100000, ControllerStep); // call every 50000 10µs "ticks"
  FlexiTimer2::start();
  
}

void loop() 
{
   
}
