// set this to the hardware serial port you wish to use

#define FH_CONTROLWORD 6060
#define FH_STATUSWORD 6061
#define POLYNOMIAL 0xD5

typedef unsigned char uint8_t;
typedef long int32_t;
typedef unsigned long uint32_t;
typedef short int16_t;
typedef unsigned short uint16_t;


#include <FlexiTimer2.h>

int pwrup_count = 0;

int PCRecCount=0;
int PCArgument[24];
enum PCRecState_t{waitforcommand, argument};
enum PCCommand_t{nix,en,di,la,gohoseq,sp,v};

enum MotXRecState_t{nothingtodo, waitforposition, waitforvelocity};
int MotXMessageCount = 0;
int MotXMessage[10];

enum PosMsgStatus_t{pos_na,pos_requested, pos_received};
enum VelMsgStatus_t{vel_na,vel_requested, vel_received};

const int led_pin = 13;      // default to pin 13

PCRecState_t PCRecState=waitforcommand;
PCCommand_t PCCommand=nix;

MotXRecState_t MotXRecState = nothingtodo; 
PosMsgStatus_t MotXPosStatus = pos_na;
VelMsgStatus_t MotXVelStatus = vel_na;


bool NeuerBefehl=false;
void command_en();
void command_di();
void command_la();
void command_sp();
void command_v();
void getPosition();
void getVelocity();





void ControllerStep()
{  
  static boolean output=HIGH;
  digitalWrite(led_pin, output);
  output = !output;
}

void getPosition()
{
  switch (MotXRecState) 
  {
      case nothingtodo:
        
        Serial2.write("pos");
        Serial2.write(13);
        MotXRecState = waitforposition;
        
        MotXPosStatus = pos_requested;
        
        Serial.println("requested position MotX");
        break;
        
      default:
        break;
  }
}

void getVelocity()
{
  switch (MotXRecState) 
  {
      case nothingtodo:
        
        Serial2.write("gn");
        Serial2.write(13);
        MotXRecState = waitforvelocity;
        
        MotXVelStatus = vel_requested;
        
        Serial.println("requested velocity MotX");
        break;
        
      default:
        break;
  }
} 



void serialEvent1()
{
   int incomingByte;
     
   if (Serial1.available() > 0) 
   {
      incomingByte = Serial1.read();
      switch (PCRecState)
      {
        case waitforcommand:
           switch (incomingByte)
           {
               case 97: PCCommand=en; break;
               case 98: PCCommand=di; break;
               case 99: PCCommand=la; break;
               case 100: PCCommand=gohoseq; break;
               case 101: PCCommand=sp; break;
               case 102: PCCommand=v; break;
           }
                      
           PCRecCount=0;
           PCRecState=argument;
           Serial.write("Befehl empfangen: ");
           Serial.println(incomingByte);
           Serial.println(PCCommand);
           
        break;

        case argument:
           PCArgument[PCRecCount++]=incomingByte;
           if (PCRecCount==4) 
           {
               PCRecState=waitforcommand;
//               for (int ii=0;ii<4;ii++)
//               {
//                  Serial.write(PCArgument[ii]);
//               }   
               NeuerBefehl=true;            
           }
        break;

        default: Serial.write(incomingByte);
        
      }
     
             
   }
}

void serialEvent2()
{
   int incomingByte;

   if (Serial2.available() > 0) 
   {
      incomingByte = Serial2.read();
      switch (MotXRecState)
      {
        case waitforposition:
           MotXMessage[MotXMessageCount++]=incomingByte;
           if (incomingByte = 10)
           {
               MotXPosStatus = pos_received;
               MotXRecState = nothingtodo;
               Serial.write("MotX_position=");
               for (int i=0; i<MotXMessageCount;i++)
               {
                  Serial.write(MotXMessage[i]);
               }
               MotXMessageCount = 0;
                           
           }
           break;

        case waitforvelocity:

           if (incomingByte = 10)
           {
               MotXVelStatus = vel_received;
               MotXRecState = nothingtodo;
               Serial.write("MotX_velocity=");
               for (int i=0; i<MotXMessageCount;i++)
               {
                  Serial.write(MotXMessage[i]);
               }
               MotXMessageCount = 0;
                           
           }
           break;

        default: Serial.write(incomingByte);
        
      }
     
             
   }
}





void serialEvent()
{
   int incomingByte;

  
   if (Serial.available() > 0) 
   {
      incomingByte = Serial.read();   
     
      Serial.write(incomingByte);
   }
}

void serialEvent3()
{
   int incomingByte;

  
   if (Serial3.available() > 0) 
   {
      incomingByte = Serial3.read();
      Serial.write(incomingByte);
   }
}

void command_en()  // enable drive
{
  Serial2.write("en");
  Serial2.write(13);
  Serial3.write("en");
  Serial3.write(13);
  Serial.println("en");
  Serial.write(13);
}

void command_di()  // disable drive
{
  Serial2.write("di");
  Serial2.write(13);
  Serial3.write("di");
  Serial3.write(13);
  Serial.write("di");
  Serial.write(13);
}

void command_la() // go to absolute position
{
  Serial2.write("la");
  for (int ii=0;ii<8;ii++)
  {
     Serial2.write(PCArgument[ii]);
  }
  Serial2.write(13);
  
  Serial3.write("la");
  for (int ii=8;ii<16;ii++)
  {
     Serial3.write(PCArgument[ii]);
  }
  Serial3.write(13);
  
  Serial4.write("la");
  for (int ii=16;ii<24;ii++)
  {
     Serial4.write(PCArgument[ii]);
  }
  Serial4.write(13);
}

void command_gohoseq()
{
  Serial.write("gohoseq");
  Serial.write(13);
  
  Serial3.write("gohoseq");
  Serial3.write(13);
  
  //Serial4.write("gohoseq");
  //Serial4.write(13);
  //Serial4.write(10);
}

void command_sp() // set max speed
{
  Serial2.write("sp");
  Serial2.write("");
  Serial2.write(13);
  
  Serial3.write("sp");
  Serial3.write("");
  Serial3.write(13);
  
  Serial4.write("sp");
  Serial4.write("");
  Serial4.write(13);
}

void command_v()
{
  Serial2.write("v");
  Serial2.write(13);
  
  Serial3.write("v");
  Serial3.write(13);
  
  Serial4.write("v");
  Serial4.write(13);
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
  Serial2.begin(19200,SERIAL_8N1);
  Serial3.begin(19200,SERIAL_8N1);
  Serial4.begin(19200,SERIAL_8N1);
  FlexiTimer2::set(50000, 1.0/100000, ControllerStep); // call every 50000 10µs "ticks"
  FlexiTimer2::start();
}

void loop() 
{
  
   if (NeuerBefehl)
   {
      NeuerBefehl=false;
      switch (PCCommand)
           {
               case en: // enable drives
                   command_en();
               break;
               
               case di:  // disable drives
                   command_di();
               break;
               
               
               case la:  // go to absolute position
                   command_la();
               break;
               
               
               case gohoseq:  // start homing sequence
                   command_gohoseq();
               break;
               
               case sp: // set max speed
                getVelocity();
               break;
               
               
               case v:  // set target velocity
                  getPosition();
               
               break;
           }
   }
}
