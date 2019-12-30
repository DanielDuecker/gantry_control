// set this to the hardware serial port you wish to use

#define FH_CONTROLWORD 6060
#define FH_STATUSWORD 6061
#define POLYNOMIAL 0xD5

typedef unsigned char uint8_t;
typedef long int32_t;
typedef unsigned long uint32_t;
typedef short int16_t;
typedef unsigned short uint16_t;

#include <Arduino.h>
#include <math.h>
#include <FlexiTimer2.h>

int pwrup_count = 0;

int PCRecCount=0;
int PCArgument[19];
char PCCommandArgument[19];
int CommandArgument_int[3];
int CommandArgument_test[3];

enum PCRecState_t{waitforcommand, argument};
enum PCCommand_t{nix,en,di,la,gohoseq,sp,v};
enum MotRecState_t{nothingtodo, waitfor_pos_data, waitfor_vel_data};
enum DataMsgStatus_t{na, avail, sent};

MotRecState_t MotXRecState = nothingtodo; 
MotRecState_t MotYRecState = nothingtodo; 
DataMsgStatus_t MotXPosState = na;
DataMsgStatus_t MotXVelState = na;
DataMsgStatus_t MotYPosState = na;
DataMsgStatus_t MotYVelState = na;


const int led_pin = 13;      // default to pin 13

PCRecState_t PCRecState=waitforcommand;
PCCommand_t PCCommand=nix;

int MotXMessageCount = 0;
int MotXMessage[14];
int MotX_pos[5];
int MotX_vel[5];

int MotYMessageCount = 0;
int MotYMessage[14];
int MotY_pos[5];
int MotY_vel[5];




char motx_pos_c[8];
int motx_pos;
char motx_vel_c[5];
int motx_vel;
char moty_pos_c[8];
int moty_pos;
char moty_vel_c[5];
int moty_vel;

boolean trigger=false;
boolean todo = false;



bool NeuerBefehl=false;
void command_en();
void command_di();
void command_la();
void command_sp();
void command_v();


void getPosMotX();
void getVelMotX();
void getPosMotY();
void getVelMotY();

void merge_transmit_data();

const int incPERmm_x = 2000000/3100;
const int incPERmm_y = 945800/1600;
const int incPERmm_z = 1;

//int convert_inc_2_mm(int x_pos, int y_pos, int z_pos);
//int convert_mm_2_inc(int x_inc, int y_inc, int z_inc);

int conv_2_int (byte incomingByte);

int conv_bytestream_2_int (byte incomingByte)
{
  static int receivedNumber=0;
  static boolean negative = false;
  switch (incomingByte)
  {                 
    //case endOfNumberDelimiter:  
    //  if (negative)
    //   processNumber (- receivedNumber);
    //  else
    //  processNumber (receivedNumber);
    
    case '0' ... '9':
      receivedNumber *= 10;
      receivedNumber += incomingByte - '0';
      break;
      
    case '-':
      negative = true;
      break;
     
    } // end of switch  

  if (negative) receivedNumber *=-1;
  return receivedNumber;
}

void ControllerStep()
{  
  static boolean output=HIGH;
  digitalWrite(led_pin, output);
  output = !output;
  trigger =!trigger;
  todo = true;
}

void getPosMotX()
{
  switch (MotXRecState) 
  {
      case nothingtodo:
        
        Serial2.write("pos");
        Serial2.write(13);
        MotXRecState = waitfor_pos_data;
        break;
        
      default:
        break;
  }
}

void getPosMotY()
{
  switch (MotYRecState) 
  {
      case nothingtodo:
        
        Serial3.write("pos");
        Serial3.write(13);
        MotYRecState = waitfor_pos_data;
        break;
        
      default:
        break;
  }
}

void getVelMotX()
{
  switch (MotXRecState) 
  {
      case nothingtodo:
        
        Serial2.write("gn");
        Serial2.write(13);
        MotXRecState = waitfor_vel_data;
        break;
        
      default:
        break;
  }
}

void getVelMotY()
{
  switch (MotYRecState) 
  {
      case nothingtodo:
        
        Serial3.write("gn");
        Serial3.write(13);
        MotYRecState = waitfor_vel_data;
        break;
        
      default:
        break;
  }
}


void serialEvent2()
{
   int incomingByte;
   // int myint;
   if (Serial2.available() > 0) 
   {
      incomingByte = Serial2.read();
      MotXMessage[MotXMessageCount++]=incomingByte;
      
      switch (MotXRecState)
      {
        case waitfor_pos_data:
           if (incomingByte == 10)
           {
               //Serial.write("MotX_position=");
               for (int i=0; i<MotXMessageCount;i++)  // MotXMessageCount contains 2 terminator bytes
               {  
                  motx_pos_c[i] = MotXMessage[i];
                  //Serial1.printf("MotX %d, %s",i, motx_pos[i]);
               }
               //Serial1.printf("MotXMessageCount = %d\n\r",MotXMessageCount);
               motx_pos_c[MotXMessageCount] = "\0";
               motx_pos = atoi(motx_pos_c) / incPERmm_x;
               //Serial1.printf("motx = %d\n\r",motx_pos);
               MotXMessageCount = 0;
               MotXPosState = avail;
               MotXRecState = nothingtodo;               
           }
           break;

        case waitfor_vel_data:
           if (incomingByte == 10)
           {
               //Serial.write("MotX_velocity=");
               for (int i=0; i<MotXMessageCount;i++)
               {
                  motx_vel_c[i] = MotXMessage[i];
               }
               motx_vel_c[MotXMessageCount] = "\0";
               motx_vel = atoi(motx_vel_c);
               MotXMessageCount = 0;
               MotXVelState = avail;
               MotXRecState = nothingtodo;                          
           }
           break;

        default: Serial.write(incomingByte);
        
      }
     
             
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
               case 97: PCCommand=en; break;        // a
               case 98: PCCommand=di; break;        // b
               case 99: PCCommand=la; break;        // c
               case 100: PCCommand=gohoseq; break;  // d
               case 101: PCCommand=sp; break;       // e
               case 102: PCCommand=v; break;        // f
           }
                      
           PCRecCount=0;
           PCRecState=argument;
           Serial1.write("Befehl empfangen: ");
           Serial1.println(incomingByte);
           
        break;

        case argument:
           PCArgument[PCRecCount++]=incomingByte;
           const int arg_length = 3;
           if (PCRecCount==9) 
           {
               char DriveCommandArgument[arg_length+1];

               PCRecState=waitforcommand;
               //Serial1.printf("Command  = %d",PCArgument);
               
               for (int ii=0;ii<arg_length;ii++)
               {
                  DriveCommandArgument[ii] = PCArgument[ii];
                  Serial1.write(DriveCommandArgument[ii]);
               }
               DriveCommandArgument[arg_length] = "\0";
               CommandArgument_int[0] = atoi(DriveCommandArgument);

               for (int ii=0;ii<arg_length;ii++)
               {
                  DriveCommandArgument[ii] = PCArgument[ii+arg_length]; // second argument
                  Serial1.write(DriveCommandArgument[ii]);
               }
               DriveCommandArgument[arg_length] = "\0";
               CommandArgument_int[1] = atoi(DriveCommandArgument);
        
               for (int ii=0;ii<arg_length;ii++)
               {
                  DriveCommandArgument[ii] = PCArgument[ii+2*arg_length]; // third argument
                  Serial1.write(DriveCommandArgument[ii]);
               }                       
               DriveCommandArgument[arg_length] = "\0";
               CommandArgument_int[2] = atoi(DriveCommandArgument);
               
               
              // debugging to serial 1
               Serial1.printf("\n\r");
               Serial1.printf("Command Arg0 = %d \n\r", CommandArgument_int[0]*incPERmm_x);
               Serial1.printf("Command Arg1 = %d \n\r", CommandArgument_int[1]*incPERmm_y);
               Serial1.printf("Command Arg2 = %d \n\r", CommandArgument_int[2]*incPERmm_z);
          
               
               NeuerBefehl=true;
               PCRecCount = 0;
           }
        break;

        default: Serial1.write(incomingByte);
        
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
      MotYMessage[MotYMessageCount++]=incomingByte;
      
      switch (MotYRecState)
      {
        case waitfor_pos_data:
           if (incomingByte == 10)
           {
               //Serial.write("MotY_position=");
               for (int i=0; i<MotYMessageCount;i++)
               {  
                  moty_pos_c[i] = MotYMessage[i];
                  //Serial.write(MotYMessage[i]);
               }
               //Serial.write(13);
               moty_pos_c[MotYMessageCount] = "\0";
               moty_pos = atoi(moty_pos_c) / incPERmm_y;
               MotYMessageCount = 0;
               MotYPosState = avail;
               MotYRecState = nothingtodo;               
           }
           break;

        case waitfor_vel_data:
           if (incomingByte == 10)
           {
               //Serial.write("MotY_velocity=");
               for (int i=0; i<MotYMessageCount;i++)
               {
                  moty_vel_c[i] = MotYMessage[i];
                  //Serial.write(MotYMessage[i]);
               }
               //Serial.write(13);
               moty_vel_c[MotYMessageCount] = "\0";
               moty_vel = atoi(moty_vel_c);
               MotYMessageCount = 0;
               MotYVelState = avail;
               MotYRecState = nothingtodo;                          
           }
           break;

        default: Serial.write(incomingByte);
        
      }
   }
}

void command_en()  // enable drive
{
  Serial2.write("en");
  Serial2.write(13);
  Serial3.write("en");
  Serial3.write(13);
  Serial1.println("en");
  Serial1.write(13);
}

void command_di()  // disable drive
{
  Serial2.write("di");
  Serial2.write(13);
  Serial3.write("di");
  Serial3.write(13);
  Serial1.write("di");
  Serial1.write(13);
}

void command_la() // go to absolute position
{
  Serial2.write("la");
  Serial2.printf("%d",CommandArgument_int[0]*incPERmm_x);
  Serial1.printf("Sent to X %d\n",CommandArgument_int[0]*incPERmm_x);
  Serial2.write(13);
  Serial2.write("M");
  Serial2.write(13);
  
  Serial3.write("la");
  Serial3.printf("%d",CommandArgument_int[1]*incPERmm_y);
  Serial1.printf("Sent to Y %d\n",CommandArgument_int[1]*incPERmm_y);
  Serial3.write(13);
  Serial3.write("M");
  Serial3.write(13);
  
  //Serial4.write("la");
  //Serial4.printf("%d",CommandArgument_int[1]*incPERmm_y);
  //Serial4.write(13);
  //Serial4.write("M");
  //Serial4.write(13);
}

void command_gohoseq()
{
  Serial2.write("gohoseq");
  Serial2.write(13);
  
  Serial3.write("gohoseq");
  Serial3.write(13);
  
  //Serial4.write("gohoseq");
  //Serial4.write(13);
  //Serial4.write(10);
}

void command_sp() // set max speed
{
  Serial2.write("sp");
  Serial2.write("1000");
  Serial2.write(13);
  
  Serial3.write("sp");
  Serial3.write("1000");
  Serial3.write(13);
  
  //Serial4.write("sp");
  //Serial4.write("");
  //Serial4.write(13);
}

void command_v()
{
  Serial2.write("v");
  Serial2.printf("%d",CommandArgument_int[0]);
  Serial1.printf("Sent Vx= %d\n\r",CommandArgument_int[0]);
  Serial2.write(13);
  
  Serial3.write("v0");
  Serial3.printf("%d",CommandArgument_int[1]);
  Serial1.printf("Sent Vx= %d\n\r",CommandArgument_int[2]);
  Serial3.write(13);
  
  //Serial4.write("v0");
  //Serial4.printf("%d",CommandArgument_int[2]);
  //Serial1.printf("Sent Vx %d\n\r",CommandArgument_int[2]);
  //Serial4.write(13);
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
  FlexiTimer2::set(5000, 1.0/100000, ControllerStep); // call every 50000 10µs "ticks"
  FlexiTimer2::start();


}

void merge_transmit_data()
  {
  //Serial1.printf("Status Xp= %d, Xv= %d, Yp= %d, Yv= %d \n\r",motx_pos,motx_vel,moty_pos,moty_vel);
  
  //Serial1.printf("%d,%d,%d,%d\n\r",motx_pos,motx_vel,moty_pos,moty_vel);
}
  

void loop() 
{
  
    if (trigger && todo)
    {
      if (MotXPosState != avail) getPosMotX();
      if (MotXVelState != avail) getVelMotX();
      if (MotYPosState != avail) getPosMotY();
      if (MotYVelState != avail) getVelMotY();
      todo = false;
      
        
    }
    if ((MotXPosState == avail) && 
        (MotXVelState == avail) &&
        (MotYVelState == avail) &&
        (MotYVelState == avail))
    {
      merge_transmit_data();
      
      MotXPosState = sent;
      MotXVelState = sent;
      MotYPosState = sent;
      MotYVelState = sent;
    }

    
    
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
                    command_sp();   
               break;
               
               
               case v:  // set target velocity
                    command_v();
               
               break;
           }
   }
}
