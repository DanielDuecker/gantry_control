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



char output_data[30];
char motx_pos[9];
char motx_vel[5];
char moty_pos[9];
char moty_vel[5];

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

int convert_inc_2_mm(int x_pos, int y_pos, int z_pos);
int convert_mm_2_inc(int x_inc, int y_inc, int z_inc);


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
               for (int i=0; i<MotXMessageCount;i++)
               {  
                  motx_pos[i] = MotXMessage[i];
               }
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
                  motx_vel[i] = MotXMessage[i];
               }
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
           Serial.write("Befehl empfangen: ");
           Serial.println(incomingByte);
           
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
                  moty_pos[i] = MotYMessage[i];
                  //Serial.write(MotYMessage[i]);
               }
               //Serial.write(13);
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
                  moty_vel[i] = MotYMessage[i];
                  //Serial.write(MotYMessage[i]);
               }
               //Serial.write(13);
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
  Serial2.write("v-1000");
  Serial2.write(13);
  
  Serial3.write("v0");
  Serial3.write(13);
  
  Serial4.write("v0");
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
  FlexiTimer2::set(5000, 1.0/100000, ControllerStep); // call every 50000 10µs "ticks"
  FlexiTimer2::start();


}

void merge_transmit_data()
  {
  int i = 0;
  int k = 0;
  do 
  {
    output_data[i++] = motx_pos[k++];
  } while (motx_pos[k]!=13);
  output_data[i++] =44;
  k = 0;
  do
  {
    output_data[i++] = motx_vel[k++];
  }while(motx_vel[k]!=13);
  output_data[i++] =44;
  k = 0;
  do 
  {
    output_data[i++] = moty_pos[k++];
  } while (moty_pos[k]!=13);
  output_data[i++] =44;
  k = 0;
  do
  {
    output_data[i++] = moty_vel[k++];
  }while(moty_vel[k]!=13);
  output_data[i++] =44;
  for (int ii=0; ii<i;ii++)
  {
    Serial1.write(output_data[ii]);
  };
  Serial1.write(13);
  Serial1.write(10);
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
