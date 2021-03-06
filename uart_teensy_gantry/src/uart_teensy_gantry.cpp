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



enum CommunicationMode_t{gantry_control, feedthrough_Serial1_2, feedthrough_Serial1_3, feedthrough_Serial1_4};
enum PCRecState_t{waitforcommand, argument};
enum PCCommand_t{nix,en,di,sp,gohoseq,la,lr,v};
enum MotRecState_t{nothingtodo, waitfor_pos_data, waitfor_vel_data};
enum DataMsgStatus_t{na, avail, sent};

// select Communication mode
const CommunicationMode_t CommunicationMode = gantry_control;



MotRecState_t MotXRecState = nothingtodo; 
MotRecState_t MotYRecState = nothingtodo; 
MotRecState_t MotZRecState = nothingtodo; 

DataMsgStatus_t MotXPosState = na;
DataMsgStatus_t MotXVelState = na;

DataMsgStatus_t MotYPosState = na;
DataMsgStatus_t MotYVelState = na;

DataMsgStatus_t MotZPosState = na;
DataMsgStatus_t MotZVelState = na;

const int led_pin = 13;      // default to pin 13

PCRecState_t PCRecState=waitforcommand;
PCCommand_t PCCommand=nix;

int MotXMessageCount = 0;
int MotXMessage[20];
int MotX_pos[10];
int MotX_vel[10];

int MotYMessageCount = 0;
int MotYMessage[20];
int MotY_pos[10];
int MotY_vel[10];

int MotZMessageCount = 0;
int MotZMessage[20];
int MotZ_pos[10];
int MotZ_vel[10];

const int arg_length = 6;
char DriveCommandArgument[arg_length+1];

int counter = 0;

// Motor X
char motx_pos_c[15];
int motx_pos_mm;
char motx_vel_c[10];
int motx_vel_mms;

// Motor Y
char moty_pos_c[15];
int moty_pos_mm;
char moty_vel_c[10];
int moty_vel_mms;

// Motor Z
char motz_pos_c[15];
int motz_pos_mm;
char motz_vel_c[10];
int motz_vel_mms;

boolean trigger=false;
boolean todo = false;


// Status and Timeouts
boolean no_time_out = true;

int time_out_counter_x = 0;
const int time_out_lim_x = 100;
boolean b_time_out_x_counting = false;

int time_out_counter_y = 0;
const int time_out_lim_y = 100;
boolean b_time_out_y_counting = false;

int time_out_counter_z = 0;
const int time_out_lim_z = 100;
boolean b_time_out_z_counting = false;

int transmit_rate_counter = 0;

bool NeuerBefehl=false;
void command_en();
void command_di();
void command_sp(int vx_mms_max, int vy_mms_max, int vz_mms_max);
void command_gohoseq();
void command_la(int x_mm, int y_mm, int z_mm);
void command_lr(int dx_mm, int dy_mm, int dz_mm);
void command_v(int vx_mms, int vy_mms, int vz_mms);

// Motor X
void getPosMotX();
void getVelMotX();
// Motor Y
void getPosMotY();
void getVelMotY();

// Motor Z
void getPosMotZ();
void getVelMotZ();

void transmit_data();
void reset_com_link_serial(int iserial);

const int incPERmm_x = 2000000/3100;
const int incPERmm_y = 945800/1600;
const int incPERmm_z = 1920612/940;
const float mmPERsPERrpm_x = 1000/28.2/500;   // 1000mm/28.2s @  500rpm
const float mmPERsPERrpm_y = 1000/35.4/1000;  // 1000mm/35.4s @ 1000rpm 
const float mmPERsPERrpm_z = 940/38.2/3000;   //  940mm/38.2s @ 3000rpm

const int invert_z_axis = -1;  // invert all z commands 

void time_out_update()
{
  if (b_time_out_x_counting)
  {
    time_out_counter_x++;
  }
  if (b_time_out_y_counting)
  {
    time_out_counter_y++;
  }
  if (b_time_out_z_counting)
  {
    time_out_counter_z++;
  }
  

  if (time_out_counter_x > time_out_lim_x)
  {
    Serial.printf("Serial 2 Timout %d ms\r\n", time_out_counter_x);
    reset_com_link_serial(2);
    no_time_out = false;
    
  }
  if (time_out_counter_y > time_out_lim_y)
  {
    reset_com_link_serial(3);
    no_time_out = false;
    Serial.printf("Serial 3 Timout %d ms\r\n", time_out_counter_y);
  }
  if (time_out_counter_z > time_out_lim_z)
  {
    reset_com_link_serial(4);
    no_time_out = false;
    Serial.printf("Serial 4 Timout %d ms\r\n", time_out_counter_z);
  }
 
}

void time_out_x_reset(boolean b_start_count)
{
  time_out_counter_x = 0;
  b_time_out_x_counting = b_start_count;
}

void time_out_y_reset(boolean b_start_count)
{
  time_out_counter_y = 0;
  b_time_out_y_counting = b_start_count;
}

void time_out_z_reset(boolean b_start_count)
{
  time_out_counter_z = 0;
  b_time_out_z_counting = b_start_count;
}
/*
void communication_success_serial(int i_drive)
{
  if (i_drive == 2)
  {
    time_out_x_reset(false);
  }
  else if (i_drive == 3)
  {
    time_out_y_reset(false);
  }
  else if (i_drive == 3)
  {
    time_out_z_reset(false);
  }   
}
*/
void reset_com_link_serial(int i_drive)
{
  if (i_drive == 2)
  {
    //MotXRecState = nothingtodo; 
    MotXPosState = na;
    MotXVelState = na;
    time_out_x_reset(false);
  }
  else if (i_drive == 3)
  {
    MotYRecState = nothingtodo; 
    MotYPosState = na;
    MotYVelState = na;
    time_out_y_reset(false);
  }
  else if (i_drive == 4)
  {
    MotZRecState = nothingtodo; 
    MotZPosState = na;
    MotZVelState = na;
    time_out_z_reset(false);
  }
}

void Status_LED()
{
  static boolean led_output=HIGH;
  static int led_count = 0;
  static int flash_rate_hz;

  if (CommunicationMode == gantry_control and no_time_out)
  {
    flash_rate_hz = 2;
  }
  else if (CommunicationMode==feedthrough_Serial1_2 or CommunicationMode==feedthrough_Serial1_3 or CommunicationMode==feedthrough_Serial1_4)
  {
    flash_rate_hz = 10;
  }
  else
  {
    flash_rate_hz = 30; // e.g if time out
  }

  led_count++;

  digitalWrite(led_pin, led_output);

  if (led_count == 1000/flash_rate_hz)
  {
    led_output = !led_output;
    led_count = 0;
  } 
}

void send_alive_msg()
{
  static int alive_cycle_counter = 0;
  static int alive_seconds = 0;
  static int alive_minutes = 0;
  static int alive_hours = 0;
  static int alive_days = 0;

  alive_cycle_counter++;

  if (no_time_out == false)
  {
    alive_seconds = 0;
    alive_minutes = 0;
    alive_hours = 0;
    alive_days = 0;
    no_time_out = true;
  }

  if (alive_cycle_counter == 1000)
  {
    alive_cycle_counter = 0;
    alive_seconds++;
    
    

    if (alive_seconds % 60 == 0)
    {
      alive_seconds = 0;
      alive_minutes++;
      if (alive_minutes % 60 == 0)
      {
        alive_hours++;
        if (alive_hours == 24)
        {
          alive_days++;
        }
      }
    }
    Serial.printf("Gantry alive w/o time-out since   %dd %dh %dmin %ds \t Transmitting data @ %dHz\r\n",
                   alive_days, alive_hours, alive_minutes, alive_seconds, transmit_rate_counter);
    
    transmit_rate_counter = 0;
  }
    
}


void ControllerStep() // running at 1000Hz
{ 
  Status_LED();
  trigger =!trigger;
  todo = true;
  time_out_update();
  send_alive_msg();
}


// Motor X
void getPosMotX()
{
  switch (MotXRecState) 
  {
      case nothingtodo:
        
        Serial2.write("pos");
        Serial2.write(13);
        MotXRecState = waitfor_pos_data;
        time_out_x_reset(true);
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
        Serial.println("gn ser 2");
        MotXRecState = waitfor_vel_data;
        time_out_x_reset(true);
        break;
        
      default:
        Serial.printf("motx_rec_state: %d\r\n", MotXRecState);
        break;
  }
}


// Motor Y
void getPosMotY()
{
  switch (MotYRecState) 
  {
      case nothingtodo:
        
        Serial3.write("pos");
        Serial3.write(13);
        MotYRecState = waitfor_pos_data;
        time_out_y_reset(true);
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
        time_out_y_reset(true);
        break;
        
      default:
        break;
  }
}

// Motor Z
void getPosMotZ()
{
  switch (MotZRecState) 
  {
      case nothingtodo:
        
        Serial4.write("pos");
        Serial4.write(13);
        MotZRecState = waitfor_pos_data;
        time_out_z_reset(true);
        break;
        
      default:
        break;
  }
}

void getVelMotZ()
{
  switch (MotZRecState) 
  {
      case nothingtodo:
        
        Serial4.write("gn");
        Serial4.write(13);
        MotZRecState = waitfor_vel_data;
        time_out_z_reset(true);
        break;
        
      default:
        break;
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

void serialEvent1()
{
  static int incomingByte;

  switch (CommunicationMode)
  {
    case gantry_control:
    {
      // incoming command has to be in the form of
      // command[1]arg1[6]arg2[6]arg3[6]\r\n
      // arguments have to be in [mm] or [mm/s] respectively
      // example for go to absolute position: e123456123456123456

      if (PCRecState == 10) // super uggly bugfix, since for negative "x" commmands PCRecState is magically set to 10
      {
        PCRecState = waitforcommand;
      }
      if (Serial1.available() > 0) 
      {
          incomingByte = Serial1.read();

          //Serial.printf("Serial1 event: Incoming Byte %d\r\n",incomingByte);
          switch (PCRecState)
          {
            case waitforcommand:
            {
              //Serial.printf("wait for command received %d\r\n",incomingByte);

              int commandByte = incomingByte;
              switch (commandByte)
              {
                  case 97:  PCCommand=en;  break;       // a
                  case 98:  PCCommand=di;  break;       // b
                  case 99:  PCCommand=sp;  break;       // c
                  case 100: PCCommand=gohoseq; break;  // d
                  case 101: PCCommand=la; break;       // e
                  case 102: PCCommand=lr; break;       // f
                  case 103: PCCommand=v;  break;       // g
                  default:  PCCommand=nix; 
                      Serial.printf("kein gueltiger Befehl: %d\r\n",commandByte);
                      break;

              }
                          
              PCRecCount=0;

              PCRecState=argument;
              Serial.printf("\r\nBefehl empfangen: %d\r\n",commandByte);
            }
            break;

            case argument:
            {
              PCArgument[PCRecCount++]=incomingByte;
              //Serial.printf("wait for argument received %d\r\n",incomingByte);
              
              if (incomingByte==10 and PCRecCount==20) 
              {
                  // Motor X commands
                  for (int ii=0;ii<arg_length;ii++)
                  {
                      DriveCommandArgument[ii] = PCArgument[ii];
                  }
                  DriveCommandArgument[arg_length] = '\0';
                  CommandArgument_int[0] = atoi(DriveCommandArgument);
                  Serial.printf("Arg0: %d\r\n", CommandArgument_int[0]);

                  // Motor Y Commands
                  for (int ii=0;ii<arg_length;ii++)
                  {
                    DriveCommandArgument[ii] = PCArgument[ii+arg_length]; // second argument
                  }
                  DriveCommandArgument[arg_length] = '\0';
                  CommandArgument_int[1] = atoi(DriveCommandArgument);
                  Serial.printf("Arg1: %d\r\n", CommandArgument_int[1]);
                  
                  // Motor Z Commmands        
                  for (int ii=0;ii<arg_length;ii++)
                  {
                    DriveCommandArgument[ii] = PCArgument[ii+2*arg_length]; // third argument
                  }                       
                  DriveCommandArgument[arg_length] = '\0';
                  CommandArgument_int[2] = atoi(DriveCommandArgument);
                  Serial.printf("Arg2: %d\r\n", CommandArgument_int[2]);         
                  
                  // debugging to serial ACM0
                  // Serial.printf("\r\n");
                  // Serial.printf("Command Arg0 = %d \r\n", CommandArgument_int[0]);
                  // Serial.printf("Command Arg1 = %d \r\n", CommandArgument_int[1]);
                  // Serial.printf("Command Arg2 = %d \r\n", CommandArgument_int[2]);
              
                  NeuerBefehl=true;
                  PCRecCount = 0;
                  PCRecState=waitforcommand;
              }
            }
            break;
        }
      }
      break;
    }
    case feedthrough_Serial1_2: // feedthrough mode
    {
      if (Serial1.available() > 0) 
      {
          incomingByte = Serial1.read();
          Serial2.write(incomingByte);
      }
      break;
    }
    case feedthrough_Serial1_3: // feedthrough mode
    {
      if (Serial1.available() > 0) 
      {
          incomingByte = Serial1.read();
          Serial3.write(incomingByte);
      }
      break;
    }
    case feedthrough_Serial1_4: // feedthrough mode
    {
      if (Serial1.available() > 0) 
      {
          incomingByte = Serial1.read();
          Serial4.write(incomingByte);
      }
      break;
    }
  }
}

// Motor X Incoming 
void serialEvent2()
{
  static int incomingByte;

  if (CommunicationMode==gantry_control)
  {
    if (Serial2.available() > 0) 
    {
        incomingByte = Serial2.read();
        MotXMessage[MotXMessageCount++]=incomingByte;
        
        switch (MotXRecState)
        {
          case waitfor_pos_data:
          {
            if (incomingByte == 10)
            {
                Serial.printf("Current Message Cound POS X %d \r\n",MotXMessageCount);
                for (int i=0; i<MotXMessageCount;i++)  // MotXMessageCount contains 2 terminator bytes
                {  
                    motx_pos_c[i] = MotXMessage[i];
                    //Serial1.printf("MotX %d, %s",i, motx_pos[i]);
                }
                //Serial1.printf("MotXMessageCount = %d\r\n",MotXMessageCount);
                motx_pos_c[MotXMessageCount] = '\0';
                
                motx_pos_mm = atoi(motx_pos_c) / incPERmm_x;
                // Serial.printf("motx_mm = %d\r\n",motx_pos_mm);
                MotXMessageCount = 0;
                MotXPosState = avail;
                MotXRecState = nothingtodo;               
            }
            break;
          }
          case waitfor_vel_data:
          {
            if (incomingByte == 10)
            {
                //Serial.write("MotX_velocity=");
                Serial.printf("Current Message Cound VEL X %d \r\n",MotXMessageCount);
                for (int i=0; i<MotXMessageCount;i++)
                {
                    motx_vel_c[i] = MotXMessage[i];
                    Serial.printf("%d\r\n",MotXMessage[i]);
                }
                motx_vel_c[MotXMessageCount] = '\0';
                motx_vel_mms = atoi(motx_vel_c) * mmPERsPERrpm_x;
                MotXMessageCount = 0;
                MotXVelState = avail;
                MotXRecState = nothingtodo;                          
            }
            break;
          }
          case nothingtodo:
            Serial.println("Nothing to DO 2 \r\n");
            Serial.printf("%d \r\n",incomingByte);
            break;
          default: 
          {
            Serial.write(incomingByte);
            break;
          } 
        }
    }
  }
  else if (CommunicationMode==feedthrough_Serial1_2)
  {
    if (Serial2.available() > 0) 
    {
      incomingByte = Serial2.read();
      Serial1.write(incomingByte);
      Serial.println("Feedthrough 1-2");
    }  
  }
  else
  {
    Serial.println("Nothing to do!");
    Serial.printf("Serial Event2: Communication Mode %d\r\n", CommunicationMode);
  }
}


// Motor Y Incoming 
void serialEvent3()
{
  int incomingByte;

  if (CommunicationMode==gantry_control)
  {
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
                moty_pos_c[MotYMessageCount] = '\0';
                moty_pos_mm = atoi(moty_pos_c) / incPERmm_y;
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
                moty_vel_c[MotYMessageCount] = '\0';
                moty_vel_mms = atoi(moty_vel_c) * mmPERsPERrpm_y;
                MotYMessageCount = 0;
                MotYVelState = avail;
                MotYRecState = nothingtodo;                          
            }
            break;
        case nothingtodo:
          Serial.println("Nothing to DO 3 \r\n");
          break;
        default: Serial.write(incomingByte);
        
      }
    }
  }
  else if (CommunicationMode==feedthrough_Serial1_3)
  {
    if (Serial3.available() > 0) 
    {
      incomingByte = Serial3.read();
      Serial1.write(incomingByte);
      Serial.println("Feedthrough 1-3");
    } 
  }
  else
  {
    Serial.println("Nothing to do");
    Serial.printf("Serial Event3: Communication Mode %d\r\n", CommunicationMode);
  }

}

// Motor Z Incoming // all value are inverted after reading /  before sending
void serialEvent4()
{
  static int incomingByte;

  if (CommunicationMode==gantry_control)
  {
    if (Serial4.available() > 0) 
    {
      incomingByte = Serial4.read();
      MotZMessage[MotZMessageCount++]=incomingByte;
      
      switch (MotZRecState)
      {
        case waitfor_pos_data:
           if (incomingByte == 10)
           {
               //Serial.write("MotZ_position=");
               for (int i=0; i<MotZMessageCount;i++)
               {  
                  motz_pos_c[i] = MotZMessage[i];
                  //Serial.write(MotZMessage[i]);
               }
               //Serial.write(13);
               motz_pos_c[MotZMessageCount] = '\0';
               motz_pos_mm = atoi(motz_pos_c) / incPERmm_z * invert_z_axis; // all value are inverted after reading
               MotZMessageCount = 0;
               MotZPosState = avail;
               MotZRecState = nothingtodo;               
           }
           break;

        case waitfor_vel_data:
           if (incomingByte == 10)
           {
               //Serial.write("MotZ_velocity=");
               for (int i=0; i<MotZMessageCount;i++)
               {
                  motz_vel_c[i] = MotZMessage[i];
                  //Serial.write(MotZMessage[i]);
               }
               //Serial.write(13);
               motz_vel_c[MotZMessageCount] = '\0';
               motz_vel_mms = atoi(motz_vel_c) * mmPERsPERrpm_z * invert_z_axis; // all value are inverted after reading
               MotZMessageCount = 0;
               MotZVelState = avail;
               MotZRecState = nothingtodo;                          
           }
           break;
         case nothingtodo:
            Serial.println("Nothing to DO 4 \r\n");
            break;
        default: Serial.write(incomingByte);
        
      }
    }
  }
  else if (CommunicationMode==feedthrough_Serial1_4)
  {
    if (Serial4.available() > 0) 
    {
      incomingByte = Serial4.read();
      Serial1.write(incomingByte);
      Serial.println("Feedthrough 1-4");
    } 
  }
  else
  {
    Serial.println("Nothing todo");
    Serial.printf("Serial Event4: Communication Mode %d\r\n", CommunicationMode);
  }

}

void command_en()  // enable drive
{
  Serial2.printf("en");
  Serial2.write(13);
  Serial3.printf("en");
  Serial3.write(13);
  Serial4.printf("en");
  Serial4.write(13);
  
  Serial.printf("en");
  Serial.write(13);
}

void command_di()  // disable drive
{
  Serial2.printf("di");
  Serial2.write(13);
  Serial3.printf("di");
  Serial3.write(13);
  Serial4.printf("di");
  Serial4.write(13);
  
  Serial.printf("di");
  Serial.write(13);
}

void command_la(int x_mm, int y_mm, int z_mm) // go to absolute position
{
  // Motor X
  Serial2.printf("la");
  Serial2.printf("%d",x_mm*incPERmm_x);
  Serial2.write(13);
  Serial2.printf("M");
  Serial2.write(13);
  
  // Motor Y
  Serial3.printf("la");
  Serial3.printf("%d",y_mm*incPERmm_y);
  Serial3.write(13);
  Serial3.printf("M");
  Serial3.write(13);

  // Motor Z
  Serial4.printf("la");
  Serial4.printf("%d",z_mm*incPERmm_z*invert_z_axis);  // Z-values have to be inverted
  Serial4.write(13);
  Serial4.printf("M");
  Serial4.write(13);

  Serial.printf("Command LA Xpos %d mm Ypos %d mm Zpos %d mm\r\n",x_mm,y_mm,z_mm*invert_z_axis);
}

void command_lr(int dx_mm, int dy_mm, int dz_mm) // go to relative position
{
  // Motor X
  Serial2.printf("lr");
  Serial2.printf("%d",dx_mm*incPERmm_x);
  Serial2.write(13);
  Serial2.printf("M");
  Serial2.write(13);
  
  // Motor Y
  Serial3.printf("lr");
  Serial3.printf("%d",dy_mm*incPERmm_y);
  Serial3.write(13);
  Serial3.printf("M");
  Serial3.write(13);
  
  // Motor Z
  Serial4.write("lr");
  Serial4.printf("%d",dz_mm*incPERmm_z*invert_z_axis);
  Serial4.write(13);
  Serial4.write("M");
  Serial4.write(13);
  
  Serial.printf("Command LR dXpos %d mm dYpos %d mm dZpos %d mm\r\n",dx_mm,dy_mm,dz_mm*invert_z_axis);
}

void command_gohoseq()
{

  Serial2.printf("gohoseq");
  Serial2.write(13);
  
  Serial3.printf("gohoseq");
  Serial3.write(13);
  
  Serial4.printf("gohoseq");
  Serial4.write(13);

  Serial.println("Command gohoseq");
}

void command_sp(int vx_mms_max, int vy_mms_max, int vz_mms_max) // set max speed
{
  int vx_rpm_max = float(vx_mms_max) / mmPERsPERrpm_x;
  int vy_rpm_max = float(vx_mms_max) / mmPERsPERrpm_y;
  int vz_rpm_max = float(vx_mms_max) / mmPERsPERrpm_z;

  // Motor X
  Serial2.printf("sp");
  Serial2.printf("%d",vx_rpm_max);
  Serial2.write(13);
  
  // Motor Y
  Serial3.printf("sp");
  Serial3.printf("%d",vy_rpm_max);
  Serial3.write(13);

  // Motor Z
  Serial4.printf("sp");
  Serial4.printf("%d",vz_rpm_max);
  Serial4.write(13);

  // Debug
  Serial.printf("Command Speedmax vx %d mm/s, v_y %d mm/s, v_z %d mm/s\r\n", vx_mms_max, vy_mms_max, vz_mms_max);
  Serial.printf("Command Speedmax vx %d rpm, v_y %d rpm, v_z %d rpm\r\n", vx_rpm_max, vy_rpm_max, vz_rpm_max);
}

void command_v(int vx_mms, int vy_mms, int vz_mms)
{
  int vx_rpm = float(vx_mms) / mmPERsPERrpm_x;
  int vy_rpm = float(vy_mms) / mmPERsPERrpm_y;
  int vz_rpm = float(vz_mms) / mmPERsPERrpm_z;


  // Motor X
  Serial2.printf("v");
  Serial2.printf("%d",vx_rpm);
  Serial2.write(13);
  
  // Motor Y
  Serial3.printf("v");
  Serial3.printf("%d",vy_rpm);
  Serial3.write(13);
  
  // Motor Z
  Serial4.printf("v");
  Serial4.printf("%d",vz_rpm*invert_z_axis);  // z-values have to be inverted due to mounting
  Serial4.write(13);

  Serial.printf("Command V Xvel %d mm/s Yvel %d mm/s Zvel %d mm/s\r\n",vx_mms,vy_mms,vz_mms);
  Serial.printf("Command V Xvel %d rpm Yvel %d rpm Zvel %d rpm\r\n",vx_rpm,vy_rpm,vz_rpm*invert_z_axis);
}

void setup() 
{  
  pinMode(led_pin, OUTPUT);  
  
  // Debug-Port / USB
  Serial.begin(115200);

  if (CommunicationMode==gantry_control)
  {
    // External Computer
    Serial1.setRX(0);
    Serial1.setTX(1);
    Serial1.begin(57600);
  }
  else // for all feedthrough modes
  {
    // External Computer
    Serial1.setRX(0);
    Serial1.setTX(1);
    Serial1.begin(19200);  // has to be the same as the motor baudrate
  }
  
  
  // Motor X
  Serial2.setRX(7);
  Serial2.setTX(8);
  Serial2.begin(19200,SERIAL_8N1);

  // Motor Y
  Serial3.setRX(15);
  Serial3.setTX(14);
  Serial3.begin(19200,SERIAL_8N1);

  // Motor Z
  Serial4.setRX(16);
  Serial4.setTX(17);
  Serial4.begin(19200,SERIAL_8N1);

  FlexiTimer2::set(1, 1.0/1000, ControllerStep); // set(unit, resolution(1/xxx s) ) hz = unit*resolution
  FlexiTimer2::start();

  //command_di();
  command_v(0, 0, 0);
  //command_en();
}

void transmit_data()
{
  // transmission format:
  // x_pos_mm,x_vel_mms,y_pos_mm,y_vel_mms,z_pos_mm,z_vel_mms\r\n

  // Send data to external Computer via Serial1
  Serial1.printf("%d,%d,%d,%d,%d,%d\r\n",motx_pos_mm,motx_vel_mms, moty_pos_mm,moty_vel_mms, motz_pos_mm,motz_vel_mms);

  // Send data to Debug-Port
  Serial.printf("%d,%d,%d,%d,%d,%d\r\n",motx_pos_mm,motx_vel_mms, moty_pos_mm,moty_vel_mms, motz_pos_mm,motz_vel_mms);
  
  // set Flags for sent Data
  MotXPosState = sent;
  MotXVelState = sent;

  MotYPosState = sent;
  MotYVelState = sent;

  MotZPosState = sent;
  MotZVelState = sent;
  transmit_rate_counter++;
}
  

void loop() 
{
  if (CommunicationMode==gantry_control)
  {
     // Request pos/vel data from motor
     if (trigger && todo)
     {
      // Motor X
      if (MotXPosState != avail) getPosMotX();
      if (MotXVelState != avail) getVelMotX();
      
      // Motor Y
      if (MotYPosState != avail) getPosMotY();
      if (MotYVelState != avail) getVelMotY();

      // Motor Z
      if (MotZPosState != avail) getPosMotZ();
      if (MotZVelState != avail) getVelMotZ();
      todo = false;
    }

    // transmit pos/vel data to ext. computer IF ALL are available
    if ((MotXPosState == avail) && 
        (MotXVelState == avail) &&
        (MotYPosState == avail) &&
        (MotYVelState == avail) &&
        (MotZPosState == avail) &&
        (MotZVelState == avail))

    {
      transmit_data();  // transmit pos/vel data to ext. computer via serial1
      counter = counter + 1;
    }

    if (NeuerBefehl)
    { 
      NeuerBefehl=false;
      switch (PCCommand)
        {
          case en: // enable drives
              command_en();
              PCCommand=nix;
          break;
            
          case di:  // disable drives
              command_di();
              PCCommand=nix;
          break;

          case sp: // set max speed
              command_sp(CommandArgument_int[0],CommandArgument_int[1],CommandArgument_int[2]);   
              PCCommand=nix;
          break;

          case gohoseq:  // start homing sequence
              command_gohoseq();
              PCCommand=nix;
          break;
          
          case la:  // go to absolute position
              command_la(CommandArgument_int[0],CommandArgument_int[1],CommandArgument_int[2]);
              PCCommand=nix;
          break;

          case lr:  // go to relative position
              command_lr(CommandArgument_int[0],CommandArgument_int[1],CommandArgument_int[2]);
              PCCommand=nix;
          break;
            
          case v:  // set target velocity
              command_v(CommandArgument_int[0],CommandArgument_int[1],CommandArgument_int[2]);
              PCCommand=nix;
          break;

          case nix:
          break;
        }
    }  
  }
  else // nothing to do for feedtrhough mode
  {

  }
}