/******************************************************************************
EV3 Sensor
EV3_Sensor file V1.0
Peter Hunt @ Gadgeteering
Original Creation Date: August 15th 2015
https://github.com/

This file implements an inferface between the Lego EV3 to the LSM9DS0

Development environment specifics:
  IDE: Arduino 1.6.5
  Hardware Platform: Arduino Micro 5V/16MHz
  
 EV3 Arduino Breakout Board


Distributed as-is; no warranty is given.
******************************************************************************/
#ifndef EV3_Sensor_h_
#define EV3_Sensor_h_
#include "Arduino.h"




class EV3_Mode {
  public:
  EV3_Mode();
  String Name;
  byte Data_Packets;
  byte Data_Type;
  byte Figures_Count;
  byte  Decimal_Places;
  //SI        10011MMM  00000011  llllllll  llllllll  llllllll  llllllll  hhhhhhhh  hhhhhhhh  hhhhhhhh  hhhhhhhh  cccccccc    SI unit value span in mode MMM
  long  SI_low,SI_high;
  //SYMBOL    10011MMM  00000100  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  cccccccc    SI symbol
  String Symbol;
  private:
 
  
  
  
};
class EV3_Sensor{
  public:
    EV3_Sensor();
    
      void begin();
      void end();
      void Add_Mode(String Name, boolean Viewable,byte Data_Packets,byte Data_Type, byte Data_Count, byte Decimal_Places,float SI_low,float SI_high,String Symbol);
      void Send_Info();
      void watch_dog();
      byte get_Selected_Mode();
      unsigned long get_Last_Response();
      
      void Send_DATA(byte Mode,int* payload);
      
   //Values
      unsigned long last_response;
  private:
    EV3_Mode* Sensor_Info_Array[8]; 
    EV3_Mode* Sensor_Info(byte mode);
    byte Convert_to_LLL(int);
    int type;
    int Selected_Mode;
    byte iii;
    byte jjj;
    void Send_CMD( byte CMD,byte* DATA,byte LEN);
    byte Pow2(byte LLL);
};


#endif
