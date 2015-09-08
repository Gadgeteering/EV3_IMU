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
#include "EV3_Sensor.h"
#include "Arduino.h"
#include "EV3_reg.h"

//#define debugEV3
#ifdef debugEV3
#include <SoftwareSerial.h>
#define rx_pin 8
#define tx_pin 9
SoftwareSerial SoftwareUart(rx_pin, tx_pin);
#endif

EV3_Mode::EV3_Mode() {
}


EV3_Sensor::EV3_Sensor(){
}

void EV3_Sensor::begin(){
  Serial.begin(2400);

  iii=0;
  jjj=0;
}

void EV3_Sensor::end() {
  // Not used but just here to keep it tidy
  Serial.end();
}

void EV3_Sensor::Add_Mode(String Name, boolean Viewable,byte Data_Packets,byte Data_Type, byte Figures_Count, byte Decimal_Places,float SI_low,float SI_high,String Symbol)
  {
  EV3_Mode* Mode = new EV3_Mode();
  Mode->Name = Name;
  Mode->Data_Packets=Data_Packets;
  Mode->Data_Type=Data_Type;
  Mode->Figures_Count=Figures_Count;
  Mode->Decimal_Places;
  Mode->SI_low =SI_low;
  Mode->SI_high=SI_high;
  Mode->Symbol=Symbol;
  Sensor_Info_Array[iii] = Mode;
  iii++;
  if (Viewable) jjj++;

}

#define SerialTx 1
void EV3_Sensor::Send_Info () {
  #ifdef debugEV3
  SoftwareUart.begin(57600);
  SoftwareUart.println("Starting EV3 Info");
  #endif
    delay(500);
boolean ACK_RX = false;
  while(ACK_RX == false){

    Serial.end();
    pinMode(SerialTx,OUTPUT);
    digitalWrite(SerialTx,LOW);
    delay(10);
    Serial.begin(LOWEST_BITRATE);
    //Set Type
    byte Bytes[4];
    byte Baud_Bytes[4];
    Bytes[0] = SENSOR_TYPE;
    Send_CMD(MESSAGE_CMD|CMD_TYPE, Bytes, 1);
    //Number of Modes
    Bytes[0] = iii - 1;
    Bytes[1] = jjj - 1;
    Send_CMD(MESSAGE_CMD |CMD_MODES, Bytes, 2); //LL =1 2bytes
    //Convert Baud to Bytes
    unsigned long Baud_rate =  SELECTED_BAUD;
    Baud_Bytes[0] = 0; //(byte) (Baud_rate);
    Baud_Bytes[1] = (byte) (Baud_rate >> 8);
    Baud_Bytes[2] = (byte) (Baud_rate >> 16);
    Baud_Bytes[3] = (byte) (Baud_rate>> 24);
    Send_CMD(MESSAGE_CMD |CMD_SPEED, Baud_Bytes,4); //LLL = 2 4 Bytes
    for(int MMM=iii-1;MMM>=0;MMM--) {
    EV3_Mode* Mode = Sensor_Info(MMM);
    byte len = Mode->Name.length();    
    byte Payload[len];
    for (int i = 0; i<=len;i++) {
      Payload[i] =0; // Set Buffer to Null
    }
    Mode->Name.getBytes(&Payload[1],len+1);
    Payload[0] = INFO_NAME;  // Name CMD
    //NAME      10011001  00000000  'L' 'i' 'g' 'h' 't' '\0' '\0' '\0'  cccccccc            "Light"
    //10LLLMMM  INFO    - Info message (next byte is command)
    //  MAKE_CMD_COMMAND(C,LC)        (MESSAGE_CMD + (C & 0x07) + ((LC & 0x07) << 3))
    Send_CMD(MESSAGE_INFO  | MMM, Payload, len);// add One for definition
    //Send SI Range
    byte SI_Payload[9];
    SI_Payload[0]=INFO_SI;
    long low = Mode->SI_low;
    SI_Payload[1] = (byte) low & 0xff;
    SI_Payload[2] = (byte) (low >> 8);
    SI_Payload[3] = (byte) (low >> 16);
    SI_Payload[4] = (byte) (low >> 24);
    long high = Mode->SI_high;
    SI_Payload[5] = (byte) high & 0xff;
    SI_Payload[6] = (byte) (high >> 8);
    SI_Payload[7] = (byte) (high >> 16);
    SI_Payload[8] = (byte) (high >> 24);
    Send_CMD(MESSAGE_INFO | MMM, SI_Payload, 8);
    //Send Symbol
    byte Symbol_Payload[]= {0,0,0,0,0,0,0,0,0};
    Symbol_Payload[0] = INFO_SYMBOL;
    byte len_symbol = Mode->Symbol.length();    
    if (len_symbol>8) len_symbol=8;
    Mode->Symbol.getBytes(&Symbol_Payload[1],len_symbol+1);
    Send_CMD(MESSAGE_INFO  | MMM, Symbol_Payload, 8);
    
    
    //Send FORMAT of Number 
    byte Format_Payload[5];
    Format_Payload[0] = INFO_FORMAT;
    Format_Payload[1] = Mode->Data_Packets;
    Format_Payload[2] = Mode->Data_Type;
    Format_Payload[3] = Mode->Figures_Count;
    Format_Payload[4] = Mode->Decimal_Places;
    //FORMAT    10010000  10000000  00000001  00000001  00000001  00000000  cccccccc        1 * DATA16, 1 figure, 0 decimals
    Send_CMD(MESSAGE_INFO  | MMM, Format_Payload, 4);  
    } //End of Sending Info data
    
    Serial.write(BYTE_ACK);

    unsigned long last_ACK = millis();
    while((Serial.available()==0) && ((millis() - last_ACK) < Timeout_ACK));
    if (Serial.available()>0) {
    
    byte response = Serial.read();
#ifdef debugEV3
    SoftwareUart.println("Response"+response);
    SoftwareUart.print("Hex=:");
    SoftwareUart.println(response,HEX);
#endif
    if (response == BYTE_ACK) {
#ifdef debugEV3
       SoftwareUart.println("Change Baud");

#endif
      //ACK_RX=true;
      Serial.end();
      Serial.begin(SELECTED_BAUD);
     
    delay (80);
    break;
    }
    }
  }
 digitalWrite(13, LOW);
#ifdef debugEV3
   SoftwareUart.println("Info Complete");
#endif
}

void EV3_Sensor::watch_dog(){
  if (Serial.available()) {
      byte Response = Serial.read();
#ifdef debugEV3
   SoftwareUart.print("Response=");SoftwareUart.println(Response,HEX);
#endif
   switch(Response){
    case BYTE_NACK:
    last_response=millis();
#ifdef debugEV3
   SoftwareUart.println("NACK");
#endif
    break;
    case MESSAGE_CMD|CMD_SELECT:
    byte Sel_Mode =Serial.read();
    byte Checksum = 0xff ^ Response ^ Sel_Mode;
    if( Checksum == Serial.read()) Selected_Mode=Sel_Mode;
    last_response=millis();
#ifdef debugEV3
   SoftwareUart.print("Selected=");SoftwareUart.print(Sel_Mode,HEX);SoftwareUart.print(" Mode Now=");SoftwareUart.println(Selected_Mode,HEX);
#endif
    break;
   }
}
}


void EV3_Sensor::Send_DATA(byte MMM,int *payload){
EV3_Mode* Mode = Sensor_Info(MMM);
byte data_packets=Mode->Data_Packets;
byte data_type=Mode->Data_Type;
int Multiplier;

if (data_type==DATA8) Multiplier =1;
if (data_type==DATA16) Multiplier =2;
if (data_type==DATA32) Multiplier =4;
if (data_type==DATAF) Multiplier =4; //floating point
byte len=data_packets*Multiplier;
byte payload_bytes[len];
#ifdef debugEV3
String Name=Mode->Name;
   SoftwareUart.print(" Data Type=");SoftwareUart.print(data_type,HEX);SoftwareUart.print(" Data Packets=");SoftwareUart.println(data_packets,DEC);
#endif
//Convert from Data_Type to bytes
for (byte i=0;i<data_packets;i++){
#ifdef debugEV3 
SoftwareUart.print(i,DEC);SoftwareUart.print("i Payload=");SoftwareUart.print(payload[i]); 
#endif
payload_bytes[Multiplier*i]=payload[i]&0xff;
#ifdef debugEV3 
SoftwareUart.print("Data=");SoftwareUart.print(payload_bytes[Multiplier*i],DEC); 
#endif
if (Multiplier>1)  {payload_bytes[Multiplier*i+1]=payload[i]>>8;
#ifdef debugEV3 
SoftwareUart.print(":");SoftwareUart.print(payload_bytes[Multiplier*i+1],DEC); 
#endif
  }
if (Multiplier>2)  {payload_bytes[Multiplier*i+2]=payload[i]>>16;
#ifdef debugEV3 
SoftwareUart.print(":");SoftwareUart.print(payload_bytes[Multiplier*i+2],DEC); 
#endif
  }
if (Multiplier>2)  {payload_bytes[Multiplier*i+3]=payload[i]>>24;
#ifdef debugEV3 
SoftwareUart.print(":");SoftwareUart.print(payload_bytes[Multiplier*i+3],DEC); 
#endif

  }
#ifdef debugEV3 
SoftwareUart.println("--------");SoftwareUart.println(); 
#endif
}

Send_CMD(MESSAGE_DATA  | MMM, payload_bytes, len);  
}
  
byte EV3_Sensor::get_Selected_Mode(){
  return Selected_Mode;
}

unsigned long EV3_Sensor::get_Last_Response(){
  return last_response;
}
///Send CMD
void EV3_Sensor::Send_CMD(byte CMD, byte* DATA, byte LEN) {
  
byte LLL=Convert_to_LLL(LEN); //twice word to bytes
byte data_packets=(Pow2(LLL));// find how many bytes
if ((CMD & 0xC0 )== MESSAGE_INFO ) data_packets++;
#ifdef debugEV3
if ((CMD & 0xC0) == MESSAGE_INFO ) {SoftwareUart.print("Message Detected");SoftwareUart.println(data_packets,DEC);}
#endif
  CMD = CMD| (LLL << CMD_LLL_SHIFT);
#ifdef debugEV3
SoftwareUart.print("CMD=:");SoftwareUart.print(CMD,HEX);
#endif  
  byte Checksum =  CMD ^ 0xff;
#ifdef debugEV3
SoftwareUart.print("Len=:");SoftwareUart.print(LEN,DEC);SoftwareUart.print("Data Packets=:");SoftwareUart.print(data_packets,DEC);SoftwareUart.print("LLL=:");SoftwareUart.println(LLL,DEC);
#endif
  Serial.write(CMD);
  for(int i=0;i<data_packets;i++) {
   Checksum ^= DATA[i];
   Serial.write(DATA[i]);
#ifdef debugEV3
SoftwareUart.print(DATA[i],HEX);SoftwareUart.print(":");
#endif
}
  Serial.write(Checksum);
  #ifdef debugEV3
SoftwareUart.print(Checksum,HEX);SoftwareUart.println("<-Checksum");
#endif
}



byte EV3_Sensor::Convert_to_LLL(int len)
{
    /* LLL       = Message pay load bytes not including command byte and check byte
                  000   = 1
                  001   = 2
                  010   = 4
                  011   = 8
                  100   = 16
                  101   = 32 */
   if (len >16) return 5;
   if (len >8) return 4;
   if (len > 4) return 3;
   if (len >2) return 2;
   if (len == 2) return 1;               
   if (len == 1) return 0;
} 


EV3_Mode* EV3_Sensor::Sensor_Info(byte Mode) {
  return Sensor_Info_Array[Mode];
}

byte EV3_Sensor::Pow2(byte LLL)
{
// Power Base 2 function due error with pow function
byte Pow=1;
  for (byte i=0;i<LLL;i++){
    Pow=Pow*2;
  }
return Pow;
}

