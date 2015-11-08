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

#define debugEV3
#ifdef debugEV3
#define debugEV3_Level1
#include <SoftwareSerial.h>
#define rx_pin 5
#define tx_pin 4
SoftwareSerial SoftwareUart(rx_pin, tx_pin);
#endif

EV3_Mode::EV3_Mode() {
}


EV3_Sensor::EV3_Sensor(){
}

void EV3_Sensor::begin(){
  iii=0;
  jjj=0;
 // pinMode(3,OUTPUT);
pinMode(2,OUTPUT);
digitalWrite(2,LOW); 
  
}

void EV3_Sensor::end() {
  // Not used but just here to keep it tidy
  Serial.end();
}
//  Name      DataSets  Format  Figures  Decimals  Views  Conn. Pins  RawMin   RawMax   PctMin  PctMax  SiMin    SiMax    Time  IdValue  Symbol
//   Name                    Type                   Connection     Mode  DataSets  Format  Figures  Decimals  Views   RawMin  RawMax  PctMin  PctMax  SiMin   SiMax   Time   IdValue  Pins Symbol
 

void EV3_Sensor::Add_Mode(String Name, boolean Viewable,byte DataSets,byte Data_Type, byte Figures_Count, byte Decimal_Places,float SI_low,float SI_high,String Symbol)
  {
  EV3_Mode* Mode = new EV3_Mode();
  Mode->Name = Name;
  Mode->DataSets=DataSets;
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
    digitalWrite(2,HIGH);
    delay(200);
    digitalWrite(2,LOW); 
boolean ACK_RX = false;
  while(ACK_RX == false){
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
    float low =Mode->SI_low;
    memcpy(&SI_Payload[1], (unsigned char*) (&low), 4);
    float high = Mode->SI_high;
    memcpy(&SI_Payload[5], (unsigned char*) (&high), 4);
    //Send_CMD(MESSAGE_INFO | MMM, SI_Payload, 8);
    
    //Send Symbol
    byte Symbol_Payload[]= {0,0,0,0,0,0,0,0,0};
    Symbol_Payload[0] = INFO_SYMBOL;
    byte len_symbol = Mode->Symbol.length();    
    if (len_symbol>8) len_symbol=8;
    Mode->Symbol.getBytes(&Symbol_Payload[1],len_symbol+1);
    //Send_CMD(MESSAGE_INFO  | MMM, Symbol_Payload, 8);
    
    //Send FORMAT of Number 
    byte Format_Payload[5];
    Format_Payload[0] = INFO_FORMAT;
    Format_Payload[1] = Mode->DataSets;
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
#ifdef debugEV3_Level2
    SoftwareUart.println("Response"+response);
    SoftwareUart.print("Hex=:");
    SoftwareUart.println(response,HEX);
#endif
    if (response == BYTE_ACK) {
#ifdef debugEV3_Level2
       SoftwareUart.println("Change Baud");
#endif
      //ACK_RX=true;
      Serial.end();
      Serial.begin(SELECTED_BAUD);
    delay (80);
    break;
    }
    else {
Serial.end();
pinMode(SerialTx,OUTPUT);
digitalWrite(SerialTx,LOW);

delay(10);
    }
    }
    delay(500);
  }

#ifdef debugEV3
   SoftwareUart.println("Info Complete");
#endif
}

void EV3_Sensor::watch_dog(){
  if (Serial.available()) {
      Response = Serial.read();
#ifdef debugEV3
   SoftwareUart.print("Response=");SoftwareUart.println(Response,HEX);
#endif
    Data_Read=false;
   switch(Response){
    case BYTE_NACK:
    last_response=millis();
     Data_Read=true;
#ifdef debugEV3 //_Level2
   SoftwareUart.println("NACK");
#endif
    break;
    case MESSAGE_CMD|CMD_SELECT:
    byte Sel_Mode =Serial.read();
    byte Checksum = 0xff ^ Response ^ Sel_Mode;
    if( Checksum == Serial.read()) Selected_Mode=Sel_Mode;
    last_response=millis();
      Data_Read=true;
#ifdef debugEV3
   SoftwareUart.print("Selected=");SoftwareUart.print(Sel_Mode,HEX);SoftwareUart.print(" Mode Now=");SoftwareUart.println(Selected_Mode,HEX);
#endif
    break;
    
   }
}
else
{
  Data_Read=false;
#ifdef debugEV3_level3
   SoftwareUart.println("No Response");
#endif 
}
}


/*! \page cInput
 *  <hr size="1"/>
 *  <b>     opINPUT_READ (LAYER, NO, TYPE, MODE, PCT)  </b>
 *
 *- Read device value in Percent\n
 *- Dispatch status unchanged
 *
 *  \param  (DATA8)   LAYER   - Chain layer number [0..3]
 *  \param  (DATA8)   NO      - Port number
 *  \param  (DATA8) \ref types "TYPE" - Device type (0 = don't change type)
 *  \param  (DATA8)   MODE    - Device mode [0..7] (-1 = don't change mode)
 *  \return (DATA8)   PCT     - Percent value from device
 */
/*! \brief  opINPUT_READ byte code
 * 
 */


/*! \page cInput
 *  <hr size="1"/>
 *  <b>     opINPUT_READSI (LAYER, NO, TYPE, MODE, SI)  </b>
 *
 *- Read device value in SI units\n
 *- Dispatch status unchanged
 *
 *  \param  (DATA8)   LAYER   - Chain layer number [0..3]
 *  \param  (DATA8)   NO      - Port number
 *  \param  (DATA8) \ref types "TYPE" - Device type (0 = don't change type)
 *  \param  (DATA8)   MODE    - Device mode [0..7] (-1 = don't change mode)
 *  \return (DATAF)   SI      - SI unit value from device
 */
/*! \brief  opINPUT_READSI byte code
 *
 */
void EV3_Sensor::Send_DATA(byte MMM,long *payload){
EV3_Mode* Mode = Sensor_Info(MMM);
byte DataSets=Mode->DataSets;
byte data_type=Mode->Data_Type;
int Multiplier;

if (data_type==DATA_8) Multiplier =1;
if (data_type==DATA_PCT) Multiplier =1;
if (data_type==DATA_16) Multiplier =2;
if (data_type==DATA_32) Multiplier =4;
if (data_type==DATA_F)  Multiplier =4; //floating point
if (data_type==DATA_SI) Multiplier =4; //Same as DATAF
if (data_type==DATA_F)  Multiplier =4; //floating point

byte len=DataSets*Multiplier;
byte payload_bytes[len];
#ifdef debugEV3_Level2
String Name=Mode->Name;
SoftwareUart.print(" Data Type=");SoftwareUart.print(data_type,HEX);SoftwareUart.print(" Data Packets=");SoftwareUart.println(DataSets,DEC);
#endif


//Convert from Data_Type to bytes
for (byte i=0;i<DataSets;i++){
memcpy(&payload_bytes[i*Multiplier], (unsigned char*) (&payload[i]), Multiplier);
}

Send_CMD(MESSAGE_DATA  | MMM, payload_bytes, len);  
}
  
byte EV3_Sensor::get_Selected_Mode(){
  return Selected_Mode;
}

boolean EV3_Sensor::get_Data_Read(){
  return Data_Read;
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
#ifdef debugEV3_Level1
SoftwareUart.print("Len=:");SoftwareUart.print(LEN,DEC);SoftwareUart.print("Data Packets=:");SoftwareUart.print(data_packets,DEC);SoftwareUart.print("LLL=:");SoftwareUart.println(LLL,DEC);
#endif
  Serial.write(CMD);

  for(int i=0;i<data_packets;i++) {
   Checksum ^= DATA[i];
   Serial.write(DATA[i]);
#ifdef debugEV3_Level1
SoftwareUart.print(DATA[i],HEX);SoftwareUart.print(":");
#endif
}
  Serial.write(Checksum);
  #ifdef debugEV3_Level1
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

