#ifndef EV3_reg_h_
#define EV3_reg_h_
#define   LOWEST_BITRATE                2400  //  Lowest possible bit rate (always used for sync and info)  [b/S]
#define   MIDDLE_BITRATE                57600  //  Highest bit rate allowed when adjusting clock             [b/S]
#define   HIGHEST_BITRATE             460800  //  Highest possible bit rate                                 [b/S]
#define   SELECTED_BAUD              MIDDLE_BITRATE
#define   SENSOR_TYPE                 31 // IMU
#define   Timeout_ACK                   2000//ms
/*
  SEQUENCE WHEN UART DEVICE IS DETECTED
  =====================================

        HOST                                                                                DEVICE
    ------------------------------------------------------------        ----------------------------------------------------------------------
                                                                                        - Reset                 <----------------------------,
                                                                                          - Set TX active (low)                              |
                                                                                          - Set RX floating                                  |
                                                                                          - Wait half a second 1st.time (next time >=10mS)   |
                                                                                                                                             |
    - Enable UART communication                                                         - Enable UART communication                          |
      - Setup UART for LOWEST_BITRATE                                                     - Setup UART for LOWEST_BITRATE                    |
      - Setup hardware buffers                                                                                                               |
                                                                                                                                             |
                                                                                                                                             |
   
                                                                                                                                             |
    - Exchange informations                                                             - Exchange informations                              |
      - Receive command data                                         <-- CMD              - Send command data (type,modes,speed, etc)        |
      - Receive info data                                            <-- INFO             - Send info data (name,scaling,data format, etc)   |
      - Receive acknowledge                                          <-- ACK              - When finished info send acknowledge              |
                                                                                          - Timeout (80mS)      -----------------------------'
      - When finished info send acknowledge                 ACK  -->                      - Receive acknowledge
      - Switch to valid communication speed                                               - Switch to valid communication speed


    - Communication running                                                              - Communication running
      - Receive data                                                 <-- DATA             - Send data
      - If out of sync, send not acknowledge                NACK -->                      - If not acknowledge, repeat send data
      - Receive data                                                 <-- DATA             - Send data
      - Receive data                                                 <-- DATA             - Send data
      - Receive data                                                 <-- DATA             - Send data
         --
      - Send data                                           DATA -->                      - Receive data
      - Receive data                                                 <-- DATA             - Send data
      - Receive data                                                 <-- DATA             - Send data
         --
      - Send command                                        CMD  -->                      - Receive command
      - Receive data                                                 <-- DATA             - Send data
      - Receive data                                                 <-- DATA             - Send data
         --
      - Send data                                           DATA -->                      - Receive data
      - If not acknowledge, repeat send data                         <-- NACK             - If out of sync, send not acknowledge
      - Send data                                           DATA -->                      - Receive data
         --


  DEVICES WITH XTAL
  =================

  Devices with a bit rate accuracy better that +-2% must start on LOWEST_BITRATE and skip the synchronisation sequence and just begin to send INFO.

  When placed wrong on an output port device TX should continue to transmit SYNC, INFO or DATA to ensure that the host detects
  that connection 6 is low
  \endverbatim
  \ref ExampleUartProtocol "Ex."
  \verbatim


  DEVICES WITHOUT XTAL
  ====================

  It is possible to use the power of the host to adjust to LOWEST_BITRATE if the device is not capable of meeting accuracy
  of the bit rate. The adjustment factor is then used when switching to higher speeds up to MIDLE_BITRATE.

  These devices must start with a bit rate at LOWEST_BITRATE + worst case deviation from LOWEST_BITRATE

  When placed wrong on an output port device TX should continue to transmit SYNC, INFO or DATA to ensure that the host detects
  that connection 6 is low



  DEVICES NOT ABLE TO RECEIVE
  ===========================

  Devices that is not able to receive should send SYNC several times (MIDLE_BITRATE_DEVIATION / MIDLE_BITRATE_INCREAMENT + 1)

  When placed wrong on an output port device TX should continue to transmit SYNC, INFO or DATA to ensure that the host detects
  that connection 6 is low



  HOST
  ====

  The host should check every data message format against the info once sent from the device and send NACK if not valid

  UART devices is connected when system powers up!: the power to input/output ports is powered up when DCM driver is ready to evaluate the devices on
  the ports (ensures that power up sync is executed when system is powered up)



  BIT RATE ADJUSTMENT
  ===================

    I.    Host starts with a bit rate at LOWEST_BITRATE

    II.   Device must start with a bit rate at LOWEST_BITRATE + worst case deviation from LOWEST_BITRATE

    III.  When the SYNC is received host will check it against the correct SYNC byte and if it is wrong the bit rate is raised MIDLE_BITRATE_INCREAMENT

    IV.   If SYNC is received correctly the bit rate is raised additionally MIDLE_BITRATE_FIX and SYNC is sent to the device

    V.    If info says that a higher bit rate is possible it is raised after transmitting ACK on the host and after receiving (or a time) ACK on the device
          (if host has adjusted the bit rate the same factor will be used when raising)




/*! \page UartProtocol UART Device Communication Protocol
 *
 *  <hr size="1"/>
 *
 *  Definition of protocol used when an UART communicating device is detected at an input port.\n
 *
 * \n
 *
 *  The communication used messages that consists of one or more bytes:
 *
 *  \verbatim

  FIRST BYTE IN MESSAGE MEANS
  ===========================

  bit 76543210
      ||
      00LLLCC0  SYS     - System message
      ||||||||
      ||000000  SYNC    - Synchronisation byte
      ||000010  NACK    - Not acknowledge byte
      ||000100  ACK     - Acknowledge byte
      ||LLL110  ESC     - Reserved for future use
      ||
      ||
      01LLLCCC  CMD     - Command message
      ||   |||
      ||   000  TYPE
      ||   001  MODES
      ||   010  SPEED
      ||   011  SELECT
      ||   100  WRITE
      ||   101
      ||   110
      ||   111
      ||
      ||
      10LLLMMM  INFO    - Info message (next byte is command)
      ||   |||
      ||   000  Mode 0 default   (must be last mode in INFO stream to select as default)
      ||   001  Mode 1
      ||   010  Mode 2
      ||   011  Mode 3
      ||   100  Mode 4
      ||   101  Mode 5
      ||   110  Mode 6
      ||   111  Mode 7
      ||
      ||
      11LLLMMM  DATA    - Data message
        |||
        000     Message pay load is 1 byte not including command byte and check byte
        001     Message pay load is 2 bytes not including command byte and check byte
        010     Message pay load is 4 bytes not including command byte and check byte
        011     Message pay load is 8 bytes not including command byte and check byte
        100     Message pay load is 16 bytes not including command byte and check byte
        101     Message pay load is 32 bytes not including command byte and check byte



  Messages From Device
  ====================

    Command messages:

      TYPE      01000000  tttttttt  cccccccc                                                                                    Device type

      MODES     01001001  00000iii  00000jjj  cccccccc                                                                          Number of modes

      SPEED     01010010  ssssssss  ssssssss  ssssssss  ssssssss  cccccccc                                                      Max communication speed

    Info messages:

      NAME      10LLLMMM  00000000  aaaaaaaa  ........  cccccccc                                                                Name of Device in mode MMM

      RAW       10011MMM  00000001  llllllll  llllllll  llllllll  llllllll  hhhhhhhh  hhhhhhhh  hhhhhhhh  hhhhhhhh  cccccccc    Raw value span in mode MMM

      PCT       10011MMM  00000010  llllllll  llllllll  llllllll  llllllll  hhhhhhhh  hhhhhhhh  hhhhhhhh  hhhhhhhh  cccccccc    Percentage span in mode MMM

      SI        10011MMM  00000011  llllllll  llllllll  llllllll  llllllll  hhhhhhhh  hhhhhhhh  hhhhhhhh  hhhhhhhh  cccccccc    SI unit value span in mode MMM

      SYMBOL    10011MMM  00000100  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  aaaaaaaa  cccccccc    SI symbol

      FORMAT    10010MMM  10000000  00nnnnnn  000000ff  0000FFFF  0000DDDD  cccccccc                                            Format of data in mode MMM

    Data messages:

      DATA      11LLLMMM  dddddddd  ........  cccccccc                                                                          Data in format described under INFO MMM


    Messages from the device must follow the above sequence
    Devices with more modes can repeat "Info messages" once for every mode
    Highest "mode number" must be first
    NAME is the first in info sequence and is necessary to initiate a mode info
    FORMAT is last in info sequence and is necessary to complete the modes info
    Other info messages is optional and has a default value that will be used if not provided
    Delay 10 mS between modes (from FORMATx to NAMEy) to allow the informations to be saved in the brick


    # After ACK only DATA is allowed at the moment

    (Simplest device only needs to send: TYPE, NAME, FORMAT - if SPEED, RAW, PCT and SI are defaults)



  Messages To Device
  ==================

    Command messages:

      SPEED     01010010  ssssssss  ssssssss  ssssssss  ssssssss  cccccccc                                                      Max communication speed

      SELECT    01000011  00000mmm  cccccccc                                                                                    Select new mode

      WRITE     01LLL100  dddddddd  ........  cccccccc                                                                          Write 1-23 bytes to device

    # After ACK only SELECT and WRITE is allowed at the moment



  BIT Explanations
  ================

      LLL       = Message pay load bytes not including command byte and check byte
                  000   = 1
                  001   = 2
                  010   = 4
                  011   = 8
                  100   = 16
                  101   = 32

      CCC       = Command
                  000   = TYPE
                  001   = MODES
                  010   = SPEED
                  011   = SELECT

      MMM       = Mode
                  000   = Mode 0 default   (must be last mode in INFO stream to select as default)
                  001   = Mode 1
                  010   = Mode 2
                  011   = Mode 3
                  100   = Mode 4
                  101   = Mode 5
                  110   = Mode 6
                  111   = Mode 7

      iii       = Number of modes
                  000   = Only mode 0      (default if message not received)
                  001   = Mode 0,1
                  010   = Mode 0,1,2
                  011   = Mode 0,1,2,3
                  100   = Mode 0,1,2,3,4
                  101   = Mode 0,1,2,3,4,5
                  110   = Mode 0,1,2,3,4,5,6
                  111   = Mode 0,1,2,3,4,5,6,7

      jjj       = Number of modes in view and data log (default is iii if not received)
                  000   = Only mode 0
                  001   = Mode 0,1
                  010   = Mode 0,1,2
                  011   = Mode 0,1,2,3
                  100   = Mode 0,1,2,3,4
                  101   = Mode 0,1,2,3,4,5
                  110   = Mode 0,1,2,3,4,5,6
                  111   = Mode 0,1,2,3,4,5,6,7

      cccccccc  = Check byte (result of 0xFF exclusively or'ed with all preceding bytes)

      ssssssss  = Speed 4 bytes (ULONG with LSB first)

      tttttttt  = Device type (used together with mode to form VM device type)

      llllllll  = Floating point value for low (RAW/PCT/SI) value (used to scale values)
                  (Default if not received - RAW = 0.0, PCT = 0.0, SI = 0.0)

      hhhhhhhh  = Floating point value for high (RAW/PCT/SI) value (used to scale values)
                  (Default if not received - RAW = 1023.0, PCT = 100.0, SI = 1.0)

      nnnnn     = Number of data sets [1..32 DATA8] [1..16 DATA16] [1..8 DATA32] [1..8 DATAF]

      ff        = Data set format
                  00    = DATA8
                  01    = DATA16
                  10    = DATA32
                  11    = DATAF

      aaaaaaaa  = Device name in ACSII characters (fill with zero '\0' to get to LLL - no zero termination necessary)
                  (Default if not received - empty)

      dddddddd  = Binary data (LSB first)

      mmm       = Mode
                  000   = Mode 0
                  001   = Mode 1
                  010   = Mode 2
                  011   = Mode 3
                  100   = Mode 4
                  101   = Mode 5
                  110   = Mode 6
                  111   = Mode 7

      FFFF      = Total number of figures shown in view and datalog [0..15] (inclusive decimal point and decimals)
                  (Default if not received - 4)

      DDDD      = Number of decimals shown in view and datalog [0..15]
                  (Default if not received - 0)


  \endverbatim \anchor ExampleUartProtocol \verbatim

  Example with info coming from a device with two modes
  =====================================================


      TYPE      01000000  tttttttt  cccccccc                                                type

      MODES     01001001  00000001  00000001  cccccccc                                      Mode 0 and 1, both shown in view

      SPEED     01010010  00000000  11100001  00000000  00000000  cccccccc                  57600 bits/Second


      NAME      10011001  00000000  'L' 'i' 'g' 'h' 't' '\0' '\0' '\0'  cccccccc            "Light"

      RAW       10011001  00000001  0.0 1023.0  cccccccc                                    RAW 0-1023

      SI        10011001  00000011  0.0 1023.0  cccccccc                                    SI 0-1023

      SYMBOL    10011001  00000100  'l' 'x' '\0' '\0' '\0' '\0' '\0' '\0' cccccccc          "lx"

      FORMAT    10010001  10000000  00000001  00000001  00000100  00000000  cccccccc        1 * DATA16, 4 figures, 0 decimals


      NAME      10011000  00000000  'C' 'o' 'l' 'o' 'r' '\0' '\0' '\0'  cccccccc            "Color"

      RAW       10011000  00000001  0.0    6.0  cccccccc                                    RAW 0-6

      SI        10011000  00000011  0.0    6.0  cccccccc                                    SI 0-6

      FORMAT    10010000  10000000  00000001  00000001  00000001  00000000  cccccccc        1 * DATA16, 1 figure, 0 decimals


      ACK


  Error handling
  ==============

  Before ACK:

  IF ANYTHING GOES WRONG - HOST WILL NOT ACK AND DEVICE MUST RESET !


  After ACK

  IF CHECK XOR FAILS - HOST WILL NACK
  IF FORMAT FAILS    - HOST WILL TRY SELECT 5 TIMES

  IF ABOVE FAIL      - ERROR IS SHOWN TO USER



\endverbatim
 *
 *  \n
 */

// FIRST BYTE

#define   BYTE_SYNC                     0x00                            // Synchronisation byte
#define   BYTE_ACK                      0x04                            // Acknowledge byte
#define   BYTE_NACK                     0x02                            // Not acknowledge byte

#define   MESSAGE_SYS                   0x00                            // System message
#define   MESSAGE_CMD                   0x40                            // Command message
#define   MESSAGE_INFO                  0x80                            // Info message
#define   MESSAGE_DATA                  0xC0                            // Data message
#define   GET_MESSAGE_TYPE(B)           (B & 0xC0)                      // Get message type

#define   CMD_TYPE                      0x00                            // CMD command - TYPE     (device type for VM reference)
#define   CMD_MODES                     0x01                            // CMD command - MODES    (number of supported modes 0=1)
#define   CMD_SPEED                     0x02                            // CMD command - SPEED    (maximun communication speed)
#define   CMD_SELECT                    0x03                            // CMD command - SELECT   (select mode)
#define   CMD_WRITE                     0x04                            // CMD command - WRITE    (write to device)
#define   GET_CMD_COMMAND(B)            (B & 0x07)                      // Get CMD command

#define   GET_MODE(B)                   (B & 0x07)                      // Get mode

#define   CONVERT_LENGTH(C)             (1 << (C & 0x07))
#define   GET_MESSAGE_LENGTH(B)         (CONVERT_LENGTH(B >> 3))        // Get message length exclusive check byte

#define   CMD_LLL_SHIFT                 3
#define   MAKE_CMD_COMMAND(C,LC)        (MESSAGE_CMD + (C & 0x07) + ((LC & 0x07) << 3))


// SECOND INFO BYTE

#define   INFO_NAME                     0x00                            // INFO command - NAME    (device name)
#define   INFO_RAW                      0x01                            // INFO command - RAW     (device RAW value span)
#define   INFO_PCT                      0x02                            // INFO command - PCT     (device PCT value span)
#define   INFO_SI                       0x03                            // INFO command - SI      (device SI  value span)
#define   INFO_SYMBOL                   0x04                            // INFO command - SYMBOL  (device SI  unit symbol)
#define   INFO_FORMAT                   0x80                            // INFO command - FORMAT  (device data sets and format)
#define   GET_INFO_COMMAND(B)           (B)                             // Get INFO command

// Data set format
#define   DATA8                         0x00                            // DATA8 Type
#define   DATA16                        0x01                            // DATA16
#define   DATA32                        0x10                            // DATA32
#define   DATAF                         0x11                            // DATAF





#endif

