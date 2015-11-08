#ifndef EV3_reg_h_
#define EV3_reg_h_
#define   LOWEST_BITRATE                2400  //  Lowest possible bit rate (always used for sync and info)  [b/S]
#define   MIDDLE_BITRATE                57600  //  Highest bit rate allowed when adjusting clock             [b/S]
#define   HIGHEST_BITRATE             460800  //  Highest possible bit rate                                 [b/S]
#define   SELECTED_BAUD              MIDDLE_BITRATE
/* Type No     Description
    --------    ----------------------------------------------------
    0           "Don't change type" type
    1..50       Reserved for LEGO existing and future devices
    51..100     Free to 3th. party devices
    101..127    Reserved for internal use
*/
#define   SENSOR_TYPE                 31 // IMU
#define   Timeout_ACK                   2000//ms
/*
 * 
 * //                DEVICE MAPPING
//
// Device         0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31
//
// Layer          0   0   0   0   1   1   1   1   2   2   2   2   3   3   3   3   0   0   0   0   1   1   1   1   2   2   2   2   3   3   3   3
// Port (INPUT)   0   1   2   3   0   1   2   3   0   1   2   3   0   1   2   3   16  17  18  19  16  17  18  19  16  17  18  19  16  17  18  19
// Port (OUTPUT)  -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   0   1   2   3   0   1   2   3   0   1   2   3   0   1   2   3
// Output         0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1

 */
 /*! \page cInput
 *  <hr size="1"/>
 *  <b>     opINPUT_DEVICE (CMD, .....)  </b>
 *
 *- Read information about device\n
 *- Dispatch status unchanged
 *
 *  \param  (DATA8)   CMD               - \ref inputdevicesubcode
 *
 *\n
 *  - CMD = GET_TYPEMODE
 *\n  Get device type and mode\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \return (DATA8) \ref types "TYPE" - Device type
 *    -  \return (DATA8)   MODE         - Device mode [0..7]
 *
 *\n
 *  - CMD = GET_NAME
 *\n  Get device name\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \param  (DATA8)   LENGTH       - Maximal length of string returned (-1 = no check)\n
 *    -  \return (DATA8)   DESTINATION  - String variable or handle to string\n
 *
 *\n
 *  - CMD = GET_SYMBOL
 *\n  Get device symbol\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \param  (DATA8)   LENGTH       - Maximal length of string returned (-1 = no check)\n
 *    -  \return (DATA8)   DESTINATION  - String variable or handle to string\n
 *
 *\n
 *  - CMD = GET_FORMAT
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \return (DATA8)   DATASETS     - Number of data sets\n
 *    -  \return (DATA8)   FORMAT       - Format [0..3]\n
 *    -  \return (DATA8)   MODES        - Number of modes [1..8]\n
 *    -  \return (DATA8)   VIEWS        - Number of views [1..8]\n
 *
 *\n
 *  - CMD = GET_RAW
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \return (DATA32)  VALUE        - 32 bit raw value\n
 *
 *\n
 *  - CMD = GET_MODENAME
 *\n  Get device mode name\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \param  (DATA8)   MODE         - Mode\n
 *    -  \param  (DATA8)   LENGTH       - Maximal length of string returned (-1 = no check)\n
 *    -  \return (DATA8)   DESTINATION  - String variable or handle to string\n
 *
 *\n
 *  - CMD = GET_FIGURES
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \return (DATA8)   FIGURES      - Total number of figures (inclusive decimal point and decimals\n
 *    -  \return (DATA8)   DECIMALS     - Number of decimals\n
 *
 *\n
 *  - CMD = GET_MINMAX
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \return (DATAF)   MIN          - Min SI value\n
 *    -  \return (DATAF)   MAX          - Max SI value\n
 *
 *\n
 *  - CMD = READY_PCT
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \param  (DATA8) \ref types "TYPE" - Device type (0 = don't change type)
 *    -  \param  (DATA8)   MODE         - Device mode [0..7] (-1 = don't change mode)
 *    -  \param  (DATA8)   VALUES       - Number of return values
 *
 *       if (VALUES == 1)
 *       \return (DATA8)   VALUE1       - First value from input
 *
 *\n
 *  - CMD = READY_RAW
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \param  (DATA8) \ref types "TYPE" - Device type (0 = don't change type)
 *    -  \param  (DATA8)   MODE         - Device mode [0..7] (-1 = don't change mode)
 *    -  \param  (DATA8)   VALUES       - Number of return values
 *
 *       if (VALUES == 1)
 *       \return (DATA32)  VALUE1       - First value from input
 *
 *\n
 *  - CMD = READY_SI
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \param  (DATA8) \ref types "TYPE" - Device type (0 = don't change type)
 *    -  \param  (DATA8)   MODE         - Device mode [0..7] (-1 = don't change mode)
 *    -  \param  (DATA8)   VALUES       - Number of return values
 *
 *       if (VALUES == 1)
 *       \return (DATAF)   VALUE1       - First value from input
 *
 *\n
 *  - CMD = GET_CHANGES
 *\n  Get positive changes since last clear\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \return (DATAF)   VALUE        - Positive changes since last clear\n
 *
 *\n
 *  - CMD = GET_BUMPS
 *\n  Get negative changes since last clear\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \return (DATAF)   VALUE        - Negative changes since last clear\n
 *
 *\n
 *  - CMD = CLR_CHANGES
 *\n  Clear changes and bumps\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *
 *\n
 *  - CMD = CAL_MINMAX
 *\n  Apply new minimum and maximum raw value for device type to be used in scaling PCT and SI\n
 *    -  \param  (DATA8) \ref types "TYPE" - Device type [1..101]
 *    -  \param  (DATA8)   MODE         - Device mode [0..7]
 *    -  \param  (DATA32)  CAL_MIN      - 32 bit raw minimum value (Zero)\n
 *    -  \param  (DATA32)  CAL_MAX      - 32 bit raw maximum value (Full scale)\n
 *
 *\n
 *  - CMD = CAL_MIN
 *\n  Apply new minimum raw value for device type to be used in scaling PCT and SI\n
 *    -  \param  (DATA8) \ref types "TYPE" - Device type [1..101]
 *    -  \param  (DATA8)   MODE         - Device mode [0..7]
 *    -  \param  (DATA32)  CAL_MIN      - 32 bit SI minimum value (Zero)\n
 *
 *\n
 *  - CMD = CAL_MAX
 *\n  Apply new maximum raw value for device type to be used in scaling PCT and SI\n
 *    -  \param  (DATA8) \ref types "TYPE" - Device type [1..101]
 *    -  \param  (DATA8)   MODE         - Device mode [0..7]
 *    -  \param  (DATA32)  CAL_MAX      - 32 bit SI maximum value (Full scale)\n
 *
 *\n
 *  - CMD = CAL_DEFAULT
 *\n  Apply the default minimum and maximum raw value for device type to be used in scaling PCT and SI\n
 *    -  \param  (DATA8) \ref types "TYPE" - Device type [1..101]
 *    -  \param  (DATA8)   MODE         - Device mode [0..7]
 *
 *\n
 *  - CMD = SETUP
 *\n  Generic setup/read IIC sensors \ref cinputdevicesetup "Example"\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3]
 *    -  \param  (DATA8)   NO           - Port number
 *    -  \param  (DATA8)   REPEAT       - Repeat setup/read "REPEAT" times  (0 = infinite)
 *    -  \param  (DATA16)  TIME         - Time between repeats [10..1000mS] (0 = 10)
 *    -  \param  (DATA8)   WRLNG        - No of bytes to write
 *    -  \param  (DATA8)   WRDATA       - DATA8 array  (handle) of data to write\n
 *    -  \param  (DATA8)   RDLNG        - No of bytes to read
 *    -  \return (DATA8)   RDDATA       - DATA8 array  (handle) to read into\n
 *
 *\n
 *  - CMD = CLR_ALL
 *\n  Clear all devices (e.c. counters, angle, ...)\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3] (-1 = all)
 *
 *\n
 *  - CMD = STOP_ALL
 *\n  Stop all devices (e.c. motors, ...)\n
 *    -  \param  (DATA8)   LAYER        - Chain layer number [0..3] (-1 = all)
 *
 *\n
 *
 */
/*! \brief  opINPUT_DEVICE byte code
 *
 */
/*! \page cInput
 *  <hr size="1"/>
 *  <b>     opINPUT_READEXT (LAYER, NO, TYPE, MODE, FORMAT, VALUES, VALUE1)  </b>
 *
 *- Read device value\n
 *- Dispatch status unchanged
 *
 *  \param  (DATA8)   LAYER   - Chain layer number [0..3]
 *  \param  (DATA8)   NO      - Port number
 *  \param  (DATA8) \ref types "TYPE" - Device type (0 = don't change type)
 *  \param  (DATA8)   MODE    - Device mode [0..7] (-1 = don't change mode)
 *  \param  (DATA8) \ref formats "FORMAT"  - Format (PCT, RAW, SI ...)
 *  \param  (DATA8)   VALUES  - Number of return values
 *
 *  if (VALUES == 1)
 *  \return (FORMAT)  VALUE1  - First value from device
 */
/*! \brief  opINPUT_READEXT byte code
 *
 */
 /*! \page cInput
 *  <hr size="1"/>
 *  <b>     opINPUT_SAMPLE (TIME, SAMPLES, INIT, DEVICES, TYPES, MODES, DATASETS, VALUES)  </b>
 *
 *- Sample devices (see \ref cinputsample "Example")\n
 *- Dispatch status unchanged
 *
 *  \param  (DATA32)  TIME      - Sample time [mS]
 *  \param  (DATA16)  SAMPLES   - Number of samples
 *  \param  (DATA16)  INIT      - DATA16 array (handle) - to start/reset buffer -> fill array with -1 otherwise don't change
 *  \param  (DATA8)   DEVICES   - DATA8 array  (handle) with devices to sample
 *  \param  (DATA8)   TYPES     - DATA8 array  (handle) with types
 *  \param  (DATA8)   MODES     - DATA8 array  (handle) with modes
 *  \param  (DATA8)   DATASETS  - DATA8 array  (handle) with data sets
 *  \return (DATAF)   VALUES    - DATAF array  (handle) with values
 *
 */
/*! \brief  opINPUT_SAMPLE byte code
 *
 */
/*  SEQUENCE WHEN UART DEVICE IS DETECTED
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

/* Data set format
#define   DATA8                         0x00                            // DATA8 Type
#define   DATA16                        0x01                            // DATA16
#define   DATA32                        0x10                            // DATA32
#define   DATAF                         0x11                            // DATAF
*/

#define   DATA8_NAN     ((DATA8)(-128))
#define   DATA16_NAN    ((DATA16)(-32768))
#define   DATA32_NAN    ((DATA32)(0x80000000))
#define   DATAF_NAN     ((float)0 / (float)0) //(0x7FC00000)

#define   DATA8_MIN     (-127)
#define   DATA8_MAX     (127)
#define   DATA16_MIN    (-32767)
#define   DATA16_MAX    (32767)
#define   DATA32_MIN    (-2147483647)
#define   DATA32_MAX    (2147483647)
#define   DATAF_MIN     (-2147483647)
#define   DATAF_MAX     (2147483647)

typedef   enum
{
  DATA_8        = 0x00,                 //!< DATA8  (don't change)
  DATA_16       = 0x01,                 //!< DATA16 (don't change)
  DATA_32       = 0x02,                 //!< DATA32 (don't change)
  DATA_F        = 0x03,                 //!< DATAF  (don't change)
  DATA_S        = 0x04,                 //!< Zero terminated string
  DATA_A        = 0x05,                 //!< Array handle

  DATA_V        = 0x07,                 //!< Variable type

  DATA_PCT      = 0x10,                 //!< Percent (used in opINPUT_READEXT)/DATA8
  DATA_RAW      = 0x12,                 //!< Raw     (used in opINPUT_READEXT)/DATA32 Format send
  DATA_SI       = 0x13,                 //!< SI unit (used in opINPUT_READEXT)/ DATAF format

  DATA_FORMATS
}
DATA_FORMAT;


#endif

