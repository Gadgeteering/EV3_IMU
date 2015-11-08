//#define SoftUart
#ifdef SoftUart
//#define verbose
#endif

/******************************************************************************
EV3 IMU
EV3_Imu file V0.2
Peter Hunt @ Gadgeteering
Original Creation Date: August 15th 2015
Revised Date: 8th November 2015
https://github.com/

This file implements an inferface between the Lego EV3 to the LSM9DS0

Development environment specifics:
  IDE: Arduino 1.6.5
  Hardware Platform: Arduino Micro 5V/16MHz
  
 EV3 Arduino Breakout Board

ToDo List:

Stop sending inform without NoAck


Distributed as-is; no warranty is given.
******************************************************************************/


#include <Wire.h>
#include <SPI.h>
#include <SFE_LSM9DS0.h>


#ifdef SoftUart
#define rx_pin 5
#define tx_pin 4
#include <SoftwareSerial.h>
SoftwareSerial SoftwareUart(rx_pin, tx_pin);
#endif

#include "EV3_Reg.h"
#include "EV3_Sensor.h"

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
const byte INT1XM = 8; // INT1XM tells us when accel data is ready
const byte INT2XM = 9; // INT2XM tells us when mag data is ready
const byte DRDYG  = 7; // DRDYG  tells us when gyro data is ready

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f


float pitch, yaw, roll, heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float temperature;
long Payload32[8];

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
//EV3 Declarartions
EV3_Sensor IMU;
//EV3 Values
unsigned long last_reading = 0;
float data =-90;


#define SerialTx 1
void setup() {
pinMode(SerialTx,INPUT);
  // begin() returns a 16-bit value which includes both the gyro 
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.

#ifdef SoftUart
SoftwareUart.begin(57600);
SoftwareUart.println("Software Uart Started");
#endif

uint32_t status = dof.begin(); //Start LSM9DS0

#ifdef verbose
SoftwareUart.print("LSM9DS0 WHO_AM_I's returned: 0x");
SoftwareUart.println(status, HEX);
SoftwareUart.println("Should be 0x49D4");
#endif

  // Set data output ranges; choose lowest ranges for maximum resolution
 // Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
    dof.setAccelScale(dof.A_SCALE_2G);
 // Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
    dof.setGyroScale(dof.G_SCALE_245DPS);
 // Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
    dof.setMagScale(dof.M_SCALE_2GS); 
 // Set output data rates  
 // Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50, 
 //                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)

    dof.setAccelODR(dof.A_ODR_200); // Set accelerometer update rate at 100 Hz
 // Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
 // Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
 // Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
 // to get a smooth ~150 Hz filter update rate
    dof.setAccelABW(dof.A_ABW_50); // Choose lowest filter setting for low noise
 // Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
 //                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
    dof.setGyroODR(dof.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

 // Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
    dof.setMagODR(dof.M_ODR_125); // Set magnetometer to update every 80 ms
    
 // Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
 // all subsequent measurements.
    dof.calLSM9DS0(gbias, abias);
  
     // Setup EV3 Protocol   
//Name Length Max 10
IMU.begin(); // Start EV3 Sensor Type 31 Baud Rate 57600
IMU.Add_Mode("Pitch",true,2,DATA_SI,3,1,0.0,1023.0,"Deg");//0
IMU.Add_Mode("Heading",true,1,DATA_SI,3,1,0.0,1023.0,"Deg");//1
IMU.Add_Mode("Gyro Rate",true,4,DATA_SI,3,1,0.0,1023.0,"Deg/S");//2
IMU.Add_Mode("Accelerom",true,4,DATA_SI,3,1,0.0,1023.0,"G");//3
IMU.Add_Mode("Magnetome",true,4,DATA_SI,3,1,0.0,1023.0,"T");//4
IMU.Send_Info();

dof.calLSM9DS0(gbias, abias);// Don't known why but end up with xgyro bias of about 20 deg/s

}


     


void loop() {

 // if((digitalRead(DRDYG)) == LOW) 
 if (bitRead(3,(dof.gReadByte(STATUS_REG_G)))==HIGH)  {  // When new gyro data is ready
    dof.readGyro();           // Read raw gyro data
  
    gx = dof.calcGyro(dof.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
    gy = dof.calcGyro(dof.gy) - gbias[1];
    gz = dof.calcGyro(dof.gz) - gbias[2];
 }
  
 // if((digitalRead(INT1XM)) == LOW) 
 if (bitRead(3,(dof.xmReadByte(STATUS_REG_A)))==HIGH) {  // When new accelerometer data is ready
    dof.readAccel();         // Read raw accelerometer data
    ax = dof.calcAccel(dof.ax) - abias[0];   // Convert to g's, remove accelerometer biases
    ay = dof.calcAccel(dof.ay) - abias[1];
    az = dof.calcAccel(dof.az) - abias[2];

  }
  
 // if((digitalRead(INT2XM)) == LOW) 
 if (bitRead(3,(dof.xmReadByte(STATUS_REG_M)))==HIGH) {  // When new magnetometer data is ready
    dof.readMag();           // Read raw magnetometer data
    mx = dof.calcMag(dof.mx);     // Convert to Gauss and correct for calibration
    my = dof.calcMag(dof.my);
    mz = dof.calcMag(dof.mz);
    
    dof.readTemp();
    temperature = 21.0 + (float) dof.temperature/8.; // slope is 8 LSB per degree C, just guessing at the intercept
  }

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s


MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

// MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);  
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination), 
  // looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    
  
   #ifdef verbose
    SoftwareUart.print("gx = "); SoftwareUart.print( gx, 2); 
    SoftwareUart.print(" gy = "); SoftwareUart.print( gy, 2); 
    SoftwareUart.print(" gz = "); SoftwareUart.print( gz, 2); SoftwareUart.println(" deg/s");
    
    SoftwareUart.print("ax = "); SoftwareUart.print(ax,3);  
    SoftwareUart.print(" ay = "); SoftwareUart.print(ay,3); 
    SoftwareUart.print(" az = "); SoftwareUart.print(az,3);
    SoftwareUart.println(" mg");
    
    SoftwareUart.print("mx = "); SoftwareUart.print( mx,3); 
    SoftwareUart.print(" my = "); SoftwareUart.print( my,3); 
    SoftwareUart.print(" mz = "); SoftwareUart.print( mz,3); SoftwareUart.println(" mG");
    SoftwareUart.print("temperature = "); SoftwareUart.println(temperature, 2);
  
 
    SoftwareUart.print("q0 = "); SoftwareUart.print(q[0]);
    SoftwareUart.print(" qx = "); SoftwareUart.print(q[1]); 
    SoftwareUart.print(" qy = "); SoftwareUart.print(q[2]); 
    SoftwareUart.print(" qz = "); SoftwareUart.println(q[3]); 
    
    SoftwareUart.print("filter rate = "); SoftwareUart.println(1.0f/deltat, 1);
      SoftwareUart.print("Pitch=");SoftwareUart.print(pitch,2);SoftwareUart.print("Roll=");SoftwareUart.println(roll,2);
    #endif
  

IMU.watch_dog();

if (millis() - IMU.get_Last_Response() > Timeout_ACK) {IMU.Send_Info();

#ifdef SoftUart
  SoftwareUart.print("Time out no Response");
#endif
}

boolean EV3_DataRead=IMU.get_Data_Read();
if (EV3_DataRead==true){
  
#ifdef SoftUart
  SoftwareUart.print("Mode="); SoftwareUart.println(Mode,DEC);
#endif
byte   Mode=IMU.get_Selected_Mode();
   switch(Mode){
    case 0: //Angle X & Y
    //roll=10.2323;
    //pitch=-45.1234;
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    roll  *= 180.0f / PI;
    memcpy(&Payload32[0], (unsigned char*) (&pitch), 4);
    memcpy(&Payload32[1], (unsigned char*) (&roll), 4);
    IMU.Send_DATA(0,Payload32);
#ifdef SoftUart
  SoftwareUart.print("Send Mode 0 Data Pitch=");SoftwareUart.print(pitch,2);SoftwareUart.print("Roll=");SoftwareUart.println(roll,2);
#endif
    break;
    
    case 1: //Heading
    //yaw=123.4567;
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    memcpy(&Payload32[0], (unsigned char*) (&yaw), 4);
    IMU.Send_DATA((byte)1,Payload32);
#ifdef SoftUart

  SoftwareUart.print("Send Mode 1 Data Heading=");SoftwareUart.println(yaw,2);
#endif

    break;
    
    case 2: //Gyro Rate X,Y & Z
    Payload32[3]=0;
    memcpy(&Payload32[0], (unsigned char*) (&gx), 4);
    memcpy(&Payload32[1], (unsigned char*) (&gy), 4);
    memcpy(&Payload32[2], (unsigned char*) (&gz), 4);
    
    IMU.Send_DATA((byte)2,Payload32);
#ifdef SoftUart
  SoftwareUart.print("Send Mode 2 Data Gyro gx,gy,gz=");SoftwareUart.print(gx,3);SoftwareUart.print(":");SoftwareUart.print(gy,3);SoftwareUart.print(":");SoftwareUart.println(gz,3);
#endif
    break;
    case 3: //Acelerometer X, Y & Z
    Payload32[3]=0;
    memcpy(&Payload32[0], (unsigned char*) (&ax), 4);
    memcpy(&Payload32[1], (unsigned char*) (&ay), 4);
    memcpy(&Payload32[2], (unsigned char*) (&az), 4);
    IMU.Send_DATA((byte)3,Payload32);
#ifdef SoftUart
  SoftwareUart.print("Send Mode 3 Data Accelerometer ax,ay,az=");SoftwareUart.print(ax,3);SoftwareUart.print(":");SoftwareUart.print(ay,3);SoftwareUart.print(":");SoftwareUart.println(az,3);
#endif
    break;
    case 4: //Magnetometer X, Y & Z
    Payload32[0]=mx;
    Payload32[1]=my;
    Payload32[2]=mz;
    memcpy(&Payload32[0], (unsigned char*) (&mx), 4);
    memcpy(&Payload32[1], (unsigned char*) (&my), 4);
    memcpy(&Payload32[2], (unsigned char*) (&mz), 4);
    IMU.Send_DATA((byte)4,Payload32);
  #ifdef SoftUart
  SoftwareUart.print("Send Mode 4 Data Magnetometer mx,my,mz=");SoftwareUart.print(mx,3);SoftwareUart.print(":");SoftwareUart.print(my,3);SoftwareUart.print(":");SoftwareUart.println(mz,3);
#endif
    break;
    default:
    break;   
   }
  
}
}


 


/*

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

*/

/*

 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones.  

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }
        */

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }

