/************************************************************
MPU9250_Basic
 Basic example sketch for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This example sketch demonstrates how to initialize the 
MPU-9250, and stream its sensor outputs to a serial monitor.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;

  
float prevAccelX = 0.00,prevAccelY = 0.00,prevAccelZ = 0.00;
unsigned long prevTime;
unsigned long currTime;
float velocityX = 0.00,velocityY = 0.00,velocityZ = 0.00;
float prevVelocityX = 0.00,prevVelocityY = 0.00,prevVelocityZ = 0.00;
float positionX = 0.00,positionY = 0.00;

  
void setup() 
{
  SerialPort.begin(115200);
  imu.begin(); // Initialize the MPU-9250.
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(10); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz
  // Initialize the digital motion processor
  imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | // Send accelerometer data
               DMP_FEATURE_GYRO_CAL       | // Calibrate the gyro data
               DMP_FEATURE_SEND_CAL_GYRO  | // Send calibrated gyro data
               DMP_FEATURE_6X_LP_QUAT     , // Calculate quat's with accel/gyro
               10);                         // Set update rate to 10Hz.
}

void loop() 
{
  if ( imu.fifoAvailable() ) // Check for new data in the FIFO
  {
    // Use dmpUpdateFifo to update the ax, gx, qx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS )
    {
      // The following variables will have data from the top of the FIFO:
      // imu.ax, imu.ay, imu.az, -- Accelerometer
      // imu.gx, imu.gy, imu.gz -- calibrated gyroscope
      // and imu.qw, imu.qx, imu.qy, and imu.qz -- quaternions
      imu.computeEulerAngles();
      if ( imu.dataReady() )
      {
        // Call update() to update the imu objects sensor data.
        // You can specify which sensors to update by combining
        // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
        // UPDATE_TEMPERATURE.
        // (The update function defaults to accel, gyro, compass,
        //  so you don't have to specify these values.)
        imu.update(UPDATE_ACCEL  | UPDATE_COMPASS);
    
      }
      if( imu.time > 20000)
      {
        printIMUData();
      }
      else
      {
        prevTime=imu.time;
      }
    }
  } 
}

void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);
  currTime = imu.time;

  float gravityX = 2 * (q1*q3 - q0*q2);
  float gravityY = 2 * (q0*q1 + q2*q3);
  float gravityZ = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  if(abs(accelX-gravityX)<= 0.02)
          velocityX = 0.00;
  if(abs(accelY-gravityY)<= 0.02)
          velocityY = 0.00;

  velocityX += (accelX-gravityX)*(currTime-prevTime)/1000.00;
  prevAccelX = accelX-gravityX;
  
  velocityY += (accelY-gravityY)*(currTime-prevTime)/1000.00;
  prevAccelY=accelY-gravityY;

  positionX+=(velocityX)*(currTime-prevTime)/1000.00;
  prevVelocityX=velocityX;
  
  positionY+=(velocityY)*(currTime-prevTime)/1000.00;
  prevVelocityY=velocityY;
  
  SerialPort.println("Accel: " + String(accelX) + ", " +
              String(accelY) + ", " + String(accelZ) + " g");
  SerialPort.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  SerialPort.println("Mag: " + String(magX) + ", " +
              String(magY) + ", " + String(magZ) + " uT");
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
  SerialPort.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));
  SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
   SerialPort.println("velocity: " + String(velocityX) + ", " +
              String(velocityY) +" m/s");
   SerialPort.println("position: " + String(positionX) + ", " +
              String(positionY) + " m");
  SerialPort.println();
 
  prevTime=currTime;
}
