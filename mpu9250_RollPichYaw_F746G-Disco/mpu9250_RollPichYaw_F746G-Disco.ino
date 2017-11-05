
#include "Wire.h"


#include "LTDC_F746_Discovery.h"
#include <Wire.h>
///#include "stm32_ub_touch_480x272.h"

LTDC_F746_Discovery tft;


// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include "MadgwickAHRS.h"

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev    I2C_M;

uint8_t   buffer_m[6];

int16_t   ax, ay, az;
int16_t   gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

/// #define sample_num_mdate  5000


void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);


    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    filter.begin(25);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
    Serial.println(accelgyro.testConnection());
    delay(1000);
    Serial.println("     ");
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();
  
    //  Mxyz_init_calibrated ();
}


void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
//  float ax, ay, az;
//  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
///    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
///    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
/*
    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);
*/

//    Axyz[0] = (double) ax / 16384;
//    Axyz[1] = (double) ay / 16384;
//    Axyz[2] = (double) az / 16384;
//    Gxyz[0] = (double) gx * 250 / 32768;
//    Gxyz[1] = (double) gy * 250 / 32768;
//    Gxyz[2] = (double) gz * 250 / 32768;
    
    // update the filter, which computes orientation
//    filter.updateIMU(gx, gy, gz, ax, ay, az);

    filter.updateIMU((double)gx * 250 / 32768, (double) gy * 250 / 32768, (double) gz * 250 / 32768, (double) ax / 16384, (double) ay / 16384, (double) az / 16384);
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    Serial.print(" - ");
    Serial.println(microsPrevious);
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

