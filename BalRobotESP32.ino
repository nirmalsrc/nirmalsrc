//ESP-WROOM-32 KIT 

// Default SPI is VSPI 
// 23 MOSI
// 19 MISO
// 18 CLK
// 0 CS

// MPU9250
// SCL CLK (18)
// SDA MOSI (23)
// ADO MISO (19)
// CS NCS (22)

#include <Arduino.h>
#include <U8x8lib.h>
#include <SPI.h>
#include "MPU9250.h"
#include <Kalman.h>
#include <ESP32Encoder.h>

const float _r2d = 180.0f / 3.14159265359f;

//enum puType ESP32Encoder::useInternalWeakPullResistors=DOWN;


ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

U8X8_SH1106_128X64_NONAME_4W_HW_SPI myDisp(4,16,17);  //CS, DC, RES
MPU9250 IMU(SPI,0);
int IMUstatus;
float gx, ay, az;
// char msgBuffer [17];
Kalman pitchKalman;
double pitch = 0.0;
int64_t lastUpdate;

void setup(void)
{
  myDisp.begin();
  
  ESP32Encoder::useInternalWeakPullResistors=NONE;
  // Attache pins for use as encoder pins
	encoderLeft.attachFullQuad(26, 27);
	encoderRight.attachFullQuad(14, 12);
  
  myDisp.setFont(u8x8_font_chroma48medium8_r);
  myDisp.setCursor(0, 0);
  myDisp.println("Initialising IMU");

  IMUstatus = IMU.begin();
  if (IMUstatus < 0) {
    myDisp.clearLine(0);
    myDisp.setCursor(0, 0);
    myDisp.println("IMU STARTUP FAIL!");
    myDisp.setCursor(0, 1);
    myDisp.print("Status: ");
    myDisp.println(IMUstatus);
  }
  else {
    myDisp.clearLine(0);
    myDisp.setCursor(0, 0);
    myDisp.println("IMU OK!");     
  }

  while( IMU.readSensor() != 1);

  //lastIMUupdate = micros();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();

  pitch = atan2(ay, -az) * _r2d;
  pitchKalman.setAngle(pitch);
  myDisp.setCursor(0, 1);
  myDisp.print("Pitch0:");
  myDisp.println(pitch); 
  lastUpdate = esp_timer_get_time();

}

void loop(void)
{
  int64_t now;

  float pitchAcc;
  if (IMU.readSensor()) {  //Check if data is ready
    gx = IMU.getGyroX_rads();// Read the x/y/z adc values
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();
    pitchAcc = atan2(ay, -az) * _r2d;
    //pitch = pitchKalman.getAngle(pitchAcc, -gx*_r2d, deltat);  
    now = esp_timer_get_time();
    myDisp.clearLine(2);
    myDisp.setCursor(0, 2);
    myDisp.println(pitchAcc);  
    myDisp.clearLine(3);
    myDisp.setCursor(0, 3);
    myDisp.println( (long int) (now - lastUpdate), 10 );
    lastUpdate = esp_timer_get_time(); 

    myDisp.clearLine(4);
    myDisp.setCursor(0, 4);
    myDisp.println( (long int)  encoderLeft.getCount() );
    
    myDisp.clearLine(5);
    myDisp.setCursor(0, 5);
    myDisp.println( (long int) encoderRight.getCount() );
  }
}