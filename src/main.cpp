//////////////////////////////////////////////////////////////////////////////////////////////////////////
// BMP388_DEV - ESP8266, I2C Communications, Default Configuration, Normal Conversion, User-Defined Pins
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <string.h>
#include <U8g2lib.h>
#include <BMP388_DEV.h>                           // Include the BMP388_DEV.h library
#include <SimpleKalmanFilter.h>
#include <SPI.h>

#include "WavePWM.h"
WavePWM wavePWM;
const int DAC_PIN = 25;     //11 is a PWM pin on the Arduino Uno. Change to whichever pin you wish to use.
const unsigned long WAVE_LEN = 2000UL;  //in milliseconds. This wave will last 2 seconds.
unsigned long milli = 0UL;

float temperature, pressure, altitude;            // Create the temperature, pressure and altitude variables
//BMP388_DEV bmp388(23, 22);                          // Instantiate (create) a BMP388 object and set-up for I2C operation on pins (SDI)SDA: 6, (SCK)SCL: 7
SPIClass SPI3(VSPI);
BMP388_DEV bmp388(15, VSPI, SPI3); 

SimpleKalmanFilter pressureKalmanFilter(0.4, 0.4, 0.01);

U8G2_ST7565_JLX12864_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 14, /* data=*/ 27, /* cs=*/ 12, /* dc=*/ 26, /* reset=*/ 13);

void setup() 
{
  u8g2.begin();
  pinMode(2,OUTPUT);
  pinMode(33,OUTPUT);
  digitalWrite(2,1);
  digitalWrite(33,1);
  u8g2.setContrast(48);
  Serial.begin(115200);                           // Initialise the serial port
  bmp388.begin();                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp388.setTimeStandby(TIME_STANDBY_80MS);     // Set the standby time to 80 Mseconds
  bmp388.startNormalConversion();                 // Start BMP388 continuous conversion in NORMAL_MODE
  bmp388.setIIRFilter(IIR_FILTER_OFF);  // Options are IIR_FILTER_OFF, _2, _4, _8, _16, _32

  pinMode(DAC_PIN, OUTPUT);
  wavePWM.setup(WAVE_LEN);
}


void loop() 
{
  if (bmp388.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
  {
    float estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);
    Serial.print("data updated.");
    // for(int i=0; i<=10; i++){
    //   if(i==10){
        Serial.print(temperature);                    // Display the results    
        Serial.print(F("*C   "));
        Serial.print(pressure);    
        Serial.print(F("hPa   "));
        Serial.print(altitude);
        Serial.println(F("m"));  
        Serial.print(estimated_altitude,6);
        Serial.println();

        char height_str[20] = "Altitude: ";
        char height_num[20];
        char temp_str[20] = "Temperature: ";
        char temp_num[20];
        char pres_str[20] = "Pressure: ";
        char pres_num[20];
        char estimated_altitude_str[20] = "KF_Alti: ";
        char esti_alti_num[20];

        sprintf(height_num, "%.2f", altitude);
        sprintf(temp_num, "%.2f", temperature);
        sprintf(pres_num, "%.2f", pressure);
        sprintf(esti_alti_num, "%.2f", estimated_altitude);
        // 连接字符串
        strcat(height_str,height_num);
        strcat(temp_str,temp_num);
        strcat(pres_str,pres_num);
        strcat(estimated_altitude_str,esti_alti_num);
        // 送显示
        u8g2.clearBuffer();         // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
        u8g2.drawStr(0,10,"Kalman Filter Altitude");  // write something to the internal memory
        u8g2.drawStr(0,20,height_str);
        u8g2.drawStr(0,30,temp_str);
        u8g2.drawStr(0,40,pres_str);
        u8g2.drawStr(0,50,estimated_altitude_str);
        u8g2.sendBuffer();
    //   }
    // }
  }
    milli = millis();
    //write to LED
    analogWrite(DAC_PIN, wavePWM.getQuadraticValue(milli));
}