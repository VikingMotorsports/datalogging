#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_AM2320.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <SPI.h>
#include <SD.h>

#define s0 8
#define s1 7
#define s2 6

Adafruit_AM2320 am2320 = Adafruit_AM2320();
Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DSOX sox;


float rawVolt, t, r, c, f = 0;
float rawVolt2, t2, r2, c2, f2 = 0;
float humidity, humidtemp = 0;
int rawair1 = 0, rawair2 = 0;
float airspeed1, airspeed2;
float magX, magY, magZ;
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;

/*
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
*/


File myFile;

// for string combination to print CSV

String stringTotal;

void setup() {
  //mux setup

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);


  lis3mdl.begin_I2C();
  lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, true, false, true); 

  
  // Starting serial monitor for testing
  Serial.begin(9600);
  while (!Serial); //waits for serial terminal to be open, necessary in newer arduino boards.

  // Start up the humidity / temp sensor sensor
  am2320.begin();

  // Attempt startup of the SD writer
  //if (!SD.begin(4)) {
  // Serial.println("initialization failed!");
  // while (1);
 // Serial.println("SD initialization success");
   // }


  if (!sox.begin_I2C()) {
    // if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }


  sox.begin_I2C();
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_52_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_52_HZ);


  
}

void loop() {
    
    myFile = SD.open("datalog.csv", FILE_WRITE);
    
    
    for (int i = 0; i <= 10; ++i) {


    
        // Humidity / Temp sensor
        humidity = (am2320.readHumidity());
        humidtemp = (am2320.readTemperature());
        Serial.print("Humid/temp: ");
        Serial.print(humidity,3);
        Serial.print(",");
        Serial.print(humidtemp,3);
        Serial.print("\n");





     
        // Airspeed 1 sensor
        rawair1 = analogRead(A1);
        Serial.print("Airspeed 1: ");
        airspeed1 = pow((( (float)rawair1 - 264.0) / 85.6814), 3.36814);
        Serial.print(airspeed1);
        Serial.print("\n");

         // Airspeed 2 sensor
        rawair2 = analogRead(A2);
        Serial.print("Airspeed 2: ");
        airspeed2 =  pow((((float)rawair2 - 264.0) / 85.6814), 3.36814);
        Serial.print(airspeed2);
        Serial.print("\n");





        // Temp probe 1 data
        digitalWrite(s0, LOW);
        digitalWrite(s1, LOW);
        digitalWrite(s2, LOW);
        // thermistor data
        rawVolt = (analogRead(A0)) * (5.0/1023.0);
        r = (rawVolt * 10) / (5 - rawVolt);
        t = 1 / ((1 / 298.15) + ((log (r / 10)) / 3380));
        c = t - 273.15;
        Serial.print("Temp probe 1: ");
        Serial.print(c);
        Serial.print("\n");   

        // Temp probe 2 data
        digitalWrite(s0, HIGH);
        digitalWrite(s1, LOW);
        digitalWrite(s2, LOW);
        // thermistor data
        rawVolt2 = (analogRead(A0)) * (5.0/1023.0);
        r2 = (rawVolt2 * 10) / (5 - rawVolt2);
        t2 = 1 / ((1 / 298.15) + ((log (r2 / 10)) / 3380));
        c2 = t2 - 273.15;
        Serial.print("Temp probe 2: ");
        Serial.print(c2);
        Serial.print("\n");   




        
        // Magnetometer data
        lis3mdl.read();
        sensors_event_t event; 
        lis3mdl.getEvent(&event);
        magX = event.magnetic.x;
        magY = event.magnetic.y;
        magZ = event.magnetic.z;
        Serial.print("Magnetics : ");     
        Serial.print(magX);
        Serial.print(",");
        Serial.print(magY);
        Serial.print(",");
        Serial.print(magZ);
        Serial.print("\n");


        // Acclerometer / Gyroscope
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        sox.getEvent(&accel, &gyro, &temp);
        
        accX = accel.acceleration.x;
        accY = accel.acceleration.y;
        accZ = accel.acceleration.z;

        gyroX = gyro.gyro.x;
        gyroY = gyro.gyro.y;
        gyroZ = gyro.gyro.z;

        Serial.print("Acceleration : ");   
        Serial.print(accX);
        Serial.print(",");
        Serial.print(accY);
        Serial.print(",");
        Serial.print(accZ);
        Serial.print("\n");

        Serial.print("Gyroscope : ");   
        Serial.print(gyroX);
        Serial.print(",");
        Serial.print(gyroY);
        Serial.print(",");
        Serial.print(gyroZ);
        Serial.print("\n");


        
        
        delay(20);
        


        //myFile.println(stringTotal);

  
    }
    myFile.close();
    // every 10 print lines, close and reopen the SD card
    // Saves the progress

  
}
