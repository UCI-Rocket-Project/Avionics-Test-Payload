// FOR ATP

/* Useful Resources
 * MPU6050 Datasheet - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */



#include <Arduino.h>
#include <Wire.h>
#include "C:\Users\nchen\Documents\PlatformIO\Projects\TeensyGPSTest\.pio\libdeps\teensy36\Adafruit BusIO\Adafruit_I2CDevice.h"
#include "C:\Users\nchen\Documents\PlatformIO\Projects\TeensyGPSTest\.pio\libdeps\teensy35\Adafruit_Sensor-master\Adafruit_Sensor.h"     // sensor abstraction library
#include "C:\Users\nchen\Documents\PlatformIO\Projects\TeensyGPSTest\.pio\libdeps\teensy36\Adafruit BMP3XX Library\Adafruit_BMP3XX.h"     // barometric pressure sensor
#include <SD.h>
#include <SoftwareSerial.h>
#include "C:\Users\nchen\Documents\PlatformIO\Projects\TeensyGPSTest\.pio\libdeps\teensy35\TinyMPU6050\src\TinyMPU6050.h"
#include "C:\Users\nchen\Documents\PlatformIO\Projects\TeensyGPSTest\.pio\libdeps\teensy35\TinyGPSPlus\src\TinyGPS++.h"
#include <iostream>





void writeToFile(char* fileName, String telem);

#define SEALEVELPRESSURE_HPA (1013.2)

//Setup Adafruit Ultimate GPS 
#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.
//#define ARDUINO_GPS_RX 7 // GPS TX, Arduino RX pin
//#define ARDUINO_GPS_TX 8 // GPS RX, Arduino TX pin
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object
//SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

//#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo
//#define SerialMonitor Serial3

/* TODOs:
 *  Create version without Serial (keep Serial1 & Serial2) prints before flight?
 */

MPU6050 mpu(Wire);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

  #pragma pack(push, 1)
struct DataStructure {
float GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngX, AngY, AngZ;
double Temp, Pres;
float BMPAlt, Lat, Lng, altR;     
double GPSAlt, GPSSpeed;    
uint32_t Satellites;
};
  #pragma pack(pop)


char* telemFileName = "telem.csv";
char* lordFileName = "telamen.csv";
char loopIndex = 0;


DataStructure DataPack;

// for ascent/descent rate
float altRate = 0;
float prvAlt;
unsigned long prvTime = 0;
float altRateArr[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int counter = 0;


unsigned int XBEE_INTERVAL = 100; // every __ ms
unsigned long intervalTime = 0;



void setup() {
  Serial.begin(115200);
  //gpsPort.begin(GPS_BAUD);

  // Setup I2C bus (Used by the NXP 9-DOF & the BMP388 breakout boards)
  Wire.begin();
  Wire.setSDA(18);
  Wire.setSCL(19);

  // Setup IMU (MPU6050) using TinyMPU6050 library.
  mpu.Initialize(); // Must do this after setting up Wire, since it internally calls 
  mpu.RegisterWrite(MPU6050_ACCEL_CONFIG, 0x18);  // By default, TinyMPU6050 uses +-2g, we want +-16g
  Serial.println("Starting MPU6050 Calibration...");
  mpu.Calibrate();
  Serial.println("MPU6050 Calibration complete.");

  // Wait for the hardware to initialize
  bool initialized = false;
  while (!initialized) {
    delay(1000);
    initialized = true;
    if (!bmp.begin_I2C(0x77)) {
      Serial.println("Error detecting BMP388");
      initialized = false;
    }
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("Error detecting SD card");
      initialized = false;
    }
  }

  // Setup Barometer (BMP388)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X); // needed for the bmp388
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Setup XBee UART
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);

  // Setup Adafruit GPS
  Serial5.clear();
  Serial5.setRX(34);
  Serial5.setTX(33);
  Serial5.begin(9600);
 


  // Write to telemetry file, can use as a spacer to know what data is new.
  writeToFile(telemFileName, "Test");

  delay(500); // Delay for smoothing initial BMP
  Serial.println("Setup complete.");
  Serial.println("INITIALIZAING STRUCT");
  DataPack.GyroX = 0;
  DataPack.GyroY = 0;
  DataPack.GyroZ = 0;
  DataPack.AccX = 0;
  DataPack.AccY = 0;
  DataPack.AccZ = 0;
  DataPack.AngX = 0;
  DataPack.AngY = 0;
  DataPack.AngZ = 0;
  DataPack.Temp = 0;
  DataPack.Pres = 0;
  DataPack.BMPAlt = 0;
  DataPack.Lat = 0;
  DataPack.Lng = 0;
  DataPack.GPSAlt = 0;
  DataPack.GPSSpeed = 0;
  DataPack.Satellites = 0;
  DataPack.altR = 0;
  Serial.println("INITIALIZATION AND SETUP COMPLETE");
}

void loop() {
  mpu.Execute();  // Update values (which mpu.Get____() will read from) with new data.

  // Time
  unsigned long timestamp = millis();


  prvAlt = DataPack.BMPAlt;
  
  DataPack.GyroX = mpu.GetGyroX();
  DataPack.GyroY = mpu.GetGyroY();
  DataPack.GyroZ = mpu.GetGyroZ();
  DataPack.AccX = mpu.GetAccX();
  DataPack.AccY = mpu.GetAccY();
  DataPack.AccZ = mpu.GetAccZ();
  DataPack.AngX = mpu.GetAngX();
  DataPack.AngY = mpu.GetAngY();
  DataPack.AngZ = mpu.GetAngZ();
  DataPack.Temp = bmp.temperature;
  DataPack.Pres = (bmp.pressure/100);
  DataPack.BMPAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  DataPack.Lat = (tinyGPS.location.lat(), 6);
  DataPack.Lng = (tinyGPS.location.lng(), 6);
  DataPack.GPSAlt = (tinyGPS.altitude.feet());
  DataPack.GPSSpeed = (tinyGPS.speed.mph());
  DataPack.Satellites = (tinyGPS.satellites.value());

  if(counter>=20) {
    counter = 0;
  }
  altRateArr[counter] = 1000.0 * (DataPack.BMPAlt - prvAlt) / (millis() - prvTime);
  counter++;

  altRate = 0;
  for(int i = 0; i < 20; i++) {
    altRate += altRateArr[i];
  }
  altRate /= 20;
  DataPack.altR = altRate;

  prvTime = millis();
  
  


  
  if(millis() - intervalTime > XBEE_INTERVAL){
     Serial1.print ("FlaminHotMountainDew");
     Serial1.write((byte*)&DataPack, sizeof(DataPack));
     Serial1.print ("V");
     intervalTime = millis();
     //Serial.print("Data sent");
  }


  
  String newTelem = "";
  newTelem += String(timestamp) + ',';
  newTelem += String(mpu.GetGyroX()) + ',';
  newTelem += String(mpu.GetGyroY()) + ',';
  newTelem += String(mpu.GetGyroZ());
  newTelem += ',';
  newTelem += String(mpu.GetAccX()) + ',';
  newTelem += String(mpu.GetAccY()) + ',';
  newTelem += String(mpu.GetAccZ());
  newTelem += ',';
  newTelem += String(mpu.GetAngX()) + ',';
  newTelem += String(mpu.GetAngY()) + ',';
  newTelem += String(mpu.GetAngZ());
  newTelem += ','; 
  newTelem += String(bmp.temperature) + ',';
  newTelem += String(bmp.pressure) + ',';
  newTelem += String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + ',';
  newTelem += String(DataPack.altR);  // in m/s
  newTelem += ',';
  newTelem += String(tinyGPS.altitude.feet()) + ',';
  newTelem += String(tinyGPS.speed.mph()) + ',';
  newTelem += String(tinyGPS.location.lat(), 6) + ',';
  newTelem += String(tinyGPS.location.lng(), 6) + ',';
  newTelem += String(tinyGPS.satellites.value()) + ',';
  Serial.print(newTelem);
  writeToFile(telemFileName, newTelem);

  while(Serial5.available()>0){
    tinyGPS.encode(Serial5.read());
  }

}

void writeToFile(char* fileName, String telem) {
  File telemFile = SD.open(fileName, FILE_WRITE);
  if (telemFile) {
    telemFile.println(telem);
    telemFile.close();
    Serial.println("Wrote to " + (String)fileName + " on SD");
  }
  else {
    Serial.println("Error writing to " + (String)fileName + " on SD");
  }
}


































/*void writeToFile(char* fileName, String telem);
static void smartDelay();
void printDate();
void printTime();



union float_byte {
  float value;
  byte b[4];
};
union short_byte {
  short value;
  byte b[2];
};
union int_byte {
  int value;
  byte b[4];
};
union double_byte {
  double value;
  byte b[8];
};

union short_byte y, p, r;
union float_byte ax, ay, az;

union float_byte temperature;
union float_byte pressure;
union float_byte altitude;


#define SEALEVELPRESSURE_HPA (1013.2)



MPU6050 mpu(Wire);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

SoftwareSerial XBee(2, 3); // RX, TX

char* telemFileName = "telem.csv";
char* lordFileName = "telamen.csv";
char loopIndex = 0;




#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.
#define ARDUINO_GPS_RX 34 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 33 // GPS RX, Arduino TX pin
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

#define SerialMonitor Serial




void setup() {
  Serial.begin(115200);
  gpsPort.begin(GPS_BAUD);
  XBee.begin(9600);

  // Setup I2C bus (Used by the NXP 9-DOF & the BMP388 breakout boards)
  Wire.setSDA(18);
  Wire.setSCL(19);

  // Setup IMU (MPU6050) using TinyMPU6050 library.
  mpu.Initialize(); // Must do this after setting up Wire, since it internally calls 
  mpu.RegisterWrite(MPU6050_ACCEL_CONFIG, 0x08);  // By default, TinyMPU6050 uses +-2g, we want +-16g
  Serial.println("Starting MPU6050 Calibration...");
  mpu.Calibrate();
  Serial.println("MPU6050 Calibration complete.");

  // Wait for the hardware to initialize
  bool initialized = false;
  while (!initialized) {
    delay(1000);
    initialized = true;
    if (!bmp.begin_I2C(0x77)) {
      Serial.println("Error detecting BMP388");
      initialized = false;
    }
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("Error detecting SD card");
      initialized = false;
    }
  }

  // Setup Barometer (BMP388)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X); // needed for the bmp388
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Setup XBee UART
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);

  // Setup Lord IMU UART
  Serial2.clear();
  Serial2.setRX(9);
  Serial2.setTX(10);
  Serial2.begin(115200);

  // Write to telemetry file, can use as a spacer to know what data is new.
  writeToFile(telemFileName, "Test");

  delay(500); // Delay for smoothing initial BMP
  Serial.println("Setup complete.");
}

void loop() {
  mpu.Execute();  // Update values (which mpu.Get____() will read from) with new data.

  String tmp = "";
  char tmpC[100];
  byte tmpB[36];

  // Time
  unsigned long timestamp = millis();
  tmp += String(millis()) + ',';

  // Gyro x,y,z [degrees/second]
  tmp += String(mpu.GetGyroX()) + ',';
  tmp += String(mpu.GetGyroY()) + ',';
  tmp += String(mpu.GetGyroZ()) + ',';

  // Acceleration x,y,z [m/s²]
  tmp += String(mpu.GetAccX()) + ',';
  tmp += String(mpu.GetAccY()) + ',';
  tmp += String(mpu.GetAccZ()) + ',';
  
  // Attitude (yaw, pitch, roll in [degrees])
  tmp += String(mpu.GetAngX()) + ',';
  tmp += String(mpu.GetAngY()) + ',';
  tmp += String(mpu.GetAngZ()) + ',';

  // Read bmp tempature [C], pressure [hPa], and altitude [m]
  tmp += String(bmp.temperature) + ',';
  tmp += String(bmp.pressure / 100) + ',';
  tmp += String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + ',';

  // GPS Data
  tmp += String(tinyGPS.location.lat(), 7) + ",";
  tmp += String(tinyGPS.location.lng(), 7) + ",";
  tmp += String(tinyGPS.altitude.feet()) + ",";
  tmp += String(tinyGPS.satellites.value()) + ",";
  tmp += String(tinyGPS.speed.mph());
  
  writeToFile(telemFileName, tmp);

  // Send data over XBee once every 10 loops
  if (loopIndex >= 10) {
    Serial1.println(tmp);
    loopIndex = -1; // Will be updated below to 0
    Serial.print(tmp + "\tXBEE");
  }else{
    Serial.print(tmp);
  }
  ++loopIndex;

  unsigned short lordAvailable = Serial2.available();
  char lordTelem[lordAvailable];
  Serial2.readBytes(lordTelem, lordAvailable);
  writeToFile(lordFileName, (String)lordTelem);

  //Serial.print(tmp);
  //(tmp + "\n").toCharArray(tmpC, tmp.length());
  //XBee.write(tmpC);

  smartDelay();
}

void writeToFile(char* fileName, String telem) {
  File telemFile = SD.open(fileName, FILE_WRITE);
  if (telemFile) {
    telemFile.println(telem);
    telemFile.close();
    Serial.println();
  }
  else {
    Serial.println("\tError writing to " + (String)fileName + " on SD");
  }
}


// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay()
{
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
}

/*
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "C:\Users\nchen\Documents\PlatformIO\Projects\TeensyGPSTest\.pio\libdeps\teensy35\TinyGPSPlus\src\TinyGPS++.h"


static void smartDelay(unsigned long ms);
void printDate();
void printTime();

#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.
#define ARDUINO_GPS_RX 33 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 34 // GPS RX, Arduino TX pin
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

#define SerialMonitor Serial

void setup()
{
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);
}

void loop()
{
  // print position, altitude, speed, time/date, and satellites:
  SerialMonitor.print("Lat: "); SerialMonitor.println(tinyGPS.location.lat(), 6);
  SerialMonitor.print("Long: "); SerialMonitor.println(tinyGPS.location.lng(), 6);
  SerialMonitor.print("Alt: "); SerialMonitor.println(tinyGPS.altitude.feet());
  SerialMonitor.print("Course: "); SerialMonitor.println(tinyGPS.course.deg());
  SerialMonitor.print("Speed: "); SerialMonitor.println(tinyGPS.speed.mph());
  SerialMonitor.print("Sats: "); SerialMonitor.println(tinyGPS.satellites.value());
  SerialMonitor.println();

  // "Smart delay" looks for GPS data while the Arduino's not doing anything else
  smartDelay(1000); 
}


// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}

// printDate() formats the date into dd/mm/yy.
void printDate()
{
  SerialMonitor.print(tinyGPS.date.day());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.month());
  SerialMonitor.print("/");
  SerialMonitor.println(tinyGPS.date.year());
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
void printTime()
{
  SerialMonitor.print(tinyGPS.time.hour());
  SerialMonitor.print(":");
  if (tinyGPS.time.minute() < 10) SerialMonitor.print('0');
  SerialMonitor.print(tinyGPS.time.minute());
  SerialMonitor.print(":");
  if (tinyGPS.time.second() < 10) SerialMonitor.print('0');
  SerialMonitor.println(tinyGPS.time.second());
}*/