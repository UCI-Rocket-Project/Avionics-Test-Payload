/* Useful Resources
 * MPU6050 Datasheet - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */

#include <Adafruit_Sensor.h>     // sensor abstraction library
#include <Adafruit_BMP3XX.h>     // barometric pressure sensor
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <TinyMPU6050.h>
#include <Wire.h>
#include <iostream>
#define SEALEVELPRESSURE_HPA (1013.2)

//Setup Adafruit Ultimate GPS 
//#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.
//#define ARDUINO_GPS_RX 7 // GPS TX, Arduino RX pin
//#define ARDUINO_GPS_TX 8 // GPS RX, Arduino TX pin
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object
//SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

//#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo
// #define SerialMonitor Serial3

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
float BMPAlt, Lat, Lng;     
double GPSAlt, GPSSpeed;    
uint32_t Satellites;
};
  #pragma pack(pop)


char* telemFileName = "telem.csv";
char* lordFileName = "telamen.csv";
char loopIndex = 0;

void setup() {
  Serial.begin(115200);
  //gpsPort.begin(GPS_BAUD);

  // Setup I2C bus (Used by the NXP 9-DOF & the BMP388 breakout boards)
  Wire.begin();
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

  // Setup Adafruit GPS
  Serial5.clear();
  Serial5.setRX(34);
  Serial5.setTX(33);
  Serial5.begin(9600);
 


  // Write to telemetry file, can use as a spacer to know what data is new.
  writeToFile(telemFileName, "Test");

  delay(500); // Delay for smoothing initial BMP
  Serial.println("Setup complete.");
}

void loop() {
  mpu.Execute();  // Update values (which mpu.Get____() will read from) with new data.

  // Time
  unsigned long timestamp = millis();
  Serial.println(timestamp);
{
  // Gyro x,y,z [degrees/second]
//  Serial.print("\t");
//  Serial.print(mpu.GetGyroX());
//  Serial.print(", ");
//  Serial.print(mpu.GetGyroY());
//  Serial.print(", ");
//  Serial.println(mpu.GetGyroZ());
//
//  // Acceleration x,y,z [m/s²]
//  Serial.print("\t");
//  Serial.print(mpu.GetAccX());
//  Serial.print(", ");
//  Serial.print(mpu.GetAccY());
//  Serial.print(", ");
//  Serial.println(mpu.GetAccZ());
//
//  // Attitude (yaw, pitch, roll in [degrees])
//  Serial.print("\t");
//  Serial.print(mpu.GetAngX());
//  Serial.print(", ");
//  Serial.print(mpu.GetAngY());
//  Serial.print(", ");
//  Serial.println(mpu.GetAngZ());
//
//  // Read bmp tempature [C], pressure [hPa], and altitude [m]
//  Serial.print("\t");
//  Serial.print(bmp.temperature);  
//  Serial.print(", ");
//  Serial.print(bmp.pressure / 100);
//  Serial.print(", ");
//  Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
//
//  // Print Latitude, Longitute, and number of satallites
//  Serial.print("\t");
//  Serial.println(tinyGPS.altitude.feet());
//  Serial.print("\t");
//  Serial.println(tinyGPS.speed.mph());
//  Serial.print("\t");
//  Serial.println(tinyGPS.location.lat(), 6);
//  Serial.print("\t");
//  Serial.println(tinyGPS.location.lng(), 6);
//  Serial.print("\t");
//  Serial.println(tinyGPS.satellites.value());
}
  DataStructure DataPack;
  {
    
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
  DataPack.Lat = 111;//(tinyGPS.location.lat(), 6);
  DataPack.Lng = 222;//(tinyGPS.location.lng(), 6);
  DataPack.GPSAlt = 333;//(tinyGPS.altitude.feet());
  DataPack.GPSSpeed = 444;//(tinyGPS.speed.mph();
  DataPack.Satellites = (uint32_t(timestamp));//(tinyGPS.satellites.value());
  
  }
  
  //std::cout << sizeof(DataPack) << std::endl;
  Serial.print(sizeof(DataPack));
      
  //Serial.print((byte*)&DataPack, sizeof(DataPack)); 
//  if(loopIndex >= 10){

  
     Serial1.print ("FlaminHotMountainDew");
     Serial1.write((byte*)&DataPack, sizeof(DataPack));
     Serial1.print ("V");
     Serial.print("Data sent");

//     Serial1.print ("FlaminHotMountainDew");
//     Serial1.write((byte*)&DataPack, sizeof(DataPack));
//     Serial1.print ("V");
//     loopIndex = -1;
//  }
//  ++loopIndex;
//  
  String newTelem = "Hey";
//  newTelem += ',';
//  newTelem += String(timestamp) + ',';
//  newTelem += String(mpu.GetGyroX()) + ',';
//  newTelem += String(mpu.GetGyroY()) + ',';
//  newTelem += String(mpu.GetGyroZ());
//  newTelem += ',';
//  newTelem += String(mpu.GetAccX()) + ',';
//  newTelem += String(mpu.GetAccY()) + ',';
//  newTelem += String(mpu.GetAccZ());
//  newTelem += ',';
//  newTelem += String(mpu.GetAngX()) + ',';
//  newTelem += String(mpu.GetAngY()) + ',';
//  newTelem += String(mpu.GetAngZ());
//  newTelem += ','; 
//  newTelem += String(bmp.temperature) + ',';
//  newTelem += String(bmp.pressure) + ',';
//  newTelem += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
//  newTelem += ',';
//  newTelem += String(tinyGPS.altitude.feet()) + ',';
//  newTelem += String(tinyGPS.speed.mph()) + ',';
//  newTelem += String(tinyGPS.location.lat(), 6) + ',';
//  newTelem += String(tinyGPS.location.lng(), 6) + ',';
//  newTelem += String(tinyGPS.satellites.value());
//  newTelem += ',';
//  newTelem += 'v';
//  writeToFile(telemFileName, newTelem);

{
  while(Serial5.available()>0)
    tinyGPS.encode(Serial5.read());
  }

  delay(100);

  // Send data over XBee once every 10 loops
//  if (loopIndex >= 3) {
//    Serial1.println(newTelem);
//    //Serial1.flush();
//    loopIndex = -1; // Will be updated below to 0
//  }
//  ++loopIndex;

  // unsigned short lordAvailable = Serial2.available();
  // char lordTelem[lordAvailable];
  // Serial2.readBytes(lordTelem, lordAvailable);
  // writeToFile(lordFileName, (String)lordTelem + '\n');
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
