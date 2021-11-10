/* Useful Resources
 * MPU6050 Datasheet - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */

#include <Adafruit_Sensor.h>     // sensor abstraction library
#include <Adafruit_BMP3XX.h>     // barometric pressure sensor
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include <TinyMPU6050.h>         //used only to initalize

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

// Added to try to compile 
MPU6050 mpu(Wire);

// IMU STUFF
int basic = 0;

// MPU6050 I2C address
const int MPU = 0x68; //low accel

unsigned long currentTime, previousTime;
float elapsedTime;
int c = 0;

// MPU #1
float AccX, AccY, AccZ;
float yaw, pitch, roll;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

float y, p, r;
float AX, AY, AZ;
// IMU STUFF END

Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

struct DataStructure {
float GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngX, AngY, AngZ;
double Temp, Pres;
float BMPAlt, Lat, Lng;     
double GPSAlt, GPSSpeed;    
uint32_t Satellites;
};


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

  // --------------- INITIALIZE ACCELEROMETER #1 ---------------
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x18);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x18);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);

  //  --------------- --------------- --------------- ---------------
  
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error(200);
  delay(20);


  
  Serial.println("Setup complete.");
}

void loop() {

  // ---------------- MPU1 ---------------
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  AccX = (Wire.read() << 8 | Wire.read()) / 2048.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 2048.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 2048.0; // Z-axis value

  
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
  
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000.0; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 8.2;
  GyroY = (Wire.read() << 8 | Wire.read()) / 8.2;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 8.2;
  
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
  
  // raw values deg/s, multiply by s to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  
  // filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;



  // Time
  unsigned long timestamp = millis();
  Serial.println(timestamp);

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

  DataStructure DataPack;
  DataPack.GyroX = GyroX;
  DataPack.GyroY = GyroY;
  DataPack.GyroZ = GyroZ;
  DataPack.AccX = AccX;
  DataPack.AccY = AccY;
  DataPack.AccZ = AccZ;
  DataPack.AngX = roll;
  DataPack.AngY = yaw;
  DataPack.AngZ = pitch;
  DataPack.Temp = bmp.temperature;
  DataPack.Pres = (bmp.pressure/100);
  DataPack.BMPAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  DataPack.Lat = 111;//(tinyGPS.location.lat(), 6);
  DataPack.Lng = 222;//(tinyGPS.location.lng(), 6);
  DataPack.GPSAlt =  333;//(tinyGPS.altitude.feet());
  DataPack.GPSSpeed = 444;//tinyGPS.speed.mph();
  DataPack.Satellites = 555;//(tinyGPS.satellites.value());

  //Serial.print((byte*)&DataPack, sizeof(DataPack)); 

   Serial1.print ("FlaminHotMountainDew");
   Serial1.write((byte*)&DataPack, sizeof(DataPack));
   Serial1.print ("V");
 // Serial.print(DataPack.GyroX)
  
  String newTelem = "Hey";
  newTelem += ',';
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
  newTelem += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  newTelem += ',';
//  newTelem += String(tinyGPS.altitude.feet()) + ',';
//  newTelem += String(tinyGPS.speed.mph()) + ',';
//  newTelem += String(tinyGPS.location.lat(), 6) + ',';
//  newTelem += String(tinyGPS.location.lng(), 6) + ',';
//  newTelem += String(tinyGPS.satellites.value());
  newTelem += ',';
  newTelem += 'v';
  writeToFile(telemFileName, newTelem);

{
  while(Serial5.available()>0)
    tinyGPS.encode(Serial5.read());
  }

  // Send data over XBee once every 10 loops
  if (loopIndex >= 3) {
    Serial1.println(newTelem);
    //Serial1.flush();
    loopIndex = -1; // Will be updated below to 0
  }
  ++loopIndex;

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


void calculate_IMU_error(int numVal) {
  // Note accel should be flat for calibration (standstill) for calibration period
  // Read accelerometer values numVal times
  while (c < numVal) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 2048.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 2048.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 2048.0 ;
    
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  
  //Divide the sum by numVal to get the error value
  AccErrorX = AccErrorX / numVal;
  AccErrorY = AccErrorY / numVal;
  
  c = 0;
  
  // Read gyro values numVal times
  while (c < numVal) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 8.2);
    GyroErrorY = GyroErrorY + (GyroY / 8.2);
    GyroErrorZ = GyroErrorZ + (GyroZ / 8.2);
    c++;
  }
  
  //Divide the sum by numVal to get the error value
  GyroErrorX = GyroErrorX / numVal;
  GyroErrorY = GyroErrorY / numVal;
  GyroErrorZ = GyroErrorZ / numVal;
  
  // Print the error values on the Serial Monitor
  Serial.print(F("AccErrorX: "));
  Serial.println(AccErrorX, 5);
  Serial.print(F("AccErrorY: "));
  Serial.println(AccErrorY, 5);
  Serial.print(F("GyroErrorX: "));
  Serial.println(GyroErrorX, 5);
  Serial.print(F("GyroErrorY: "));
  Serial.println(GyroErrorY, 5);
  Serial.print(F("GyroErrorZ: "));
  Serial.println(GyroErrorZ, 5);
  Serial.print(F("AccErrorX2: "));

  delay(2000);
}
