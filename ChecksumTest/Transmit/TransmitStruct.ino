// Omar Hossain
// Transmit.ino

#include <iostream>

const int divisor = 32;

byte checksum = 0;
int iterations = 0;
unsigned int sum = 0;
String a = "It's Working";

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

DataStructure testPack;

void setup() {
  // put your setup code here, to run once:

  testPack.GyroX = -12.345;
  testPack.GyroY = 67.891;
  testPack.GyroZ = -23.456;
  testPack.AccX = 78.912;
  testPack.AccY = -34.567;
  testPack.AccZ = 89.123;
  testPack.AngX = -45.678;
  testPack.AngY = 91.234;
  testPack.AngZ = -56.789;
  testPack.Temp = 12.345;
  testPack.Pres = -67.891;
  testPack.BMPAlt = 23.456;
  testPack.Lat = -78.912;
  testPack.Lng = 34.567;
  testPack.GPSAlt = -89.123;
  testPack.GPSSpeed = 45.678;
  testPack.Satellites = 4;
  
  Serial.begin(115200);
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);
}

void loop() {

  ++iterations;
  if ( iterations == 10)
  {
    
    // Running Sum of ASCII characters for one message
    for(int i = 0; i < a.length(); i++)
    { 
      sum += int(a[i]);
    }
    // Checksum can only be 1 (or a set number) byte 
    // More than that will cause issues
    // Modulus with some Divisor to give Checksum
    checksum = byte(sum%divisor);
    Serial.print("Checksum: ");
    Serial.println(checksum);
    Serial1.print(a);
    // Have an indicator for checksum after
    Serial1.print('\t');
    Serial1.write(checksum);
    sum = 0;
    iterations = 0;
  
  }
  
 }
