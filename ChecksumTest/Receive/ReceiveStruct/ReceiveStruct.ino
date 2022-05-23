// Omar Hossain
// Receive.ino

#include <iostream>

const unsigned char MAXSENTENCESIZE = 160;
const int DIVISOR = 32;

bool checkReady = false;
byte checksum = 0;
byte checksum_cur = 0;
unsigned int toRead = 0;
unsigned int count = 0;
unsigned int success = 0;
char incomingByte = '0';
char buffer[1];
char sentence[MAXSENTENCESIZE];
String body;

#pragma pack(push,1)
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

DataStructure bodyPacket;

void setup() {
  // Serial is Console/System
  // Serial1 is Receving Input
  // We have a baud rate of 115200
  Serial.begin(115200);
  Serial1.clear();
  // Set the RX and TX pins
  Serial1.setRX(0);
  Serial1.setTX(1);
  // Serial and Serial1 must have a consistent baud rate
  Serial1.begin(115200);

}

void loop() {
  toRead = Serial1.available();
  
  if (toRead > 0) {
    Serial1.readBytes(buffer, 1);
    if(checkReady){
      count++;
      checkReady = false;
      checksum = byte(buffer[0]);
      checksum_cur = checksumCalc(body, DIVISOR);
      
      if(checksum == checksum_cur) {
        success++;
        Serial.print("Packet Retreival Success: ");
        Serial.println(checksum);
        Serial.println(body);
      }
      else {
        Serial.print("Pakcet Retreival Failure: ");
        Serial.println(checksum_cur);
      }
      Serial.print("Accuracy: ");
      Serial.print((float)( ((float)success)/((float)count)) * 100);
      Serial.println("%");
      body = "";
    }
    else if(buffer[0] == '\t') {
      Serial.print("Message End Received, Body: ");
      Serial.println(body);
      // Copy the contents of body (a string of characters) to 
      // bodyPacket, which is DataStructure
      memcpy(&bodyPacket, &body, sizeof(body));
      Serial.print("GyroX: ");
      Serial.println(bodyPacket.GyroX);
      Serial.print("GyroY: ");
      Serial.println(bodyPacket.GyroY);
      Serial.print("GyroZ: ");
      Serial.println(bodyPacket.GyroZ);
      Serial.print("AccX: ");
      Serial.println(bodyPacket.AccX);
      Serial.print("AccY: ");
      Serial.println(bodyPacket.AccY);
      Serial.print("AccZ: ");
      Serial.println(bodyPacket.AccZ);
      Serial.print("AngX: ");
      Serial.println(bodyPacket.AngX);
      Serial.print("AngY: ");
      Serial.println(bodyPacket.AngY);
      Serial.print("AngZ: ");
      Serial.println(bodyPacket.AngZ);
      Serial.print("Temp: ");
      Serial.println(bodyPacket.Temp);
      Serial.print("Pres: ");
      Serial.println(bodyPacket.Pres);
      Serial.print("BMPAlt: ");
      Serial.println(bodyPacket.BMPAlt);
      Serial.print("Lat: ");
      Serial.println(bodyPacket.Lat);
      Serial.print("Lng: ");
      Serial.println(bodyPacket.Lng);
      Serial.print("GPSAlt: ");
      Serial.println(bodyPacket.GPSAlt);
      Serial.print("GPSSpeed: ");
      Serial.println(bodyPacket.GPSSpeed);
      Serial.print("Satellites: ");
      Serial.println(bodyPacket.Satellites);
      checkReady = true;
    }
    else {
      body.concat(buffer[0]);
    }
  }
  else {
    // Nothing Received
  }

}

// Checksum Calculation for a string message and int divisor
// Returns Checksum
int checksumCalc(String message, int divisor) {
  int sum = 0;
  for(int i = 0; i < message.length(); i++){ sum += int(message[i]); }
  return sum%divisor;
}
