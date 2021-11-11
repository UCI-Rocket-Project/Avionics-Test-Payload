#define HEADER      "FlaminHotMountainDew"
#define TERMINATOR  "V"

const unsigned int packetSize=84; //assumes no packing

#pragma pack(push,1)
struct DataStructure {
  float GyroX, GyroY, GyroZ;
  float AccX, AccY, AccZ;
  float AngX, AngY, AngZ;
  double Temp, Pres;
  float BMPAlt, Lat, Lng;     
  double GPSAlt, GPSSpeed;    
  uint32_t Satellites;
} data;
#pragma pack(pop)
char dataBytes[84];

// Packet diagnostic variables.
unsigned int pps;
unsigned int packetsReceived;
unsigned int startTime;

//moved variables outside of loop 
unsigned int currTime=0;
int readCount=0;
void setup() {
  Serial.begin(115200);

  // Setup UART on Serial1 for reading from XBee
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);
  
  pps = 0;
  packetsReceived = 0;
  startTime = millis();
}

void loop() {
  // Reset buffer.
  //memset(sentence, 0, maxSentenceSize);
  memset(&data, 0, packetSize);
  

  // Receive header.
  while (!Serial1.findUntil(HEADER, '\0')) {
   // Serial.println("Waiting for start...");
//    delay(10);
  }

  // Receive body.
  readCount = Serial1.readBytesUntil(TERMINATOR[0], (byte*) &data, packetSize);
  Serial1.flush();
  if(readCount != packetSize-1){
    Serial.println("readCount is " + (String)readCount);
//    Serial.println("readCount!=struct size: check if error| Still Currently printing incomplete packets");
    return;
  }

  // Packet received.
  ++packetsReceived;

  currTime = millis();
  if ((currTime - startTime) >= 1000) {
    pps = packetsReceived;
    packetsReceived = 0;
    startTime = currTime;
  }
  Serial.println("Printing Data Pack: Size= "+(String)readCount);
  // Output data and packet rate.
   
  Serial.print((String)data.GyroX + ',' + (String)data.GyroY + ',' + (String) data.GyroZ + ',') ;
  Serial.print((String)data.AccX + ',' + (String)data.AccY + ',' + (String)data.AccZ + ',' );
  Serial.print((String)data.AngX + ',' + (String)data.AngY + ',' + (String)data.AngZ + ',' );
  Serial.print((String)data.Temp + ',' + (String)data.Pres + ',');
  Serial.print((String)data.BMPAlt + ',' + (String)data.Lat + ',' + (String)data.Lng + ',' );
  Serial.print( (String)data.GPSAlt + ',' + (String)data.GPSSpeed + ',' );
  Serial.print((String)data.Satellites + ',' + (String)pps );
Serial.println();

}
