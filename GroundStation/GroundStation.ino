#define HEADER      "FlaminHotMountainDew"
#define TERMINATOR  "v"


struct DataStructure {
  float GyroX, GyroY, GyroZ;
  float AccX, AccY, AccZ;
  float AngX, AngY, AngZ;
  double Temp, Pres;
  float BMPAlt, Lat, Lng;     
  double GPSAlt, GPSSpeed;    
  uint32_t Satellites;
} data;
char dataBytes[84];

// Packet diagnostic variables.
unsigned int pps;
unsigned int packetsReceived;
unsigned int startTime;

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
  Serial.print(sizeof(data));
}

void loop() {
  // Reset buffer.
  //memset(sentence, 0, maxSentenceSize);
  memset(&data, 0, sizeof(data));

  // Receive header.
  while (!Serial1.findUntil(HEADER, '\0')) {
//    Serial.println("Waiting for start...");
//    delay(10);
  }

  // Receive body.
  int readCount = Serial1.readBytesUntil(TERMINATOR[0], (byte*) &data, sizeof(data));

  if(readCount != 84){
//    Serial.println("readCount is " + (String)readCount);
//    Serial.println("readCount!=struct size: check if error| Still Currently printing incomplete packets");
    return;
  }

  // Packet received.
  ++packetsReceived;

  unsigned int currTime = millis();
  if ((currTime - startTime) >= 1000) {
    pps = packetsReceived;
    packetsReceived = 0;
    startTime = currTime;
  }

  // Output data and packet rate.
  Serial.println((String)data.GyroX + ',' + (String)data.GyroY + ',' + (String) data.GyroZ + ',' + 
    (String)data.AccX + ',' + (String)data.AcbcY + ',' + (String)data.AccZ + ',' + 
    (String)data.AngX + ',' + (String)data.AngY + ',' + (String)data.AngZ + ',' + 
    (String)data.Temp + ',' + (String)data.Pres + ',' + 
    (String)data.BMPAlt + ',' + (String)data.Lat + ',' + (String)data.Lng + ',' + 
    (String)data.GPSAlt + ',' + (String)data.GPSSpeed + ',' + 
    (String)data.Satellites + ',' + (String)pps);
}
