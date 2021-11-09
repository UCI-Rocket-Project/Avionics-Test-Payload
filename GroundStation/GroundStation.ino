#define HEADER      "FlaminHotMountainDew"
#define TERMINATOR  "V"


struct DataStructure {
float GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngX, AngY, AngZ;
double Temp, Pres;
float BMPAlt, Lat, Lng;     
double GPSAlt, GPSSpeed;    
uint32_t Satellites;
} DataPack;


const unsigned char maxSentenceSize = 255;
const unsigned int maxStructSize=sizeof(DataPack);

// Packet buffer variables.
char sentence[maxSentenceSize+1];
byte byteArray[maxStructSize];

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
  DataStructure DataReading;
  int readCount=0;

  sentence[structSize] = '';
  pps = 0;
  packetsReceived = 0;
  startTime = millis();
}

void loop() {
  // Reset buffer.
  //memset(sentence, 0, maxSentenceSize);
  memset(byteArray, 0, maxStructSize);
  memset(DataReading, 0, maxStructSize);


  // Receive header.
  while (!Serial1.findUntil(HEADER, '\0')) {
  }

  // Receive body.
  //Serial1.readBytesUntil(TERMINATOR[1], sentence, maxSentenceSize);
  readCount=Serial1.readBytesUntil(TERMINATOR[0], byteArray, maxStructSize);

  if(readCount!=maxStructSize){
    Serial.println("readCount!=struct size: check if error| Still Currently printing incomplete packets"
  }

  ++packetsReceived;

  unsigned int currTime = millis();
  if ((currTime - startTime) >= 1000) {
    pps = packetsReceived;
    packetsReceived = 0;
    startTime = currTime;
  }

  // Output data and packet rate.
  //Serial.println(sentence+String(pps));
  Serial.println("Print Recieved Data")
  
}
