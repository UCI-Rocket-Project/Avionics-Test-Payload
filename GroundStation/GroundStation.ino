#define HEADER      "Hey,"
#define TERMINATOR  ",v"

const unsigned char maxSentenceSize = 255;

// Packet buffer variables.
char sentence[maxSentenceSize+1];

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

  sentence[maxSentenceSize] = '\0';
  pps = 0;
  packetsReceived = 0;
  startTime = millis();
}

void loop() {
  // Reset buffer.
  memset(sentence, 0, maxSentenceSize);

  // Receive header.
  while (!Serial1.findUntil(HEADER, '\0')) {
  }

  // Receive body.
  Serial1.readBytesUntil(TERMINATOR[1], sentence, maxSentenceSize);
  ++packetsReceived;

  unsigned int currTime = millis();
  if ((currTime - startTime) >= 1000) {
    pps = packetsReceived;
    packetsReceived = 0;
    startTime = currTime;
  }

  // Output data and packet rate.
  Serial.println(sentence+String(pps));
}
