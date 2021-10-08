#define HEADER      "Hey,"
#define TERMINATOR  ",v"

const char START_BYTE = HEADER[0];
const unsigned int HEADER_LEN = strlen(HEADER);
const unsigned char maxSentenceSize = 255;

// Packet buffer info/diagnostics variables.
char sentence[maxSentenceSize+1];
unsigned int toRead = 0;
unsigned int alReady = 0;


void setup() {
  Serial.begin(115200);

  // Setup UART on Serial1 for reading from XBee
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);

  sentence[maxSentenceSize] = '\0';
}

void loop() {
  // Get time for tracking packet rate.
  
  
  // Reset buffer/tracking for this sentence.
  unsigned int alReady = 0;
  memset(sentence, 0, maxSentenceSize);
  unsigned int readThisIter;

  // Receive header.
  while (!Serial1.findUntil(HEADER, '\0')) {
  }

  // Receive body.
  Serial1.readBytesUntil(TERMINATOR[1], sentence, maxSentenceSize);
  Serial.println(sentence);
  /*
  while (true) {
    readThisIter = 0;
    toRead = Serial1.available();
    if (toRead > 0) {
      if ((toRead + alReady) > maxSentenceSize) {
        break;  
      }
      readThisIter = Serial1.readBytesUntil(TERMINATOR[1], sentence+alReady, toRead);
      alReady += readThisIter;
      
      if (readThisIter < toRead && Serial1.peek() == TERMINATOR[1]) {
        // Publish complete sentence/packet.
        Serial1.readBytes(sentence+alReady, 1);
        Serial.println(sentence);
        break;
      }
    }
    else {
      // Serial.println("To read: " + String(toRead));
      // delay(50);
    }
  }
  */
}
