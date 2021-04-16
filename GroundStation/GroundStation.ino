unsigned int toRead = 0;
const unsigned char maxSentenceSize = 160;
char sentence[maxSentenceSize];

void setup() {
  Serial.begin(115200);

  // Setup UART on Serial1 for reading from XBee
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);
}

void loop() {
  toRead = Serial1.available();

  if (toRead > 0) {
    // XBee has received data
    Serial1.readBytes(sentence, toRead);
    Serial.println(sentence);
  }
  else {
    // Nothing received.
    
  }
}
