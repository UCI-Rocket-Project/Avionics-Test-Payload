// Omar Hossain
// Receive.ino

const unsigned char MAXSENTENCESIZE = 160;
const int DIVISOR = 32;

bool checkReady = false;
byte checksum = 0;
unsigned int toRead = 0;
unsigned int count = 0;
unsigned int success = 0;
char incomingByte = '0';
char buffer[1];
char sentence[MAXSENTENCESIZE];
String body;

// SoftwareSerial Xbee(0, 1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  toRead = Serial1.available();
  
  if (toRead > 0) {
    Serial1.readBytes(buffer, 1);
    if(checkReady){
      count++;
      checkReady = false;
      checksum = byte(buffer[0]);
      
      if(checksum == checksumCalc(body, DIVISOR)) {
        success++;
        Serial.print("Packet Retreival Success: ");
        Serial.println(checksum);
        Serial.println(body);
      }
      else {
        Serial.print("Pakcet Retreival Failure: ");
        Serial.println(checksum);
      }
      body = "";
    }
    else if(buffer[0] == '\t') {
      Serial.print("Message End Received, Body: ");
      Serial.println(body);
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

int checksumCalc(String message, int divisor) {
  int sum = 0;
  for(int i = 0; i < message.length(); i++){ sum += int(message[i]); }
  return sum%divisor;
}
