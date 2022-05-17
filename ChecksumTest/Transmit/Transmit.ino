// Omar Hossain
// Transmit.ino

const int divisor = 32;

byte checksum = 0;
int iterations = 0;
unsigned int sum = 0;
String a = "It's Working";

void setup() {
  // put your setup code here, to run once:
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
