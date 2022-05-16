// Omar Hossain
// Transmit.ino

int iterations = 0;
unsigned int sum = 0;

String a = "It's Working";
byte checksum = 0;
const int divisor = 32;

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
//  delay(1000);
  
//  delay(100);
//  Serial1.print('O');
//  delay(100);
  //  Serial1.print('H'); 

  ++iterations;
  if ( iterations == 10)
  {
    
    for(int i = 0; i < a.length(); i++)
    { 
      sum += int(a[i]);
    }
    Serial.print("Checksum: ");
    Serial.println(sum%32);
    Serial1.print(a);
    Serial1.print('\t');
    checksum = byte(sum%divisor);
    Serial1.write(byte(checksum));
    sum = 0;
    iterations = 0;
  
  }
  
  
    
  
 }
