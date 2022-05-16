// Omar Hossain
// Transmit.ino

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
  Serial1.println("It's Working!");
  delay(100);
  Serial1.print('O');
  delay(100);
  Serial1.print('H');

}
