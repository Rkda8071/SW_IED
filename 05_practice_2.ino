#define PIN7 7

void setup(){
  pinMode(PIN7, OUTPUT);
  digitalWrite(PIN7, LOW);
  delay(1000);
  digitalWrite(PIN7, HIGH);
  int i = 5;
  while(i-- > 0){
    delay(100);
    digitalWrite(PIN7, LOW);
    delay(100);
    digitalWrite(PIN7, HIGH);
  }
  
}

void loop(){
  
}
