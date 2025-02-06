const int ledPin = 12;
char receivedData;

void setup(){
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);
  Serial.begin(9600);
}

void loop(){
  if(Serial.available()>0){
    char receivedData=Serial.read();
    Serial.print("Received:");
    Serial.println(receivedData);

    if(receivedData == '1'){
      digitalWrite(ledPin,HIGH);
    }
    else if(receivedData == '0'){
      digitalWrite(ledPin,LOW);
    }
  }
}
