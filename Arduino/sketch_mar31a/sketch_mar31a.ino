int ledPin = 13;
int val;

void setup() {
pinMode(ledPin, OUTPUT);
Serial1.begin(19200);
}

void loop() {
// send data to another XBee module
Serial1.write('A');
delay(1000);

// receive data from another XBee module
val = Serial1.read();
Serial.print(val);
if (-1 != val) {
if ('A' == val) {
digitalWrite(ledPin, HIGH);
delay(250);
digitalWrite(ledPin, LOW);
delay(250);
digitalWrite(ledPin, HIGH);
delay(250);
digitalWrite(ledPin, LOW);
delay(250);
}
}
}
