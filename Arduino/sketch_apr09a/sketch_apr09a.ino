const int ANALOG_READ_AVERAGING = 10;  // Number of samples to average with each ADC reading.
const int ANALOG_READ_RESOLUTION = 12; // Bits of resolution for the ADC.
const int Q1Pin = 16;
const int I1Pin = 17;
//const int gain = 2;

void setup() {
  //ADC::Sync_result result = adc->analogSyncRead(pin1, pin2);
  //enablePGA(gain);
  adc->setReference(ADC_REF_3V3, ADC_0);
  pinMode(ledPin, OUTPUT);
  Serial1.begin(19200);
  pinMode (Q1Pin, INPUT);
  pinMode (I1Pin, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
}

void loop() {
  Serial1.write('A');
  
}
