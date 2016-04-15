#include "AudioStream.h"
#include "arm_math.h"

#include "ADC.h"
#include "RingBuffer.h"
// and IntervalTimer
#include <IntervalTimer.h>

// ADC INITIAL
const int ledPin = LED_BUILTIN;
const int readPin0 = A2;
const int period0 = 100; // us
const int readPin1 = A3;
const int period1 = 100; // us
const int readPeriod = 102400; // us
static int kn = 0;
ADC *adc = new ADC(); // adc object
IntervalTimer timer0;
// RingBuffer *buffer0 = new RingBuffer; // buffers to store the values
int startTimerValue0 = 0;
// ADC INITIAL

// FFT INITIAL
static q15_t testInput[1024];
// FFT INITIAL

int val;

void setup() {
  
  pinMode(ledPin, OUTPUT); // led blinks every loop
  pinMode(readPin0, INPUT); pinMode(readPin1, INPUT);
  
  ///// ADC0 ////
  // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
  //adc->setReference(ADC_REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc->setAveraging(16); // set number of averages
  adc->setResolution(12); // set bits of resolution

  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  // see the documentation for more information
  adc->setConversionSpeed(ADC_MED_SPEED); // change the conversion speed
  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->setSamplingSpeed(ADC_MED_SPEED); // change the sampling speed
  // with 16 averages, 12 bits resolution and ADC_HIGH_SPEED conversion and sampling it takes about 32.5 us for a conversion
  
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial.println("Starting");
  adc->enableInterrupts(ADC_0);
  Serial.println("Timers started");
  
  delay(1000);
  
}

//int value = 0;
char c=0;

void loop() {
  
  uint16_t i;
//  
  Serial.println("Starting");
  Serial.println(millis()); // <-------

  kn = 0;
  startTimerValue0 = timer0.begin(timer0_callback, period0);
//  Serial.println(millis());

  if(startTimerValue0==false) {
          Serial.println("Timer0 setup failed");
  }

  if (Serial.available()) {
      c = Serial.read();
      if(c=='s') { // stop timer
          Serial.println("Stop timer0");
          timer0.end();
      } else if(c=='r') { // restart timer
          Serial.println("Restart timer0");
          startTimerValue0 = timer0.begin(timer0_callback, period0);
      } else if(c=='p') { // restart timer
          Serial.print("isContinuous: ");
          Serial.println(adc->adc0->isContinuous());
      }
  }

  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
  delayMicroseconds(readPeriod);
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
  timer0.end(); // <------- Question here, maybe

  Serial.println("Serial Output in ms:");
  unsigned long tt1 = millis(); // <-------
  for(i=0; i<1024; i++)
  {
    Serial1.printf("%d\t", testInput[i]);
  }
  unsigned long tt2 = millis(); // <-------
  Serial.println(tt2-tt1); // <-------
  Serial1.printf("\n****testOutput Above****\n");

//  Serial1.write('A');

//  // receive data from another XBee module
//  val = Serial1.read();
//  Serial.print(val);
//  if (-1 != val) {
//    if ('A' == val) {
//      digitalWrite(ledPin, HIGH);
//      delay(250);
//      digitalWrite(ledPin, LOW);
//      delay(250);
//      digitalWrite(ledPin, HIGH);
//      delay(250);
//      digitalWrite(ledPin, LOW);
//      delay(250);
//    }
//  }

}

void timer0_callback(void) {

    adc->startSingleRead(readPin0, ADC_0); // also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential

}


// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr() {

    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

    // add value to correct buffer
    if(pin==readPin0) {
        testInput[kn] = (adc->readSingle());
        kn++;
        if (kn >= 1024) {
          kn = 0;
          timer0.end();
        }
    } 
    else { // clear interrupt anyway
        adc->readSingle();
    }

    // restore ADC config if it was in use before being interrupted by the analog timer
    if (adc->adc0->adcWasInUse) {
        // restore ADC config, and restart conversion
        adc->adc0->loadConfig(&adc->adc0->adc_config);
        // avoid a conversion started by this isr to repeat itself
        adc->adc0->adcWasInUse = false;
    }

}

