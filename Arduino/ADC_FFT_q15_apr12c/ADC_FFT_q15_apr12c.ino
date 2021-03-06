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
ADC *adc = new ADC(); // adc object
IntervalTimer timer0;
// RingBuffer *buffer0 = new RingBuffer; // buffers to store the values
int startTimerValue0 = 0;
// ADC INITIAL

// FFT INITIAL
#define TEST_LENGTH 2048
static q15_t testInput[TEST_LENGTH];
static q15_t testOutput[TEST_LENGTH];
static int kn = 0;

uint32_t fftSize = 1024; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1;
// FFT INITIAL

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
  Serial.println("Starting");
  adc->enableInterrupts(ADC_0);
  Serial.println("Timers started");
  
  delay(1000);
}

int value = 0;
char c=0;

void loop() {
  Serial.println("Starting\n");

  kn = 0;
  startTimerValue0 = timer0.begin(timer0_callback, period0);

  if(startTimerValue0==false) {
          Serial.println("Timer0 setup failed");
  }
  
//  if(!buffer0->isEmpty()) { // read the values in the buffer
//      Serial.print("Read pin 0: ");
//      Serial.println(buffer0->read()*3.3/adc->getMaxValue());
//      Serial.print("\n");
//      //Serial.println("New value!");
//  }


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
  timer0.end();
    
  arm_cfft_q15_app();
}

void timer0_callback(void) {
    adc->startSingleRead(readPin0, ADC_0); // also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential
    //digitalWriteFast(ledPin+1, !digitalReadFast(ledPin+1));
}


// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr() {

    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

    // add value to correct buffer
    if(pin==readPin0) {
        testInput[kn*2] = (adc->readSingle());
        testInput[kn*2+1] = 0;
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
 
 
void arm_cfft_q15_app(void)
{
  uint32_t i;
  arm_cfft_radix4_instance_q15 S;

  arm_cfft_radix4_init_q15(&S, fftSize, ifftFlag, doBitReverse);
   
  for(i=0; i<1024; i++)
  {
    // testInput[i*2+1] = 0;
    // testInput[i*2] = arm_sin_q15(2*3.1415926f*50*i/1000);
    Serial.printf("%d\t", testInput[i*2]);
  }

   Serial.printf("\n****Input Above****\n");
   
  arm_cfft_radix4_q15(&S, testInput);
  arm_copy_q15(testInput, testOutput, 2048);

  for(i=0; i<1024; i++)
  {
    Serial.printf("%d\t", testInput[i*2]);
  }
  Serial.printf("\n****Output Above****\n");
  for(i=0; i<1024; i++)
  {
    Serial.printf("%d\t", testInput[i*2+1]);
  }
  Serial.printf("\n****Output Above****\n");
  
  //ifft test  
  for(i=0; i<1024; i++)
  {
    uint16_t tmp = testOutput[i*2] & 0x8000;
    uint16_t tmp2 = (testOutput[i*2] & 0x7fe0) >> 5;
    testOutput[i*2] = tmp2 + (tmp2 << 1) + (tmp2 << 2) + (tmp2<< 3) + (tmp2 << 4);
    testOutput[i*2] = (testOutput[i*2] & 0x7fff) | tmp;
    Serial.printf("%d\t", testOutput[i*2]);
//    Serial.printf("%d\t", testOutput[i*2+1]);
  }
  Serial.printf("\n****Output Above****\n");
  for(i=0; i<1024; i++)
  {
    uint16_t tmp = testOutput[i*2+1] & 0x8000;
    uint16_t tmp2 = (testOutput[i*2+1] & 0x7fe0) >> 5;
    testOutput[i*2+1] = tmp2 + (tmp2 << 1) + (tmp2 << 2) + (tmp2<< 3) + (tmp2 << 4);
    testOutput[i*2+1] = (testOutput[i*2+1] &  0x7fff) | tmp;
    Serial.printf("%d\t", testOutput[i*2+1]);
  }
  Serial.printf("\n****Output Above****\n");
    
  arm_cfft_radix4_init_q15(&S, fftSize, 1, doBitReverse);
  arm_cfft_radix4_q15(&S, testOutput);
  for(i=0; i<1024; i++)
  {
    testOutput[i*2] = testOutput[i*2] << 10;
    Serial.printf("%d\t", testOutput[i*2]);
//    Serial.printf("%d\t", testOutput[i*2+1]);
  }
  Serial.printf("\n****IFFT Output Above****\n");
  for(i=0; i<1024; i++)
  {
    testOutput[i*2+1] = testOutput[i*2+1] << 10;
    Serial.printf("%d\t", testOutput[i*2+1]);
  }
  //ifft test
  Serial.printf("\n****IFFT Output Above****\n");

//  uint16_t ii = 0;
//  q15_t qq = 0;
//  for (ii = 0; ii < 65536; ii++) {
//    qq = ii;
//    Serial.printf("%d\n", qq);
//  }
}
