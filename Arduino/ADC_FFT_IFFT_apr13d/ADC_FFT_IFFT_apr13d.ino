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
static float32_t testInput[TEST_LENGTH];
static float32_t testOutput[TEST_LENGTH];
static float32_t testOutput2[TEST_LENGTH*2];
static int kn = 0;
uint32_t fftSize = 1024;
uint32_t fftSize2 = 2048;
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
//  uint16_t i;
  
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
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
  timer0.end();

//  for(i=0; i<1024; i++)
//  {
//    Serial.printf("%f\t", testInput[i*2]);
//  }
//  Serial.printf("\n****testInput Above****\n");
//  for(i=0; i<1024; i++)
//  {
//    Serial.printf("%f\t", testInput[i*2+1]);
//  }
//  Serial.printf("\n****testInput Above****\n");
    
  arm_cfft_f32_app(testInput);
  arm_cifft_f32_app(testOutput);
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
 
 
void arm_cfft_f32_app(float32_t *data)
{
//  uint16_t i;
  arm_copy_f32(data, testOutput, 2048);
  arm_cfft_radix4_instance_f32 S;
  
  // fft
  arm_cfft_radix4_init_f32(&S, fftSize, 0, 1);
  arm_cfft_radix4_f32(&S, testOutput);
  // fft

//  for(i=0; i<1024; i++)
//  {
//    Serial.printf("%f\t", testOutput[i*2]);
////    Serial.printf("%f\t", testOutput[i*2+1]);
//  }
//  Serial.printf("\n****testOutput Above****\n");
//  for(i=0; i<1024; i++)
//  {
//    Serial.printf("%f\t", testOutput[i*2+1]);
//  }
//  Serial.printf("\n****testOutput Above****\n");
//    
//  Serial.printf("****Finshed****\n");
}

void arm_cifft_f32_app(float32_t *data)
{
//  uint16_t i;
  arm_copy_f32(data, testOutput, 2048);
  arm_cfft_radix4_instance_f32 S;
  // ifft
  arm_cfft_radix4_init_f32(&S, fftSize, 1, 1);
  arm_cfft_radix4_f32(&S, testOutput);
  // ifft

//  for(i=0; i<1024; i++)
//  {
//    Serial.printf("%f\t", testOutput[i*2]);
//  }
//    Serial.printf("\n****IFFT Output Above****\n");
//  for(i=0; i<1024; i++)
//  {
//    Serial.printf("%f\t", testOutput[i*2+1]);
//  }
//  Serial.printf("\n****IFFT Output Above****\n");
//      
//  Serial.printf("****Finshed****\n");
}

void arm_cfft_f32_app(float32_t *data)
{
  arm_copy_f32(data, testOutput2, 4096);
  arm_cfft_radix4_instance_f32 S;
  // fft
  arm_cfft_radix4_init_f32(&S, fftSize2, 0, 1);
  arm_cfft_radix4_f32(&S, testOutput2);
  // fft
}

void arm_cifft_f32_app(float32_t *data)
{
  arm_copy_f32(data, testOutput2, 4096);
  arm_cfft_radix4_instance_f32 S;
  // ifft
  arm_cfft_radix4_init_f32(&S, fftSize2, 1, 1);
  arm_cfft_radix4_f32(&S, testOutput2);
  // ifft
}
