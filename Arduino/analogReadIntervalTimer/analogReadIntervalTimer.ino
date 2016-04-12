/*
    This example shows how to use the IntervalTimer library and the ADC library in the tensy 3.0/3.1

    The three important objects are: the ADC, the (one or more) IntervalTimer and the same number of RingBuffer.

    The ADC sets up the internal ADC, you can change the settings as usual. Enable interrupts.
    The IntervalTimer (timerX) executes a function every 'period' microseconds.
    This function, timerX_callback, starts the desired ADC measurement, for example:
        startSingleRead, startSingleDifferential, startSynchronizedSingleRead, startSynchronizedSingleDifferential
    you can use the ADC0 or ADC1 (only for Teensy 3.1).

    When the measurement is done, the adc_isr is executed:
        - If you have more than one timer per ADC module you need to know which pin was measured.
        - Then you store/process the data
        - Finally, if you have more than one timer you can check whether the last measurement interruped a
          a previous one (using adc->adcX->adcWasInUse), and restart it if so.
          The settings of the interrupted measurement are stored in the adc->adcX->adc_config struct.


*/



#include "ADC.h"
#include "RingBuffer.h"
// and IntervalTimer
#include <IntervalTimer.h>

const int ledPin = LED_BUILTIN;

const int readPin0 = A2;
const int period0 = 100; // us

const int readPin1 = A3;
const int period1 = 100; // us

const int readPeriod = 102400; // us

ADC *adc = new ADC(); // adc object

IntervalTimer timer0, timer1; // timers

RingBuffer *buffer0 = new RingBuffer; // buffers to store the values
RingBuffer *buffer1 = new RingBuffer;

int startTimerValue0 = 0, startTimerValue1 = 0;

void setup() {

    pinMode(ledPin, OUTPUT); // led blinks every loop

    pinMode(readPin0, INPUT); pinMode(readPin1, INPUT);

    Serial.begin(9600);

    delay(1000);

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

    Serial.println("Starting Timers");
    
    startTimerValue0 = timer0.begin(timer0_callback, period0);


    adc->enableInterrupts(ADC_0);

    Serial.println("Timers started");

    delay(1000);
}

int value = 0;
char c=0;

void loop() {
    startTimerValue0 = timer0.begin(timer0_callback, period0);
    if(startTimerValue0==false) {
            Serial.println("Timer0 setup failed");
    }

    if(!buffer0->isEmpty()) { // read the values in the buffer
        Serial.print("Read pin 0: ");
        Serial.println(buffer0->read()*3.3/adc->getMaxValue());
        //Serial.println("New value!");
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

    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );

    delayMicroseconds(readPeriod);
    timer0.end();
    while (!buffer0->isEmpty()) {
      double tmp = buffer0->read();
      Serial.print(tmp);Serial.print('\t');
    }
    Serial.print('\n');
}

// This function will be called with the desired frequency
// start the measurement
// in my low-res oscilloscope this function seems to take 1.5-2 us.
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
        buffer0->write(adc->readSingle());
    } else if(pin==readPin1) {
        buffer1->write(adc->readSingle());
        if(adc->adc0->isConverting()) {
            digitalWriteFast(LED_BUILTIN, 1);
        }
    } else { // clear interrupt anyway
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
