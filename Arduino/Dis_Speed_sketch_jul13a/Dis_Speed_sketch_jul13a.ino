#include <AD9833.h>
#include <math.h>

#include "Arduino.h"
#include "AudioStream.h"
#include "arm_math.h"

#include "ADC.h"
#include "RingBuffer.h"
// and IntervalTimer
#include <IntervalTimer.h>

const float32_t winGauss512[] =
{
  0.0439369336234074, 0.0450228098987249, 0.0461311061351582, 0.0472621594091714, 0.0484163084621372, 0.0495938936098565, 0.0507952566491003, 0.0520207407611572, 0.0532706904123688, 0.0545454512516385, 0.0558453700048986, 0.0571707943665277, 0.0585220728877032, 0.0598995548616859, 0.0613035902060271, 0.0627345293416955, 0.0641927230691215, 0.0656785224411575, 0.0671922786329582, 0.0687343428087834, 0.0703050659857283, 0.0719047988943934, 0.0735338918365012, 0.0751926945394732, 0.0768815560079847, 0.0786008243725127, 0.0803508467348979, 0.0821319690109445, 0.0839445357700802, 0.0857888900721078, 0.0876653733010760, 0.0895743249963023, 0.0915160826805858, 0.0934909816856459, 0.0954993549748283, 0.0975415329631239, 0.0996178433345437, 0.101728610856902, 0.103874157194058, 0.106054800715674, 0.108270856304542, 0.110522635161541, 0.112810444608303, 0.115134587887620, 0.117495363961705, 0.119893067308341, 0.122327987715013, 0.124800410071106, 0.127310614158225, 0.129858874438751, 0.132445459842696, 0.135070633552962, 0.137734652789088, 0.140437768589584, 0.143180225592948, 0.145962261817467, 0.148784108439914, 0.151645989573233, 0.154548122043332, 0.157490715165097, 0.160473970517731, 0.163498081719551, 0.166563234202348, 0.169669604985442, 0.172817362449558, 0.176006666110643, 0.179237666393759, 0.182510504407185, 0.185825311716866, 0.189182210121332, 0.192581311427247, 0.196022717225708, 0.199506518669457, 0.203032796251138, 0.206601619582748, 0.210213047176436, 0.213867126226803, 0.217563892394839, 0.221303369593673, 0.225085569776279, 0.228910492725293, 0.232778125845105, 0.236688443956388, 0.240641409093212, 0.244636970302917, 0.248675063448902, 0.252755611016491, 0.256878521922041, 0.261043691325465, 0.265251000446311, 0.269500316383593, 0.273791491939512, 0.278124365447247, 0.282498760602977, 0.286914486302296, 0.291371336481190, 0.295869089961736, 0.300407510302687, 0.304986345655106, 0.309605328623212, 0.314264176130594, 0.318962589291958, 0.323700253290558, 0.328476837261478, 0.333291994180903, 0.338145360761558, 0.343036557354432, 0.347965187856973, 0.352930839627872, 0.357933083408597, 0.362971473251809, 0.368045546456815, 0.373154823512172, 0.378298808045601, 0.383476986781322, 0.388688829504949, 0.393933789036066, 0.399211301208605, 0.404520784859143, 0.409861641823231, 0.415233256939873, 0.420634998064244, 0.426066216088761, 0.431526244972611, 0.437014401779801, 0.442529986725858, 0.448072283233219, 0.453640557995434, 0.459234061050213, 0.464852025861425, 0.470493669410078, 0.476158192294368, 0.481844778838828, 0.487552597212636, 0.493280799557121, 0.499028522122500, 0.504794885413885, 0.510578994346576, 0.516379938410654, 0.522196791844912, 0.528028613820092, 0.533874448631460, 0.539733325900698, 0.545604260787107, 0.551486254208090, 0.557378293068906, 0.563279350501654, 0.569188386113445, 0.575104346243731, 0.581026164230717, 0.586952760686823, 0.592883043783111, 0.598815909542613, 0.604750242142486, 0.610684914224898, 0.616618787216560, 0.622550711656808, 0.628479527534117, 0.634404064630955, 0.640323142876834, 0.646235572709452, 0.652140155443788, 0.658035683649000, 0.663920941533001, 0.669794705334546, 0.675655743722673, 0.681502818203346, 0.687334683533106, 0.693150088139578, 0.698947774548635, 0.704726479818029, 0.710484935977308, 0.716221870473798, 0.721936006624460, 0.727626064073397, 0.733290759254804, 0.738928805861125, 0.744538915316201, 0.750119797253163, 0.755670159996841, 0.761188711050437, 0.766674157586221, 0.772125206939990, 0.777540567109037, 0.782918947253366, 0.788259058199886, 0.793559612949321, 0.798819327185553, 0.804036919787128, 0.809211113340641, 0.814340634655722, 0.819424215281323, 0.824460592023044, 0.829448507461170, 0.834386710469156, 0.839273956732248, 0.844109009265950, 0.848890638934028, 0.853617624965757, 0.858288755472113, 0.862902827960591, 0.867458649848365, 0.871955038973464, 0.876390824103681, 0.880764845442891, 0.885075955134486, 0.889323017761611, 0.893504910843911, 0.897620525330473, 0.901668766088668, 0.905648552388592, 0.909558818382799, 0.913398513581044, 0.917166603319723, 0.920862069225732, 0.924483909674445, 0.928031140241524, 0.931502794148283, 0.934897922700314, 0.938215595719108, 0.941454901966385, 0.944614949560872, 0.947694866387255, 0.950693800497050, 0.953610920501127, 0.956445415953641, 0.959196497727115, 0.961863398378444, 0.964445372505565, 0.966941697094578, 0.969351671857081, 0.971674619557503, 0.973909886330218, 0.976056841986239, 0.978114880309279, 0.980083419340997, 0.981961901655234, 0.983749794621054, 0.985446590654430, 0.987051807458394, 0.988564988251498, 0.989985701984440, 0.991313543544700, 0.992548133949057, 0.993689120523860, 0.994736177072928, 0.995689004032970, 0.996547328616422, 0.997310904941606, 0.997979514150118, 0.998552964511376, 0.999031091514249, 0.999413757945719, 0.999700853956499, 0.999892297113595, 0.999988032439748, 0.999988032439748, 0.999892297113595, 0.999700853956499, 0.999413757945719, 0.999031091514249, 0.998552964511376, 0.997979514150118, 0.997310904941606, 0.996547328616422, 0.995689004032970, 0.994736177072928, 0.993689120523860, 0.992548133949057, 0.991313543544700, 0.989985701984440, 0.988564988251498, 0.987051807458394, 0.985446590654430, 0.983749794621054, 0.981961901655234, 0.980083419340997, 0.978114880309279, 0.976056841986239, 0.973909886330218, 0.971674619557503, 0.969351671857081, 0.966941697094578, 0.964445372505565, 0.961863398378444, 0.959196497727115, 0.956445415953641, 0.953610920501127, 0.950693800497050, 0.947694866387255, 0.944614949560872, 0.941454901966385, 0.938215595719108, 0.934897922700314, 0.931502794148283, 0.928031140241524, 0.924483909674445, 0.920862069225732, 0.917166603319723, 0.913398513581044, 0.909558818382799, 0.905648552388592, 0.901668766088668, 0.897620525330473, 0.893504910843911, 0.889323017761611, 0.885075955134486, 0.880764845442891, 0.876390824103681, 0.871955038973464, 0.867458649848365, 0.862902827960591, 0.858288755472113, 0.853617624965757, 0.848890638934028, 0.844109009265950, 0.839273956732248, 0.834386710469156, 0.829448507461170, 0.824460592023044, 0.819424215281323, 0.814340634655722, 0.809211113340641, 0.804036919787128, 0.798819327185553, 0.793559612949321, 0.788259058199886, 0.782918947253366, 0.777540567109037, 0.772125206939990, 0.766674157586221, 0.761188711050437, 0.755670159996841, 0.750119797253163, 0.744538915316201, 0.738928805861125, 0.733290759254804, 0.727626064073397, 0.721936006624460, 0.716221870473798, 0.710484935977308, 0.704726479818029, 0.698947774548635, 0.693150088139578, 0.687334683533106, 0.681502818203346, 0.675655743722673, 0.669794705334546, 0.663920941533001, 0.658035683649000, 0.652140155443788, 0.646235572709452, 0.640323142876834, 0.634404064630955, 0.628479527534117, 0.622550711656808, 0.616618787216560, 0.610684914224898, 0.604750242142486, 0.598815909542613, 0.592883043783111, 0.586952760686823, 0.581026164230717, 0.575104346243731, 0.569188386113445, 0.563279350501654, 0.557378293068906, 0.551486254208090, 0.545604260787107, 0.539733325900698, 0.533874448631460, 0.528028613820092, 0.522196791844912, 0.516379938410654, 0.510578994346576, 0.504794885413885, 0.499028522122500, 0.493280799557121, 0.487552597212636, 0.481844778838828, 0.476158192294368, 0.470493669410078, 0.464852025861425, 0.459234061050213, 0.453640557995434, 0.448072283233219, 0.442529986725858, 0.437014401779801, 0.431526244972611, 0.426066216088761, 0.420634998064244, 0.415233256939873, 0.409861641823231, 0.404520784859143, 0.399211301208605, 0.393933789036066, 0.388688829504949, 0.383476986781322, 0.378298808045601, 0.373154823512172, 0.368045546456815, 0.362971473251809, 0.357933083408597, 0.352930839627872, 0.347965187856973, 0.343036557354432, 0.338145360761558, 0.333291994180903, 0.328476837261478, 0.323700253290558, 0.318962589291958, 0.314264176130594, 0.309605328623212, 0.304986345655106, 0.300407510302687, 0.295869089961736, 0.291371336481190, 0.286914486302296, 0.282498760602977, 0.278124365447247, 0.273791491939512, 0.269500316383593, 0.265251000446311, 0.261043691325465, 0.256878521922041, 0.252755611016491, 0.248675063448902, 0.244636970302917, 0.240641409093212, 0.236688443956388, 0.232778125845105, 0.228910492725293, 0.225085569776279, 0.221303369593673, 0.217563892394839, 0.213867126226803, 0.210213047176436, 0.206601619582748, 0.203032796251138, 0.199506518669457, 0.196022717225708, 0.192581311427247, 0.189182210121332, 0.185825311716866, 0.182510504407185, 0.179237666393759, 0.176006666110643, 0.172817362449558, 0.169669604985442, 0.166563234202348, 0.163498081719551, 0.160473970517731, 0.157490715165097, 0.154548122043332, 0.151645989573233, 0.148784108439914, 0.145962261817467, 0.143180225592948, 0.140437768589584, 0.137734652789088, 0.135070633552962, 0.132445459842696, 0.129858874438751, 0.127310614158225, 0.124800410071106, 0.122327987715013, 0.119893067308341, 0.117495363961705, 0.115134587887620, 0.112810444608303, 0.110522635161541, 0.108270856304542, 0.106054800715674, 0.103874157194058, 0.101728610856902, 0.0996178433345437, 0.0975415329631239, 0.0954993549748283, 0.0934909816856459, 0.0915160826805858, 0.0895743249963023, 0.0876653733010760, 0.0857888900721078, 0.0839445357700802, 0.0821319690109445, 0.0803508467348979, 0.0786008243725127, 0.0768815560079847, 0.0751926945394732, 0.0735338918365012, 0.0719047988943934, 0.0703050659857283, 0.0687343428087834, 0.0671922786329582, 0.0656785224411575, 0.0641927230691215, 0.0627345293416955, 0.0613035902060271, 0.0598995548616859, 0.0585220728877032, 0.0571707943665277, 0.0558453700048986, 0.0545454512516385, 0.0532706904123688, 0.0520207407611572, 0.0507952566491003, 0.0495938936098565, 0.0484163084621372, 0.0472621594091714, 0.0461311061351582, 0.0450228098987249, 0.0439369336234074
};

const float32_t winGauss100[] =
{
  0.043936934, 0.049786433, 0.056270983, 0.063438107, 0.0713359, 0.080012583, 0.089515998, 0.099893043, 0.111189057, 0.12344715, 0.13670749, 0.151006545, 0.166376296, 0.182843426, 0.200428496, 0.21914512, 0.238999152, 0.259987899, 0.282099377, 0.305311625, 0.329592083, 0.354897077, 0.381171386, 0.40834795, 0.436347696, 0.465079517, 0.49444041, 0.524315772, 0.554579875, 0.58509651, 0.615719812, 0.646295256, 0.676660814, 0.706648278, 0.736084719, 0.76479408, 0.792598876, 0.81932198, 0.844788471, 0.868827522, 0.891274293, 0.911971801, 0.930772746, 0.94754125, 0.962154492, 0.974504204, 0.984498007, 0.99206056, 0.997134508, 0.999681206, 0.999681206, 0.997134508, 0.99206056, 0.984498007, 0.974504204, 0.962154492, 0.94754125, 0.930772746, 0.911971801, 0.891274293, 0.868827522, 0.844788471, 0.81932198, 0.792598876, 0.76479408, 0.736084719, 0.706648278, 0.676660814, 0.646295256, 0.615719812, 0.58509651, 0.554579875, 0.524315772, 0.49444041, 0.465079517, 0.436347696, 0.40834795, 0.381171386, 0.354897077, 0.329592083, 0.305311625, 0.282099377, 0.259987899, 0.238999152, 0.21914512, 0.200428496, 0.182843426, 0.166376296, 0.151006545, 0.13670749, 0.12344715, 0.111189057, 0.099893043, 0.089515998, 0.080012583, 0.0713359, 0.063438107, 0.056270983, 0.049786433, 0.043936934
};

// ADC INITIAL
const int ledPin = LED_BUILTIN;
const int readPin0 = A2;
const int readPin1 = A3;
const int readPin2 = A4;
static int period0 = 100; // us
static int readPeriod = 51200; // us
static int mode = 0; // Mode: Speed or Distance Measurement
ADC *adc = new ADC(); // adc object
ADC::Sync_result result;
IntervalTimer timer0;
// RingBuffer *buffer0 = new RingBuffer; // buffers to store the values
int startTimerValue0 = 0;
// ADC INITIAL

static float32_t bias = 0; // BIAS for input

// FFT INITIAL
#define TEST_LENGTH 1024
static float32_t testInput[TEST_LENGTH];
static int testInput1[TEST_LENGTH / 2];
static int testInput2[TEST_LENGTH / 2];
static float32_t testOutput[TEST_LENGTH];
static float32_t testOutput2[TEST_LENGTH * 2];
static float32_t a0, a1;
// static float32_t w0, w1;
static float32_t aa[TEST_LENGTH];
static float32_t ww[TEST_LENGTH * 2];
static int kn = 0;
uint32_t fftSize = 512;
uint32_t fftSize2 = 1024;
// FFT INITIAL

static int sshow = 0;
int val;
elapsedMicros etime;

void setup() {

//***************************
//    Pin assign      
//    Teensy3.1     AD9833
//    A5    ---> FSYNC
//    A6    ---> SCK
//    A7    ---> DAT
//    A8    ---> CS
//*************************** 


  pinMode(ledPin, OUTPUT); // led blinks every loop
  pinMode(readPin0, INPUT); pinMode(readPin1, INPUT);

  Serial.begin(9600);
  Serial3.begin(19200);

  ///// ADC0 ////
  // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
  adc->setReference(ADC_REF_3V3, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc->setAveraging(8); // set number of averages
  adc->setResolution(12); // set bits of resolution

  adc->setConversionSpeed(ADC_HIGH_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_HIGH_SPEED); // change the sampling speed

  ////// ADC1 /////
  adc->setReference(ADC_REF_3V3, ADC_1);
  adc->setAveraging(8, ADC_1); // set number of averages
  adc->setResolution(12, ADC_1); // set bits of resolution

  adc->setConversionSpeed(ADC_HIGH_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_HIGH_SPEED, ADC_1); // change the sampling speed

  Serial.println("Starting");
  adc->enableInterrupts(ADC_0);
  Serial.println("Timers started");
  
  AD9833_Init();
  AD9833_Write(0x0140); // Cut off DAC
  AD9833_AmpSet(0);

  delay(2000);
  
  AD9833_WaveSeting(100, 0, TRI_WAVE, 0);
  AD9833_AmpSet(106); //106 --> 1.5V, offset 0.2V

}

//int value = 0;
char c = 0;
float32_t tdis[20];
uint8_t tdis_i = 0;
uint8_t tsp_i = 0;
float32_t df1, df2;
int i_max;

void loop() {
  if (mode == 0)
  {
    //int value = 0;
    char c=0;
    float32_t tspeed[20];      
    int i, deltaf;
    int f1 = 0, f2 = 200;
    float32_t testTmp[TEST_LENGTH*2];
 
    kn = 0;
    startTimerValue0 = timer0.begin(timer0_callback, period0);

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
          else if (c == 't')
        {
          AD9833_WaveSeting(100, 0, TRI_WAVE, 0);
          AD9833_AmpSet(106); //106 --> 1.5V, offset 0.2V
          mode = 1;
          period0 = 50; // us
          readPeriod = 25000; // us
        }
          else if (c == 'n')
        {
          AD9833_Write(0x0140); // Cut off DAC
          AD9833_AmpSet(0);
          mode = 0;
          period0 = 100; // us
          readPeriod = 51200; // us
        }
          else if ('A' == c) {
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

    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    delayMicroseconds(readPeriod);
    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    timer0.end();

    bias = 0;

    for(i=0; i<512; i++)
    {
      bias = bias + testInput[i*2] / 512.0;
    }
    
    // ADD WINDOW FOR INPUT
    apply_window0(testInput, winGauss512);
    // ADD WINDOW FOR INPUT

    arm_cfft_f32_app(testInput);

    arm_cmplx_mag_f32(testOutput, testOutput, 512);

    for (i = 0; i < 256; i++) {
      testOutput[i] = 20*log10(testOutput[i] / 1240); // 1 vdB
    }

    for (i = 255; i > 0; i--) {
      if (testOutput[i] > - 1) break;
    }

    //////// CZT INITIAL ////////
    deltaf = i * 10000 / 512;

    if  (deltaf < 500) {
      f1 = 0; f2 = 1000; // in hertz
    }
    else if (deltaf >= 4500) {
      f1 = 4000; f2 = 5000;
    }
    else {
      i = floor(deltaf/1000.0);
      f1 = int(i*1000 - 500); f2 = int(i*1000 + 500);
    }

    a0 = arm_cos_f32(2*PI*f1/10000);
    a1 = -arm_sin_f32(2*PI*f1/10000);
    aa[0] = 1;
    aa[1] = 0;

    for(int i = 0; i < 512; i++)
    {
      float32_t tmp_r, tmp_i, tmpa_r, tmpa_i;
      tmp_r = arm_cos_f32(i * i * PI*(f2-f1)/512/10000);
      tmp_i = -arm_sin_f32(i * i * PI*(f2-f1)/512/10000);
      ww[(511 + i) * 2] = tmp_r;
      ww[(511 + i) * 2 + 1] = tmp_i;
      ww[(511 - i) * 2] = tmp_r;
      ww[(511 - i) * 2 + 1] = tmp_i;
      aa[i*2 + 2] = a0 * aa[i*2] - a1 * aa[i*2 + 1];
      aa[i*2 + 3] = a0 * aa[i*2 + 1] + a1 * aa[i*2];
      tmpa_r = aa[i*2] * tmp_r - aa[i*2 + 1] * tmp_i;
      tmpa_i = aa[i*2] * tmp_i + aa[i*2 + 1] * tmp_r;
      aa[i*2] = tmpa_r;
      aa[i*2 + 1] = tmpa_i;
    }
    ww[2046] = 0;
    ww[2047] = 0;
    
    arm_cmplx_conj_f32(ww, ww, 1024);
    //////// CZT INITIAL ////////

    arm_cmplx_mult_cmplx_f32(testInput, aa, testInput, 512);

    for (i=0; i<1024; i++)
    {
      testTmp[i] = testInput[i];
    }

    for (i=1024; i<2048; i++)
    {
      testTmp[i] = 0;
    }

    arm_cfft2_f32_app(testTmp); // The output is testOutput2

    arm_copy_f32(testOutput2, testTmp, 2048);

    arm_cfft2_f32_app(ww);

    arm_cmplx_mult_cmplx_f32(testOutput2, testTmp, testTmp, 1024);

    arm_cifft2_f32_app(testTmp);

    for(i=0 ; i<512; i++)
    {
      testInput[i*2] = testOutput2[(511+i)*2] * ww[(511+i)*2] + testOutput2[(511+i)*2 + 1] * ww[(511+i)*2 + 1];
      testInput[i*2 + 1] = testOutput2[(511+i)*2 + 1] * ww[(511+i)*2] - testOutput2[(511+i)*2] * ww[(511+i)*2 + 1];
    }

    arm_cmplx_mag_f32(testInput, testInput, 512); //input buffer, real output buffer, number of complex samples in the input vector

    for (i = 0; i < 512; i++)
    {
      testInput[i] = 20*log10(testInput[i] / 1240);
    }

//    for(i=0; i<512; i++)
//    {
//      Serial.printf("%f\t", testInput[i]);
//    }
//  //  Serial.printf("\ncft above\n");
//    Serial.printf("\n");
    
    for (i = 511; i > 0; i--)
    {
      if (testInput[i] >=  -1)
      {
        break;
      }
    }

    tspeed[tsp_i] = f1 + i*1000/512.0; // f

    tspeed[tsp_i] = tspeed[tsp_i] / 161.0;  // f/(8.05*2*10)

    if (tspeed[tsp_i] < 0.125) tspeed[tsp_i] = 0;

  //  Serial.printf("The speed is: %f m/s \n", tspeed[tsp_i]);
    if (tsp_i >= 4) {
      Serial3.printf("Speed:\n");
      for (int j = 0; j < 5; j++) {
        Serial3.print(etime,DEC);
        Serial3.printf(" %f\n", tspeed[j]);
      }
      delay(50);
      tsp_i = 0;
    }
    else {
//      Serial3.printf("tsp: %d\n", tsp_i);
      tsp_i ++;
    }

    // receive data from another XBee module
    // Serial.print(val);
    if(Serial3.available())
    {
      val = Serial3.read();
      // Serial.print(val);
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
        else if (val == 't')
        {
          AD9833_WaveSeting(100, 0, TRI_WAVE, 0);
          AD9833_AmpSet(106); //106 --> 1.5V, offset 0.2V
          mode = 1;
          period0 = 50; // us
          readPeriod = 25000; // us
        }
        else if (val == 'n')
        {
          AD9833_Write(0x0140); // Cut off DAC
          AD9833_AmpSet(0);
          mode = 0;
          period0 = 100; // us
          readPeriod = 51200; // us
        }
        else if (val == '0')
        {
          sshow = 0;
        }
        else if (val == '1')
        {
          sshow = 1;
        }
      }
    }
  }

  if (mode == 1)
  {
    int i; // ------->, deltaf;
    int f1 = 0, f2 = 10000, Fs = 20000;

    // Data sampling

    kn = 0;
    startTimerValue0 = timer0.begin(timer0_callback, period0);
    //  Serial.println(millis());

    if (startTimerValue0 == false) {
      Serial.println("Timer0 setup failed");
    }

    c = 0;

    if (Serial.available()) {
     c = Serial.read();
     if (c == 's') { // stop timer
       Serial.println("Stop timer0");
       timer0.end();
     } else if (c == 'r') { // restart timer
       Serial.println("Restart timer0");
       startTimerValue0 = timer0.begin(timer0_callback, period0);
     } else if (c == 'p') { // restart timer
       Serial.print("isContinuous: ");
       Serial.println(adc->adc0->isContinuous());
     }
     else if (c == 't')
     {
       AD9833_WaveSeting(100, 0, TRI_WAVE, 0);
       AD9833_AmpSet(106); //106 --> 1.5V, offset 0.2V
       mode = 1;
       period0 = 50; // us
       readPeriod = 25000; // us
     }
     else if (c == 'n')
     {
       AD9833_Write(0x0140); // Cut off DAC
       AD9833_AmpSet(0);
       mode = 0;
       period0 = 100; // us
       readPeriod = 51200; // us
     }
     else if ('A' == c) {
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

    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    delayMicroseconds(readPeriod);
    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    timer0.end();
    // Data sampling

    bias = 0;
    for (i = 0; i < 500; i++)
    {
      bias += testInput2[i]/500.0;
    }

    int tmp = 0;
    for (i = 0; i < 100; i++)
    {
      if (testInput1[tmp] > testInput1[i]) // m@713
        tmp = i;
    }

    //////// INITIAL BEGIN////////
    a0 = arm_cos_f32(2*PI*f1/Fs);
    a1 = -arm_sin_f32(2*PI*f1/Fs);
    aa[0] = 1;
    aa[1] = 0;
    int kk = 100 - 1, m = 512;

    for(int i = 0; i < m; i++)
    {
      float32_t tmp_r, tmp_i, tmpa_r, tmpa_i;
      tmp_r = arm_cos_f32(i * i * PI*(f2-f1)/m/Fs);
      tmp_i = -arm_sin_f32(i * i * PI*(f2-f1)/m/Fs);
      ww[(kk + i) * 2] = tmp_r;
      ww[(kk + i) * 2 + 1] = tmp_i;
      if (kk - i >= 0)
      {
        ww[(kk - i) * 2] = tmp_r;
        ww[(kk - i) * 2 + 1] = tmp_i;
        aa[i*2 + 2] = a0 * aa[i*2] - a1 * aa[i*2 + 1];
        aa[i*2 + 3] = a0 * aa[i*2 + 1] + a1 * aa[i*2];
        tmpa_r = aa[i*2] * tmp_r - aa[i*2 + 1] * tmp_i;
        tmpa_i = aa[i*2] * tmp_i + aa[i*2 + 1] * tmp_r;
        aa[i*2] = tmpa_r;     // final aa
        aa[i*2 + 1] = tmpa_i; // final aa
      }
    }
    for(int i = kk + m; i < 2*m-1; i++)
    {
      ww[i * 2] = 0;
      ww[i * 2 + 1] = 0;
    }
    
    arm_cmplx_conj_f32(ww, ww, 1024); // equals 1/ww
    //////// INITIAL FINISH////////

    /// CAL_FRE ///
    for (i = 0; i < 100; i++)
    {
      testInput[i*2] = testInput2[i + tmp] - bias;
      testInput[i*2 + 1] = 0;
    //    Serial.printf("%f\t", testInput[i*2]);
    }
    //  Serial.printf("\ntestInput\n");

    // ADD WINDOW FOR INPUT
    apply_window1(testInput, winGauss100);
    // ADD WINDOW FOR INPUT

    cal_fre(kk, m);
    i_max = 0;
    for(i = 0; i < 150; i++)
    {
      if (testInput[i] > testInput[i_max])
        i_max = i;
    }
    df1 = i_max/512.0 * f2;
    // Serial.printf("df1 %f\n", df1);
    // Serial3.printf("df1 %f\n", df1);

    for (i = 0; i < 100; i++)
    {
      testInput[i*2] = testInput2[i + tmp + 100] - bias;
      testInput[i*2 + 1] = 0;
    }

    cal_fre(kk, m);
    i_max = 0;
    for(i = 0; i < 150; i++) // 150 -------> m
    {
      if (testInput[i] > testInput[i_max])
        i_max = i;
    }
    df2 = i_max/512.0 * f2;
    // Serial.printf("df2 %f\n", df2);
    // Serial3.printf("df2 %f\n", df2);
    /// CAL_FRE ///

    float32_t dis = 0;
    dis = (df1+df2)/2 * 3/512 * 1.7 * 1.6/1.5;

    tdis[tdis_i] = dis;

    if (tdis[tdis_i] < 2.5) tdis[tdis_i] = 0;

    //  Serial.printf("The speed is: %f m/s \n", tdis[tdis_i]);
    if (tdis_i >= 4) {
      Serial3.printf("Distance:\n");
      for (int j = 0; j < 5; j++) {
//        if (tdis[j] != 0) {
          Serial3.print(etime,DEC);
          Serial3.printf(" %f\n", tdis[j]);
//        }
      }
      delay(50);
      tdis_i = 0;
    }
    else {
      tdis_i ++;
    }

    // receive data from another XBee module
    if(Serial3.available())
    {
      val = Serial3.read();
      // Serial.print(val);
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
        else if (val == 't')
        {
          AD9833_WaveSeting(100, 0, TRI_WAVE, 0);
          AD9833_AmpSet(106); //106 --> 1.5V, offset 0.2V
          mode = 1;
          period0 = 50; // us
          readPeriod = 25000; // us
        }
        else if (val == 'n')
        {
          AD9833_Write(0x0140); // Cut off DAC
          AD9833_AmpSet(0);
          mode = 0;
          period0 = 100; // us
          readPeriod = 51200; // us
        }
        else if (val == '0')
        {
          sshow = 0;
        }
        else if (val == '1')
        {
          sshow = 1;
        }
      }
    }
  }

}

void timer0_callback(void) {
  if (mode == 1) {
      adc->startSynchronizedSingleRead(readPin0, readPin1);
  }
  else
    adc->startSingleRead(readPin2, ADC_0);
}


// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr()
{
  if (mode == 1) {
    result = adc->readSynchronizedSingle();
    testInput1[kn] = (uint16_t)result.result_adc0;
    //        testInput[kn*2+1] = 0;
    testInput2[kn] =  (uint16_t)result.result_adc1;
    Serial.printf("%d %d@tri\n", testInput1[kn], testInput2[kn]);
    kn++;
    if (kn >= 500) {
      kn = 0;
      timer0.end();
    }
  }
  else {
//    testInput[kn*2] = (adc->readSingle());
    testInput[kn*2+1] = 0;
//    Serial.printf("%f@none\n", testInput[kn*2]);
    Serial.println(adc->readSingle(), DEC);
    kn++;
    if (kn >= 512) {
      kn = 0;
      timer0.end();
    }
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
  arm_copy_f32(data, testOutput, 1024);
  arm_cfft_radix2_instance_f32 S;

  // for(int i=0; i<512; i++)
  // {
  //   Serial.printf("%f\t", testOutput[i*2]);
  //   Serial.printf("%f\t", testOutput[i*2+1]);
  // }
  // Serial.printf("\n****testInput Above****\n");
  
  // fft
  arm_cfft_radix2_init_f32(&S, fftSize, 0, 1);
  arm_cfft_radix2_f32(&S, testOutput);
  // fft

  // for(int i=0; i<512; i++)
  // {
  //   Serial.printf("%f\t", testOutput[i*2]);
  //   Serial.printf("%f\t", testOutput[i*2+1]);
  // }
  // Serial.printf("\n****testOutput Above****\n");
}

void arm_cfft2_f32_app(float32_t *data)
{
  arm_copy_f32(data, testOutput2, 2048);
  arm_cfft_radix4_instance_f32 S;
  // fft
  arm_cfft_radix4_init_f32(&S, fftSize2, 0, 1);
  arm_cfft_radix4_f32(&S, testOutput2);
  // fft
}

void arm_cifft2_f32_app(float32_t *data)
{
  arm_copy_f32(data, testOutput2, 2048);
  arm_cfft_radix4_instance_f32 S;
  // ifft
  arm_cfft_radix4_init_f32(&S, fftSize2, 1, 1);
  arm_cfft_radix4_f32(&S, testOutput2);
  // ifft
}

static void apply_window0(float32_t *buf, const float32_t *win)
{
 for (int i=0; i < 512; i++) {
   *buf = (*buf - bias) * *win++;
   buf += 2;
 }

}

void cal_fre(int kk, int m)
{
  int i = 0;
  float32_t testTmp[TEST_LENGTH * 2];
  // Serial.println(micros()); // <-------
  // Serial.println(micros()); // <-------
  arm_cmplx_mult_cmplx_f32(testInput, aa, testInput, kk+1); // y = x.*aa, the length is kk (measured)
  // Serial.println(micros()); // <-------
  for (i = 0; i < (kk+1) * 2; i++)
  {
    testTmp[i] = testInput[i];
  }

  for (i = (kk+1)*2; i < 2048; i++)
  {
    testTmp[i] = 0;
  }

  arm_cfft2_f32_app(testTmp); // The output is testOutput2

  arm_copy_f32(testOutput2, testTmp, 2048);

  arm_cfft2_f32_app(ww);

  arm_cmplx_mult_cmplx_f32(testOutput2, testTmp, testTmp, 1024);

  arm_cifft2_f32_app(testTmp);

  for(i = 0 ; i < m; i++)
  {
    testInput[i*2] = testOutput2[(kk+i)*2] * ww[(kk+i)*2] + testOutput2[(kk+i)*2 + 1] * ww[(kk+i)*2 + 1];
    testInput[i*2 + 1] = testOutput2[(kk+i)*2 + 1] * ww[(kk+i)*2] - testOutput2[(kk+i)*2] * ww[(kk+i)*2 + 1];
  }

  // for (i = 0; i < m*2; i++)
  // {
  //   Serial.printf("%f\t", testInput[i]);
  // }
  // Serial.printf("\noutput\n");

  arm_cmplx_mag_f32(testInput, testInput, m); //input buffer, real output buffer, number of complex samples in the input vector
  // the output is in testInput
  if (sshow == 1) {
    for (i = 0; i < m; i++)
    {
      Serial3.printf("%f\t", testInput[i]);
    }
    Serial3.printf("\noutputM\n");
  }
}

static void apply_window1(float32_t *buf, const float32_t *win)
{
  for (int i=0; i < 100; i++) {
    *buf = (*buf) * *win++;
    buf += 2;
  }
}
