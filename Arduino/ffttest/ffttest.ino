#include "AudioStream.h"
#include "arm_math.h"

#define TEST_LENGTH_SAMPLES 2048

static float32_t testOutput[TEST_LENGTH_SAMPLES/2]; 
static float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];

q15_t testOutputQ15[TEST_LENGTH_SAMPLES/2];
q15_t testInput_radix4_q15_50hz[TEST_LENGTH_SAMPLES];

uint32_t fftSize = 1024; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1;

void setup() {
    Serial.begin(9600);
    Serial.println("Starting");
    delay(1000);
}

void loop() {
  Serial.println("Starting");
  arm_cfft_f32_app();
}


void arm_cfft_radix4_q15_app(void)
{
  uint16_t i,j;
  arm_cfft_radix4_instance_q15 S;
 
  fftSize = 1024; 
  ifftFlag = 0; 
  doBitReverse = 1; 
 
 
  arm_cfft_radix4_init_q15(&S, fftSize, ifftFlag, doBitReverse);
 
 
  for(i=0; i<1024; i++)
  {
    testInput_radix4_q15_50hz[i*2+1] = 0;
    j = i % 20;
    testInput_radix4_q15_50hz[i*2] = arm_sin_q15(1638*j);
    Serial.printf("%d\t", testInput_radix4_q15_50hz[i*2]);
  }
 
  Serial.printf("\r\n");
  Serial.printf("**************************************************\r\n");
  Serial.printf("**************************************************\r\n");
 
 
  arm_cfft_radix4_q15(&S, testInput_radix4_q15_50hz);
  arm_cmplx_mag_q15(testInput_radix4_q15_50hz, testOutputQ15, fftSize);
 
 
  for(i=0; i<1024; i++)
  {
    Serial.printf("%d\t", testOutputQ15[i]);
  }
 
}
 
 
void arm_cfft_f32_app(void)
{
  uint16_t i;
  arm_cfft_radix4_instance_f32 S;

  arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse);
   
  for(i=0; i<1024; i++)
  {
    testInput_f32_10khz[i*2+1] = 0;
    testInput_f32_10khz[i*2] = arm_sin_f32(2*3.1415926f*50*i/1000);
    Serial.printf("%f\t", testInput_f32_10khz[i*2]);
  }

   Serial.printf("\n*********\n");
   
  arm_cfft_radix4_f32(&S, testInput_f32_10khz);
  arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);
   
   
  for(i=0; i<1024; i ++)
  {
    Serial.printf("%f\t", testOutput[i]);
  }
  Serial.printf("\nOutput Above*********\n");
 
}
