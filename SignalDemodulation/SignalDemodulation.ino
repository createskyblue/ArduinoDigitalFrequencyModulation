#include "arduinoFFT.h"
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

#define CHANNEL 32
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 8000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}

void loop()
{
  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  Decode(vReal, samplingFrequency, samples);

//   Serial.println("Computed magnitudes:");
//   PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

#define setbit(x,y)       x|=(1<<y)
#define clrbit(x,y)       x&=~(1<<y)
#define reversebit(x,y)   x^=(1<<y)
#define getbit(x,y)       ((x) >> (y)&1)

#define CLK_FREQ 2500
#define O_FREQ   2000
#define I_FREQ   3000

typedef struct DIGITALFM
{
    uint16_t freq;
    uint8_t state;
    char Describe[10];
}DigitalFm;

enum FreqForm_Name{
    Freq_O = 0,
    Freq_CLK,
    Freq_I,
    Freq_EOF,
};

DigitalFm FreqForm[4] = {
       {1200,0,"Freq_O"},
       {1500,0,"Freq_CLK"},
       {1900,0,"Freq_I"},
       {2300,0,"Freq_EOF"},
       
};

void Decode(double vData[], uint32_t samplingFrequency, uint32_t size) {
    static uint8_t reciveFlag = 0;
    static uint8_t receDataSize = 0;
    static uint8_t data = 0;
    static uint32_t receTimer = 0;
    //Serial.println();
    for (uint8_t i=0;i<4;i++) {
        if (Get_FREQUENCY(FreqForm[i].freq, vData, samplingFrequency, size) > 500) {
            FreqForm[i].state = 1;
            //Serial.print("1");
        }else {
            FreqForm[i].state = 0;
            //Serial.print("0");
        }
    }
    

    if (FreqForm[Freq_CLK].state) {
        //同步信号
        reciveFlag = 1;
    }else {
        if (reciveFlag == 1) {
            if ((FreqForm[Freq_O].state || FreqForm[Freq_I].state) &&
                (FreqForm[Freq_O].state != FreqForm[Freq_I].state)) {
                data <<= 1;
                
                if (FreqForm[Freq_I].state) {
                    data |= 1;
                    //Serial.print("1");
                }//else Serial.print("0");

                receDataSize++;
                reciveFlag = 2;
            }
        }else{
            if (!FreqForm[Freq_CLK].state && FreqForm[Freq_EOF].state && reciveFlag == 2 && receDataSize >= 8) {
                //字节EOF信号
                //Serial.print(" ");
                //Serial.print(millis() - receTimer);
                //Serial.print("ms ");
                Serial.print((char)data);
                reciveFlag = 3;
                receDataSize = 0;
                receTimer = millis();
            }
        }
    }
}

double Get_FREQUENCY(uint32_t targetFrequency,double vData[], uint32_t samplingFrequency ,uint32_t size) {
    return vData[(targetFrequency * size ) / samplingFrequency];
}