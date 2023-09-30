#include "cmsis.h"

#define FFT_SAMPLES 1024U

// float FFTInBuffer[FFT_SAMPLES];
// float FFTOutBuffer[FFT_SAMPLES];
//arm_rfft_fast_instance_f32 FFTHandler;

// fft init function
// arm_rfft_fast_init_f32(FFTHandler, FFT_SAMPLES);

float Im_TO_Real(float real, float compl )
{
    return sqrtf(real * real + compl *compl );
}

void CalculateFFT(arm_rfft_fast_instance_f32 FFTHandler, float32_t *FFTInBuffer, float32_t *FFTOutBuffer, uint16_t Offset)
{
    arm_rfft_fast_f32(&FFTHandler, FFTInBuffer, FFTOutBuffer, 0);

    int Freqs[FFT_SAMPLES];
    int FreqPoint = 0;

    for (int i = 0; i < FFT_SAMPLES; i++)
    {
        Freqs[FreqPoint] = (Im_TO_Real(FFTOutBuffer[i], FFTOutBuffer[i + 1])) - Offset;

        if (Freqs[FreqPoint] < 0)
        {
            Freqs[FreqPoint] = 0;
        }
        FreqPoint++;
    }
}