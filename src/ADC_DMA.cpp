#include "ADC_DMA.h"
int16_t ADC_Calibrattion_Val = 0;

#define ADC_filter_n_pow 8        // Sliding window filter length (power of 2)
constexpr int mypow(int a, int b) // Define a function for simple power calculation
{
    int x = 1;
    while (b--)
        x *= a;
    return x;
}
constexpr const int ADC_filter_n = mypow(2, ADC_filter_n_pow); // Sliding window filter length n
// Note: After sliding filter, signal frequency becomes f/(2n), f is about 8000, n=256, so filtered becomes 15Hz
uint16_t ADC_data[ADC_filter_n][8];
float ADC_V[8];

void ADC_DMA_init()
{
    // Set IO mode
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    // Initialize DMA
    {
        DMA_InitTypeDef DMA_InitStructure;

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        DMA_DeInit(DMA1_Channel1);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_data; // Array name represents first element address
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = ADC_filter_n * 8;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel1, &DMA_InitStructure);

        DMA_Cmd(DMA1_Channel1, ENABLE); // Enable DMA
    }

    // Initialize ADC
    {
        ADC_DeInit(ADC1);
        RCC_ADCCLKConfig(RCC_PCLK2_Div8);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        ADC_InitTypeDef ADC_InitStructure;
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;
        ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_NbrOfChannel = 8;
        ADC_Init(ADC1, &ADC_InitStructure);

        ADC_Cmd(ADC1, ENABLE);
        ADC_BufferCmd(ADC1, DISABLE); // Disable buffer

        ADC_ResetCalibration(ADC1); // Reset ADC calibration
        while (ADC_GetResetCalibrationStatus(ADC1))
            ;
        ADC_StartCalibration(ADC1); // Start ADC calibration
        while (ADC_GetCalibrationStatus(ADC1))
            ;
        ADC_Calibrattion_Val = Get_CalibrationValue(ADC1); // Save ADC calibration value
        for (int i = 0; i < 8; i++)
            ADC_RegularChannelConfig(ADC1, i, i + 1, ADC_SampleTime_239Cycles5); // Set 8 channels as regular channels, ~72KHz per channel, 8K total rate
        ADC_DMACmd(ADC1, ENABLE);                                                // Enable ADC DMA mode
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);                                  // Start ADC conversion
    }

    delay(ADC_filter_n); // Wait for buffer to fill 8 times (can convert 8 data sets per ms)
}

float *ADC_DMA_get_value()
{
    for (int i = 0; i < 8; i++)
    {
        int data_sum = 0;
        for (int j = 0; j < ADC_filter_n; j++)
        {
            uint16_t val = ADC_data[j][i]; // Data from channel i on j-th sample
            int sum = val + ADC_Calibrattion_Val;
            if (sum < 0 || val == 0)
                ; // data_sum += 0;
            else if (sum > 4095 || val == 4095)
                data_sum += 4095;
            else
                data_sum += sum;
        }
        data_sum >>= ADC_filter_n_pow;             // Calculate average
        ADC_V[i] = ((float)data_sum) / 4096 * 3.3; // Get voltage value
    }

    return ADC_V;
}