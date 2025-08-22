#include "hydrosensor.h"
#include "driver/adc.h"
#include "esp_log.h"

#define ADC_RES 4095 // Resolução do ADC do ESP32

int adc_channel; // Variável inteira que identifica o canal ADC

float pressure_kpa; // Variável para valor de pressão lida do sensor, em kPa

const char *TAG = "Hydrostatic Sensor";

// Inicialização do sensor, com base no canal definido
void hydrosensor_init(int channel)
{
    adc_channel = channel;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);

    ESP_LOGI(TAG, "Sensor inicializado no canal %i", channel);
}

// Leitura da pressão lida pelo sensor
float hydrosensor_read_pressure(void)
{
    int adc_val = adc1_get_raw(adc_channel); // Leitura do valor "puro" vindo do sensor

    const float max_press_kpa = 100.0; // Pressão máxima do sensor em

    pressure_kpa = (adc_val / ADC_RES) * max_press_kpa;

    return pressure_kpa;
}

// Leitura direta da "altura" do sensor
float hydrosensor_read_height(void)
{
    // A altura h em metros = (pressão * 1000) / densidade do fluido * gravidade ou aceleração
    // Nesse caso, h = (pressão * 1000) / 1000 * 9.81
    float m_height = (pressure_kpa * 1000.0) / (1000.0 * 9.81);

    return m_height;
}