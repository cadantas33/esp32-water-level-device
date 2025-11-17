#include "hydrosensor.h"
#include "driver/adc.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#define ADC_RES 4095 // Resolução do ADC do ESP32
#define MAX_KPA 100000.0f
#define VREF_MAX 3.3f
#define VREF_MIN 0.66f
#define ENV_GRAVITY 9806.65f

int adc_channel; // Variável inteira que identifica o canal ADC

float pressure_kpa; // Variável para valor de pressão lida do sensor, em kPa

const char *TAG = "Hydrostatic Sensor";

static adc_cali_handle_t cal_handle = NULL;
// Inicialização do sensor, com base no canal definido
void hydrosensor_init(int channel)
{
    adc_cali_handle_t handle = NULL;
    adc_cali_line_fitting_config_t cal_cfg = {
        .unit_id = 1,
        .atten = 11,
        .bitwidth = ADC_BITWIDTH_DEFAULT};

    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cal_cfg, &handle));
    ESP_LOGI(TAG, "Calibração do ADC: Linear");

    adc_channel = channel;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);

    ESP_LOGI(TAG, "Sensor inicializado no canal %i", channel);
}

// Leitura da pressão lida pelo sensor
float hydrosensor_read_pressure(void)
{
    int voltage_mv = 0;
    int adc_raw = adc1_get_raw(adc_channel);

    adc_cali_raw_to_voltage(cal_handle, adc_raw, &voltage_mv);

    pressure_kpa = (((voltage_mv / 1000) - VREF_MIN) / (VREF_MAX - VREF_MIN)) * MAX_KPA;

    return pressure_kpa;
}

// Leitura direta da coluna d'água
float hydrosensor_read_height(void)
{
    // A altura h em metros = (pressão * 1000) / densidade do fluido * gravidade ou aceleração
    // Nesse caso, h = (pressão * 1000) / 1000 * 9.81
    float m_height = pressure_kpa / ENV_GRAVITY;

    return m_height;
}