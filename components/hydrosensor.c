#include "hydrosensor.h"
//#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#define ADC_RES 4095 // Resolução do ADC do ESP32
#define MAX_KPA 100000.0f // Pressão máxima em KPA (x1000)
#define VREF_MAX 3.3f // Tensão de referência máxima
#define VREF_MIN 0.66f // Tensão de referência mínima
#define ENV_ACC_X1000 9806.65f // Aceleração do ambiente multiplicada pela densidade da água (x 1000)

int adc_channel; // Variável inteira que identifica o canal ADC
int adc_raw; // Variável que recebe valor adc "puro"
int voltage; // Variável para armazenar a tensão medida no ADC

float pressure_kpa; // Variável para valor de pressão lida do sensor, em kPa

const char *TAG = "Hydrostatic Sensor";

adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t cal_handle = NULL;
// Inicialização do sensor, com base no canal definido
void hydrosensor_init(int channel)
{
    adc_channel = channel;

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channel, &adc_cfg));
    // adc_cali_handle_t handle = NULL;
    adc_cali_line_fitting_config_t cal_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT};

    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cal_cfg, &cal_handle));
    ESP_LOGI(TAG, "Calibração do ADC: Linear");

    //ESP_ERROR_CHECK(adc_calibration_init(ADC_UNIT_1, adc_channel, ADC_ATTEN_DB_12, &cal_handle));

    ESP_LOGI(TAG, "Sensor inicializado no canal %i", channel);
}

// Leitura da pressão lida pelo sensor
float hydrosensor_read_pressure(void)
{
    // Lê o pino analógico e armazena na variável adc_raw
    adc_oneshot_read(adc1_handle, adc_channel, &adc_raw);
    // int voltage = ((adc_raw * VREF_MAX) / ADC_RES);
    // Converte o valor para tensão calibrada
    adc_cali_raw_to_voltage(cal_handle, adc_raw, &voltage);
    // Algoritmo de conversão de valor adc para pressão
    pressure_kpa = ((voltage - VREF_MIN) / (VREF_MAX - VREF_MIN)) * MAX_KPA;
    // retorna o valor da pressão em kpa, ao fim
    return pressure_kpa;
}

// Leitura direta da coluna d'água
float hydrosensor_read_height(void)
{
    // A altura h em metros = (pressão * 1000) / densidade do fluido * aceleração
    // Nesse caso, h = (pressão * 1000) / 1000 * 9.81
    float m_height = pressure_kpa / ENV_ACC_X1000;

    return m_height;
}