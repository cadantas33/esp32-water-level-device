#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"

#include "mbcontroller.h"
#include "hydrosensor.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

// I2C Padrão
#define I2C_SDA 21
#define I2C_SCL 22
// Prioridade de tasks
#define PROV_TSK_PRIORITY 2
#define MB_TSK_PRIORITY 1
#define DISP_TASK_PRIORITY 3
#define STD_MB_PORT 502
#define MB_SLAVE_ADDR 17

static const char *TAG = "DEVICE"; // Nome que vai aparecer nos LOGs do dispositivo

// Buffers para armazenar dados da rede em string/texto
char net_ssid_buff[64], device_ip_buff[128], device_ip_addr_str[16];

esp_netif_t *netif = NULL; // Ponteiro da interface de rede

typedef enum deviceMode // Modos do dispositivo
{
    dMode_MODBUS,
    dMode_PROV,
    dMode_STA
} dMode;
dMode deviceMode;

// Tarefa/função do display OLED
void ssd1306_display_service()
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    // Configuração de endereço e canal i2c para o display
    SSD1306_t disp_cfg = {
        ._address = 0x3C};

    // Inicializa o display com as configurações
    ssd1306_init(&disp_cfg, 128, 64);
    // Limpa a tela
    ssd1306_clear_screen(&disp_cfg, false);
    // Configura contraste
    ssd1306_contrast(&disp_cfg, 0xff);

    for (;;)
    {
        switch (deviceMode)
        {
        case dMode_PROV:
            ssd1306_display_text(&disp_cfg, 0, "Provisionamento iniciou!", 16, false);
            ssd1306_display_text(&disp_cfg, 1, "ESP32-PROVISION", 16, false);
            ssd1306_display_text(&disp_cfg, 2, "12345678", 16, false);

            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        case dMode_STA:
            ssd1306_display_text(&disp_cfg, 0, "CONECTADO EM:", 16, false);
            ssd1306_display_text(&disp_cfg, 2, net_ssid_buff, 16, false);

            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        case dMode_MODBUS:
            ssd1306_display_text(&disp_cfg, 0, "Modbus Iniciado em:", 16, false);
            ssd1306_display_text(&disp_cfg, 2, device_ip_buff, 16, false);
            ssd1306_display_text(&disp_cfg, 3, "PORTA: 502", 16, false);

            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        default:
            ssd1306_display_text(&disp_cfg, 4, "INICIANDO...", 16, false);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// Funçao do modbus
void modbus_tcp_slave_init(void *pvParams)
{

    ESP_LOGI(TAG, "netif pointer: %p", netif);

    void *mb_slave_handler = NULL;

    static uint16_t holding_reg[1] = {0}, discr_in[2] = {0, 1};

    mb_communication_info_t tcp_cfg = {
        .tcp_opts.mode = MB_TCP,
        .tcp_opts.port = STD_MB_PORT,
        .tcp_opts.addr_type = MB_IPV4,
        .tcp_opts.uid = MB_SLAVE_ADDR,
        .tcp_opts.ip_addr_table = NULL,
        .tcp_opts.ip_netif_ptr = (void *)esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")};

    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_err_t err = mbc_slave_create_tcp(&tcp_cfg, &mb_slave_handler);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Conexão modbus tcp falhou: 0x%x", err);
        return;
    }
    else
    {
        ESP_LOGI(TAG, "mbc_slave_create_tcp OK!");
    }

    mb_register_area_descriptor_t reg_area = {0};

    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = 0;
    reg_area.address = (void *)&holding_reg;
    reg_area.size = sizeof(holding_reg);
    reg_area.access = MB_ACCESS_RW;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mb_slave_handler, reg_area));

    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = 0;
    reg_area.address = (void *)&discr_in;
    reg_area.size = sizeof(discr_in);
    reg_area.access = MB_ACCESS_RW;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mb_slave_handler, reg_area));
    // modbus_slave_init();
    while (deviceMode != dMode_STA)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_ERROR_CHECK(mbc_slave_start(mb_slave_handler));
    ESP_LOGI(TAG, "mb_slave_handler: %p", mb_slave_handler);

    ESP_LOGI(TAG, "Dispositivo modbus slave iniciado em %s:%d\n", device_ip_addr_str, STD_MB_PORT);
    deviceMode = dMode_MODBUS;

    for (;;)
    {
        hydrosensor_read_pressure();
        float analog = hydrosensor_read_height();
        int sw_in_1 = gpio_get_level(GPIO_NUM_2);
        int sw_in_2 = gpio_get_level(GPIO_NUM_4);
        (void)mbc_slave_lock(mb_slave_handler);

        holding_reg[0] = analog; // Leitura do sensor de pressão
        discr_in[0] = sw_in_1;   // Leitura das boias
        discr_in[1] = sw_in_2;
        (void)mbc_slave_unlock(mb_slave_handler);

        // Leitura do botao de reset do provisionamento
        if (gpio_get_level(GPIO_NUM_18) == 0) // Quando é pressionado, retorna LOW (pull-up)
        {
            ESP_LOGI(TAG, "Dispositivo reiniciando...\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
            mbc_slave_stop(mb_slave_handler); // Modbus slave é parado quando o botão é pressionado
            esp_wifi_stop();                  // Para o sistema de WiFi
            nvs_flash_deinit();               // Desativa armazenamento não volátil
            nvs_flash_erase();                // Apaga armazenamento
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart(); // Performa um reinício via software
        }
        /*esp_log_level_set("MB_TCP_SLAVE", ESP_LOG_DEBUG);
        esp_log_level_set("MB_PORT_COMMON", ESP_LOG_DEBUG);
        esp_log_level_set("MB_CONTROLLER_SLAVE", ESP_LOG_DEBUG);*/
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG, "Conectado com sucesso!");
        ESP_LOGI(TAG, "IP obtido: " IPSTR, IP2STR(&event->ip_info.ip));

        //  Converte o ip do dispositivo para uma string, após a conexão
        esp_ip4addr_ntoa(&event->ip_info.ip, device_ip_addr_str, sizeof(device_ip_addr_str));

        printf("Trocando para STA...");
        deviceMode = dMode_STA;
        // Passa o ip do dispositivo para um buffer
        snprintf(device_ip_buff, sizeof(device_ip_buff), "%s", device_ip_addr_str);
        ESP_LOGI(TAG, "%s", device_ip_buff);
    }
}

static void on_prov_end(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Provisioning concluído! Trocando para WiFi para modo STA...\n");
    ESP_LOGI(TAG, "IP do dispositivo: %s", device_ip_addr_str);

    wifi_prov_mgr_deinit();
    // deviceMode = dMode_STA;
    //   Iniciar Modbus TCP ou outras lógicas pós-conexão
    //  modbus_tcp_slave_init();
}

void start_wifi_prov()
{
    // Inicialização básica
    // ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, WIFI_PROV_END, &on_prov_end, NULL));

    netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    // Configuração do WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Cria interface AP corretamente
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = WIFI_PROV_EVENT_HANDLER_NONE};

    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    const char *ap_service_name = "ESP32-PROVISION";
    const char *ap_service_key = "12345678";

    bool provisioned = false;

    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    deviceMode = dMode_PROV;

    if (!provisioned)
    {
        ESP_LOGI(TAG, "Iniciando provisioning...");
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_0, NULL, ap_service_name, ap_service_key));
    }
    else
    {
        ESP_LOGI(TAG, "Dispositivo já provisionado. Conectando à rede wireless...");

        vTaskDelay(pdMS_TO_TICKS(10000));
        deviceMode = dMode_STA;

        // Obtém o SSID da rede conectada e passa para um buffer
        wifi_config_t wifi_cfg;
        esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_cfg);
        sprintf(net_ssid_buff, "Rede:\n%s", (char *)wifi_cfg.sta.ssid);

    }
}

void app_main(void)
{
    // Inicializa o sensor de pressão hidrostática
    hydrosensor_init(ADC1_CHANNEL_4);
    // Configura e inicializa as boais digitais
    gpio_config_t sw_cfg = {
        .pin_bit_mask = (1ULL << GPIO_NUM_2) | (1ULL << GPIO_NUM_4) | (1ULL << GPIO_NUM_18),
        .mode = GPIO_MODE_INPUT,               // Configura entradas 18 e 19 para boias
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Pull-up habilitado
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Pull-down desativado pois nao será necessário
        .intr_type = GPIO_INTR_DISABLE         // Interrupções desabilitadas nestes pinos
    };
    gpio_config(&sw_cfg);

    nvs_flash_erase();
    //  Inicia o provisionamento
    start_wifi_prov();
    //  Inicia o display
    xTaskCreate(ssd1306_display_service, "DISPLAY_TASK", 2048 * 2, NULL, DISP_TASK_PRIORITY, NULL);
    //  Inicia o processo modbus
    xTaskCreate(modbus_tcp_slave_init, "MB_SLAVE_TASK", 1024 * 4, NULL, MB_TSK_PRIORITY, NULL);
}