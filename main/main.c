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
// #include "pl_modbus.h"
#include "mbcontroller.h"
#include "ssd1306.h"
#include "hydrosensor.h"

#define I2C_SDA 16
#define I2C_SCL 17
// #define SWITCH_PIN_1 GPIO_NUM_18 // Definição das bóias como pinos digitais 18 e 19
// #define SWITCH_PIN_2 GPIO_NUM_19
// #define SENSOR_CHANNEL ADC1_CHANNEL_0 // Definição do sensor como pino analógico 32
#define PROV_TSK_PRIORITY 2
#define MB_TSK_PRIORITY 1
#define DISP_TASK_PRIORITY 3
#define STD_MB_PORT 502
#define MB_SLAVE_ADDR 17

static const char *TAG = "DEVICE";

char net_ssid_buff[64], device_ip_buff[64], device_ip_addr_str[16];

esp_netif_t *netif = NULL;
typedef enum deviceMode
{
    dMode_MODBUS,
    dMode_PROV,
    dMode_STA
} dMode;
dMode deviceMode;

// Tarefa/função do display OLED
void ssd1306_display_service()
{
    // Define configurações do canal i2c master
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true};
    // Cria e inicializa um handler para o barramento i2c
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    ssd1306_config_t disp_cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT; // Configuração de display i2c 128x64px padrão
    ssd1306_handle_t disp_handler;                                 // Handler padrão

    // Inicialização do display com base na cfg e handler definidos
    ssd1306_init(bus_handle, &disp_cfg, &disp_handler);
    // Verificação do handler
    if (disp_handler == NULL)
    {
        ESP_LOGI(TAG, "Falha na inicialização de handler SSD1306");
        assert(disp_handler);
    }

    for (;;)
    {
        // Exibição de texto no display
        ESP_LOGI(TAG, "Iniciando display...");
        switch (deviceMode)
        {
        case dMode_PROV:
            ssd1306_clear_display(disp_handler, false);
            ssd1306_set_contrast(disp_handler, 0xff);
            ssd1306_display_text(disp_handler, 0, "INICIANDO PROV", false);
            ssd1306_display_text(disp_handler, 1, "REDE: ESP32-SOFT-AP\nSENHA: 12345678", false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        case dMode_STA:
            ssd1306_clear_display(disp_handler, false);
            ssd1306_set_contrast(disp_handler, 0xff);
            ssd1306_display_text(disp_handler, 0, "CONECTADO AO WI-FI", false);
            ssd1306_display_text(disp_handler, 1, net_ssid_buff, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        case dMode_MODBUS:
            ssd1306_clear_display(disp_handler, false);
            ssd1306_set_contrast(disp_handler, 0xff);
            ssd1306_display_text(disp_handler, 0, device_ip_buff, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        default:
            ssd1306_clear_display(disp_handler, false);
            ssd1306_set_contrast(disp_handler, 0xff);
            ssd1306_display_text(disp_handler, 0, "###INICIANDO###", false);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

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
    if (deviceMode == dMode_STA)
    {
        ESP_ERROR_CHECK(mbc_slave_start(mb_slave_handler));
        ESP_LOGI(TAG, "mb_slave_handler: %p", mb_slave_handler);

        ESP_LOGI(TAG, "Dispositivo modbus slave iniciado em %s:%d\n", device_ip_addr_str, STD_MB_PORT);
        deviceMode = dMode_MODBUS;
    }

    for (;;)
    {

        (void)mbc_slave_lock(mb_slave_handler);
        hydrosensor_read_pressure();
        holding_reg[0] = hydrosensor_read_height(); // Leitura do sensor de pressão
        discr_in[0] = gpio_get_level(GPIO_NUM_18);  // Leitura das boias
        discr_in[1] = gpio_get_level(GPIO_NUM_19);
        (void)mbc_slave_unlock(mb_slave_handler);

        if (gpio_get_level(GPIO_NUM_21) == 1)
        {                                     // Leitura do botao de reset do provisionamento
            nvs_flash_erase();                // Memmoria nao volatil (nvs) é apagada quando o botão é pressionado
            //mbc_slave_stop(mb_slave_handler); // Modbus slave é parado quando o botão é pressionado
            esp_restart();                    // Performa um reinício via software
        }
        /*esp_log_level_set("MB_TCP_SLAVE", ESP_LOG_DEBUG);
        esp_log_level_set("MB_PORT_COMMON", ESP_LOG_DEBUG);
        esp_log_level_set("MB_CONTROLLER_SLAVE", ESP_LOG_DEBUG);*/
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*void modbus_service()
{
    if (deviceMode == dMode_STA)
    {
        modbus_tcp_slave_init();
    }
}*/

/*extern "C"  void modbus_tcp_slave_init()
{
    PL::ModbusServer mb_server(STD_MB_PORT);

    PL::TcpServer& mb_tcp_server = (PL::TcpServer&)*(mb_server.GetBaseServer().lock());

    auto holdingRegisters = std::make_shared<PL::ModbusMemoryArea>(PL::ModbusMemoryType::holdingRegisters, 0, 10);
} */

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

        // netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void on_prov_end(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Provisioning concluído! Trocando para WiFi para modo STA...\n");
    ESP_LOGI(TAG, "IP do dispositivo: %s", device_ip_addr_str);

    wifi_prov_mgr_deinit();
    deviceMode = dMode_STA;
    //  Iniciar Modbus TCP ou outras lógicas pós-conexão
    // modbus_tcp_slave_init();
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

    const char *ap_service_name = "ESP32-SOFT-AP";
    const char *ap_service_key = "12345678";
    // const char *ap_pop = "12345678";

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
        // Passa o ip do dispositivo para um buffer
        sprintf(device_ip_buff, "IP do dispositivo:\n %s", device_ip_addr_str);
        // printf("Provisionado, Trocando para STA...");
        //  modbus_tcp_slave_init();
        //   ESP_ERROR_CHECK(esp_wifi_connect());
    }
}

/*static esp_err_t mbc_slave_callback(mb_event_group_t event, void *arg)
{
    ESP_LOGI(TAG, "Eventos modbus: %d", event);
    return ESP_OK;
}*/

void app_main(void)
{
    // Inicializa o sensor de pressão hidrostática
    hydrosensor_init(ADC1_CHANNEL_4);
    // Configura e inicializa as boais digitais
    gpio_config_t sw_cfg = {
        .pin_bit_mask = (1ULL << GPIO_NUM_18) | (1ULL << GPIO_NUM_19), // Pinos 18 e 19 como conexão de entrada das boias
        .mode = GPIO_MODE_INPUT,                                       // Configura como pinos de entrada
        .pull_up_en = GPIO_PULLUP_DISABLE,                             // Pull-up desabilitado
        .pull_down_en = GPIO_PULLDOWN_ENABLE,                          // Pull-down ativado pois será necessário
        .intr_type = GPIO_INTR_DISABLE                                 // Interrupções desabilitadas nestes pinos
    };
    gpio_config(&sw_cfg);

    // Configura e inicializa botao de reset do provisioning
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << GPIO_NUM_21), // Pino 20 como conexão de entrada do botao
        .mode = GPIO_MODE_INPUT,               // Configura como entrada
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Pull-up habilitado
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Pull-down desativado
        .intr_type = GPIO_INTR_DISABLE         // Interrupções desabilitadas
    };
    gpio_config(&rst_cfg);
    //  Inicia o display
    // xTaskCreate(ssd1306_display_service, "DISPLAY_TASK", 2048, NULL, DISP_TASK_PRIORITY, NULL);
    //  Inicia o provisionamento
    nvs_flash_erase();
    start_wifi_prov();
    // xTaskCreate(start_wifi_prov, "PROV_INIT", 1024 * 4, NULL, PROV_TSK_PRIORITY, NULL);
    //  Inicia o processo modbus
    xTaskCreate(modbus_tcp_slave_init, "MB_SLAVE_TASK", 1024 * 5, NULL, MB_TSK_PRIORITY, NULL);
}
