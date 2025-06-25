#include <stdio.h>
#include <string.h>

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

#define STD_MB_PORT 502
#define MB_SLAVE_ADDR 17

esp_netif_t *netif = NULL;

static const char *TAG = "DEVICE";

char device_ip_addr_str[16];

typedef enum dMode
{
    dMode_MODBUS,
    dMode_PROV,
    dMode_STA
} deviceMode;

deviceMode Mode;

void modbus_tcp_slave_init()
{
    ESP_LOGI(TAG, "netif pointer: %p", netif);

    void *mb_slave_handler = NULL;

    static uint16_t holding_reg[1] = {0}, input_reg[2] = {0, 1};

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

    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = 0;
    reg_area.address = (void *)&input_reg;
    reg_area.size = sizeof(input_reg);
    reg_area.access = MB_ACCESS_RW;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mb_slave_handler, reg_area));
    // modbus_slave_init();
    ESP_ERROR_CHECK(mbc_slave_start(mb_slave_handler));
    ESP_LOGI(TAG, "mb_slave_handler: %p", mb_slave_handler);

    ESP_LOGI(TAG, "Dispositivo modbus slave iniciado em %s:%d\n", device_ip_addr_str, STD_MB_PORT);
    Mode = dMode_MODBUS;

    while (true)
    {
        (void)mbc_slave_lock(mb_slave_handler);
        holding_reg[0] = 235;
        (void)mbc_slave_unlock(mb_slave_handler);

        /*esp_log_level_set("MB_TCP_SLAVE", ESP_LOG_DEBUG);
        esp_log_level_set("MB_PORT_COMMON", ESP_LOG_DEBUG);
        esp_log_level_set("MB_CONTROLLER_SLAVE", ESP_LOG_DEBUG);*/
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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
        Mode = dMode_STA;
        // netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void on_prov_end(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Provisioning concluído! Trocando para WiFi para modo STA...\n");
    ESP_LOGI(TAG, "IP do dispositivo: %s", device_ip_addr_str);

    wifi_prov_mgr_deinit();
    Mode = dMode_STA;
    //  Iniciar Modbus TCP ou outras lógicas pós-conexão
    //modbus_tcp_slave_init();
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

    // Configuração do AP
    /*wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32-SOFT-AP-00",
            .password = "12345678",
            .ssid_len = strlen("ESP32-SOFT-AP-00"),
            .channel = 6,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 4,
            .pmf_cfg = {
                .required = false,
                .capable = true}}};

    //  Aplica a configuração
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));*/

    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = WIFI_PROV_EVENT_HANDLER_NONE};

    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    const char *ap_service_name = "ESP32-SOFT-AP-00";
    const char *ap_service_key = "12345678";
    // const char *ap_pop = "12345678";

    bool provisioned = false;

    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    Mode = dMode_PROV;

    if (!provisioned)
    {
        ESP_LOGI(TAG, "Iniciando provisioning...");
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_0, NULL, ap_service_name, ap_service_key));
    }
    else
    {
        ESP_LOGI(TAG, "Dispositivo já provisionado. Conectando à rede wireless...");

        vTaskDelay(pdMS_TO_TICKS(10000));
        Mode = dMode_STA;
        //modbus_tcp_slave_init();
        // ESP_ERROR_CHECK(esp_wifi_connect());
    }
}

/*static esp_err_t mbc_slave_callback(mb_event_group_t event, void *arg)
{
    ESP_LOGI(TAG, "Eventos modbus: %d", event);
    return ESP_OK;
}*/

void app_main()
{
    // nvs_flash_erase();
    //   Inicia o provisionamento
    start_wifi_prov();
    if (Mode == dMode_STA)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        modbus_tcp_slave_init();
    }
}