#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"

#define D_M      0.1f // change of height [m]
#define D_DEG    0.5f // change of angle [deg]

#define ESPNOW_WIFI_MODE WIFI_MODE_STA

#if ESPNOW_WIFI_MODE == WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#elif ESPNOW_WIFI_MODE == WIFI_MODE_AP
#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#endif

#define ESPNOW_CHANNEL           1
#define ESPNOW_ENABLE_LONG_RANGE 1

// #define ESPNOW_ENABLE_POWER_SAVE 1
#ifdef ESPNOW_ENABLE_POWER_SAVE
#define ESPNOW_WAKE_WINDOW   50
#define ESPNOW_WAKE_INTERVAL 100
#endif

#define ESPNOW_PMK "pmk1234567890123"
#define ESPNOW_LMK "lmk1234567890123"

#define ESPNOW_MAXDELAY 512

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t s_self_mac[ESP_NOW_ETH_ALEN];

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)
#define IS_SELF_ADDR(addr) (memcmp(addr, s_self_mac, ESP_NOW_ETH_ALEN) == 0)

static QueueHandle_t queue = NULL;

static const char* TAG = "drone_controller";

typedef struct
{
    float height_m;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    bool arm_state;
    bool keep_alive;
    uint8_t mac[ESP_NOW_ETH_ALEN];
} drone_telemetry_t;


/**
 * @brief read input from UART keyboard and update drone telemetry
 * 
 * @param drone_telem pointer to drone telemetry
 */
static void read_uart_input(drone_telemetry_t* drone_telem)
{
    uint8_t data;
    uint32_t data_len = 1;

    // read from UART
    uint32_t buf = uart_read_bytes(CONFIG_CONSOLE_UART_NUM, &data, data_len, 50 / portTICK_PERIOD_MS);

    if (buf == data_len)
    {
        switch (data)
        {
            case 'w':
                drone_telem->pitch_deg += D_DEG;
                break;
            case 's':
                drone_telem->pitch_deg -= D_DEG;
                break;
            case 'a':
                drone_telem->roll_deg -= D_DEG;
                break;
            case 'd':
                drone_telem->roll_deg += D_DEG;
                break;
            case 'q':
                drone_telem->yaw_deg += D_DEG;
                break;
            case 'e':
                drone_telem->yaw_deg -= D_DEG;
                break;
            case 'p':
                drone_telem->height_m += D_M;
                break;
            case 'l':
                drone_telem->height_m -= D_M;
                break;
            case 'm':
                drone_telem->arm_state = false;
                break;
            case 'n':
                drone_telem->arm_state = true;
                break;
            default:
                break;
        }
    }
    else // slowly go back to zero
    {
        if (drone_telem->roll_deg > 0.0f)
            drone_telem->roll_deg -= D_DEG;
        else if (drone_telem->roll_deg < 0.0f)
            drone_telem->roll_deg += D_DEG;

        if (drone_telem->pitch_deg > 0.0f)
            drone_telem->pitch_deg -= D_DEG;
        else if (drone_telem->pitch_deg < 0.0f)
            drone_telem->pitch_deg += D_DEG;
    }

    drone_telem->keep_alive = true;

    ESP_LOGI(TAG, "height: %f, roll: %f, pitch: %f, yaw: %f, armed: %d", drone_telem->height_m, drone_telem->roll_deg, drone_telem->pitch_deg, drone_telem->yaw_deg, drone_telem->arm_state);
}


/**
 * @brief initialize WIFI
 */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR));
#endif
}


/**
 * @brief callback function for sending data
 * 
 * @param mac_addr MAC address of the peer
 * @param status status of sending data
 */
static void espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
}


/**
 * @brief callback function for receiving data
 *
 * @param recv_info information about the received data
 * @param data received data
 * @param len length of received data
 */
static void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len)
{
    uint8_t* mac_addr = recv_info->src_addr;
    uint8_t* des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr))
    {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    }
    else
    {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    // copy received data and MAC address to queue
    drone_telemetry_t drone_telem;
    memcpy(&drone_telem, data, sizeof(drone_telemetry_t));
    memcpy(drone_telem.mac, mac_addr, ESP_NOW_ETH_ALEN);

    if (xQueueSend(queue, &drone_telem, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGE(TAG, "Queue with received message send fail");
    }
}


/**
 * @brief task for sending and receiving data
 * 
 * @param arg pointer to drone telemetry
 */
static void espnow_task(void* arg)
{
    drone_telemetry_t drone_telem = *(drone_telemetry_t*)arg;

    // send broadcast until a response from the drone
    while (1)
    {
        ESP_LOGI(TAG, "Waiting for drone to peer...");
        esp_now_send(s_broadcast_mac, (uint8_t*)&drone_telem, sizeof(drone_telemetry_t));

        if (xQueueReceive(queue, &drone_telem, ESPNOW_MAXDELAY) == pdTRUE)
        {
            // check if drone sends back its MAC
            if (esp_now_is_peer_exist(drone_telem.mac) == false && IS_SELF_ADDR(drone_telem.mac) == false)
            {
                ESP_LOGI(TAG, "Drone peer found!");
                ESP_LOGI(TAG, "Drone MAC: %02X:%02X:%02X:%02X:%02X:%02X", drone_telem.mac[0], drone_telem.mac[1], drone_telem.mac[2], drone_telem.mac[3], drone_telem.mac[4], drone_telem.mac[5]);
                break;
            }
        }
    }

    // add drone to peer list
    esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        esp_now_deinit();
        vTaskDelete(NULL);
        return;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = true;
    memcpy(peer->lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
    memcpy(peer->peer_addr, drone_telem.mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));

    // start sending telemetry data
    while (1)
    {
        read_uart_input(&drone_telem);

        esp_now_send(peer->peer_addr, (uint8_t*)&drone_telem, sizeof(drone_telemetry_t));
    }

    free(peer);
    vTaskDelete(NULL);
}


/**
 * @brief initialize ESPNOW
 *
 * @return error status
 */
static esp_err_t espnow_init(void)
{
    // Initialize sending parameters
    drone_telemetry_t drone_telem;
    drone_telem.roll_deg = 0.0f;
    drone_telem.pitch_deg = 0.0f;
    drone_telem.yaw_deg = 0.0f;
    drone_telem.height_m = 0.0f;
    drone_telem.arm_state = false;
    drone_telem.keep_alive = false;

    queue = xQueueCreate(1, sizeof(drone_telem));
    if (queue == NULL)
    {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    // Initialize ESPNOW and register sending and receiving callback function
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
#if ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(ESPNOW_WAKE_INTERVAL));
#endif
    // Set primary master key
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESPNOW_PMK));

    // Add broadcast peer information to peer list
    esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    // get self MAC address
    esp_base_mac_addr_get(s_self_mac);

    xTaskCreate(espnow_task, "espnow_task", 4096, &drone_telem, 4, NULL);

    return ESP_OK;
}


/**
 * @brief main function
 */
void app_main(void)
{
    // init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // init WIFI
    wifi_init();

    // init UART
    uart_config_t uart_config = {
        .baud_rate = CONFIG_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(CONFIG_CONSOLE_UART_NUM, &uart_config));
#if CONFIG_CONSOLE_UART_CUSTOM
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_CONSOLE_UART_NUM, CONFIG_CONSOLE_UART_TX_GPIO, CONFIG_CONSOLE_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));

    // init ESPNOW
    ESP_ERROR_CHECK(espnow_init());
}