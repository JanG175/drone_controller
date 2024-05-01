#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_crc.h"
#include "esp_random.h"

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

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

#define ESPNOW_QUEUE_SIZE 6

#define ESPNOW_SEND_COUNT 1000 // total count of unicast esp now data to be sent (1 - 65535)
#define ESPNOW_SEND_DELAY 0 // delay between sending two esp now data [ms]
#define ESPNOW_SEND_LEN   30 // length of esp now data to be sent [byte]

typedef enum
{
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t* data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union
{
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

// when esp now sending or receiving callback function is called, post event to esp now task
typedef struct
{
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

// parameters of sending esp now data
typedef struct
{
    bool unicast;                         // send unicast esp now data
    bool broadcast;                       // send broadcast esp now data
    uint8_t state;                        // indicate that if has received broadcast esp now data or not
    uint32_t magic;                       // magic number which is used to determine which device to send unicast esp now data
    uint16_t count;                       // total count of unicast esp now data to be sent
    uint16_t delay;                       // delay between sending two esp now data [ms]
    int len;                              // length of esp now data to be sent [byte]
    uint8_t* buffer;                      // buffer pointing to esp now data
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // MAC address of destination device
} espnow_send_param_t;

// user defined field of esp now data
typedef struct
{
    uint8_t type;                         // broadcast or unicast esp now data
    uint8_t state;                        // indicate that if has received broadcast esp now data or not
    uint16_t seq_num;                     // sequence number of esp now data
    uint16_t crc;                         // CRC16 value of esp now data
    uint32_t magic;                       // magic number which is used to determine which device to send unicast esp now data

    float payload[5];                     // real payload of esp now data (height, roll, pitch, yaw)
} __attribute__((packed)) espnow_data_t;

typedef struct
{
    float height_m;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    bool armed;
} drone_telemetry_t;

typedef struct
{
    espnow_send_param_t* send_param;
    drone_telemetry_t* drone_telemetry;
} espnow_task_param_t;


static QueueHandle_t espnow_queue;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t espnow_seq[ESPNOW_DATA_MAX] = {0, 0};

static espnow_send_param_t* send_param;
static drone_telemetry_t* drone_telem;

static const char* TAG = "drone_controller";


/**
 * @brief read user input from UART
 * 
 * @param espnow_data pointer to esp now data struct
 * @param drone_telem pointer to drone telemetry struct
 */
static void read_uart_input(espnow_data_t* espnow_data, drone_telemetry_t* drone_telem)
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
                drone_telem->armed = false;
                break;
            case 'n':
                drone_telem->armed = true;
                break;
            default:
                break;
        }

        espnow_data->payload[0] = drone_telem->height_m;
        espnow_data->payload[1] = drone_telem->roll_deg;
        espnow_data->payload[2] = drone_telem->pitch_deg;
        espnow_data->payload[3] = drone_telem->yaw_deg;
        espnow_data->payload[4] = (float)drone_telem->armed;
    }
}


/**
 * @brief initialize esp now wifi
 * 
*/
static void esp_now_wifi_init()
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
 * @brief callback function when esp now data is sent
 * 
 * @param mac_addr MAC address of destination device
 * @param status status of sending esp now data
*/
static void espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t* send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
        ESP_LOGW(TAG, "Send send queue fail");
}


/**
 * @brief callback function when esp now data is received
 *
 * @param mac_addr MAC address of source device
 * @param data pointer to esp now data
 * @param len length of esp now data
*/
static void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t* recv_cb = &evt.info.recv_cb;
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
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}


/**
 * @brief deinitialize esp now
 * 
 * @param send_param pointer to sending parameters
 * @param drone_telem pointer to drone telemetry
*/
static void espnow_deinit(espnow_send_param_t* send_param, drone_telemetry_t* drone_telem)
{
    free(drone_telem);
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();

    drone_telem = NULL;
    send_param = NULL;

    esp_restart();
}


/**
 * @brief parse received esp now data
 * 
 * @param data pointer to esp now data
 * @param data_len length of esp now data
 * @param state pointer to state of received esp now data
 * @param seq pointer to sequence number of received esp now data
 * @param magic pointer to magic number of received esp now data
*/
uint8_t espnow_data_parse(uint8_t* data, uint16_t data_len, uint8_t* state, uint16_t* seq, uint32_t* magic)
{
    espnow_data_t* buf = (espnow_data_t*)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t))
    {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len: %d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const*)buf, data_len);

    if (crc_cal == crc)
    {
        for (uint32_t i = 0; i < 5; i++)
            ESP_LOGI(TAG, "payload[%lu]: %f", i, buf->payload[i]);
        ESP_LOGI(TAG, "====================================");

        return buf->type;
    }

    return -1;
}


/**
 * @brief prepare esp now data to be sent
 * 
 * @param send_param pointer to sending parameters
 * @param drone_telem pointer to drone telemetry
*/
void espnow_data_prepare(espnow_send_param_t* send_param, drone_telemetry_t* drone_telem)
{
    espnow_data_t* buf = (espnow_data_t*)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    // fill payload with telemetry data
    read_uart_input(buf, drone_telem);

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    // fill all remaining bytes after the data with random values
    esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const*)buf, send_param->len);
}


/**
 * @brief esp now task
 * 
 * @param pvParameter pointer to task parameters
*/
static void espnow_task(void* pvParameter)
{
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    // start sending broadcast ESPNOW data
    espnow_task_param_t* param = (espnow_task_param_t*)pvParameter;
    espnow_send_param_t* send_param = param->send_param;
    drone_telemetry_t* drone_telem = param->drone_telemetry;

    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error");
        espnow_deinit(send_param, drone_telem);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t* send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false))
                    break;

                if (is_broadcast)
                {
                    send_param->count--;
                    if (send_param->count == 0)
                    {
                        ESP_LOGI(TAG, "Send done");
                        espnow_deinit(send_param, drone_telem);
                        vTaskDelete(NULL);
                    }
                }

                // delay a while before sending the next data
                if (send_param->delay > 0)
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                espnow_data_prepare(send_param, drone_telem);

                // send the next data after the previous data is sent
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error");
                    espnow_deinit(send_param, drone_telem);
                    vTaskDelete(NULL);
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t* recv_cb = &evt.info.recv_cb;

                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);

                if (ret == ESPNOW_DATA_BROADCAST)
                {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    // if MAC address does not exist in peer list, add it to peer list
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                    {
                        esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL)
                        {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            espnow_deinit(send_param, drone_telem);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK(esp_now_add_peer(peer));
                        free(peer);
                    }

                    // indicates that the device has received broadcast ESPNOW data
                    if (send_param->state == 0)
                        send_param->state = 1;

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1)
                    {
                        // the device which has the bigger magic number sends ESPNOW data, the other one receives esp now data
                        if (send_param->unicast == false && send_param->magic >= recv_magic)
                        {
                            ESP_LOGI(TAG, "Start sending unicast data");
                            ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                            // start sending unicast ESPNOW data
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            espnow_data_prepare(send_param, drone_telem);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Send error");
                                espnow_deinit(send_param, drone_telem);
                                vTaskDelete(NULL);
                            }
                            else
                            {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == ESPNOW_DATA_UNICAST)
                {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    // if receive unicast esp now data, also stop sending broadcast esp now data
                    send_param->broadcast = false;
                }
                else
                {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}


/**
 * @brief initialize esp now
 *
 * @return ESP_OK on success, ESP_FAIL on error
*/
static esp_err_t espnow_init()
{
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    esp_now_wifi_init();
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

// LOW POWER MODE
#ifdef ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(ESPNOW_WAKE_INTERVAL));
#endif

    // set primary master key
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESPNOW_PMK));

    // add broadcast peer information to peer list
    esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    // initialize sending parameters
    drone_telem = malloc(sizeof(drone_telemetry_t));
    drone_telem->height_m = 0.0f;
    drone_telem->roll_deg = 0.0f;
    drone_telem->pitch_deg = 0.0f;
    drone_telem->yaw_deg = 0.0f;
    drone_telem->armed = false;

    send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random() + 1; // sending device
    send_param->count = ESPNOW_SEND_COUNT;
    send_param->delay = ESPNOW_SEND_DELAY;
    send_param->len = ESPNOW_SEND_LEN;
    send_param->buffer = malloc(ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param, drone_telem);

    espnow_task_param_t task_param = {
        .send_param = send_param,
        .drone_telemetry = drone_telem
    };

    xTaskCreate(espnow_task, "espnow_task", 4096, &task_param, 4, NULL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    return ESP_OK;
}


/**
 * @brief application entry point
 *
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

    // init esp now
    ESP_ERROR_CHECK(espnow_init());
}