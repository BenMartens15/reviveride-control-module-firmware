
/* INCLUDES *******************************************************************/
#include <string.h>

#include "engine_control.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now_core.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
/******************************************************************************/

/* DEFINES ********************************************************************/
#define TAG "ESP_NOW_CORE"
#define ESP_NOW_CORE_LOG_LEVEL ESP_LOG_INFO 
/******************************************************************************/

/* ENUMS **********************************************************************/
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
static QueueHandle_t m_receive_queue;
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
static void packet_received_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void process_receive_queue_task(void *p);
/******************************************************************************/

/* PUBLIC FUNCTIONS ***********************************************************/
void esp_now_core_init(void)
{
    esp_log_level_set(TAG, ESP_NOW_CORE_LOG_LEVEL);

    esp_err_t err = ESP_OK;

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_LOGI(TAG, "Initializing ESP-NOW...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(packet_received_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)ESPNOW_PMK));

    ESP_LOGI(TAG, "ESP-NOW initialized");

    m_receive_queue = xQueueCreate(10, sizeof(recv_packet_t));
    assert(m_receive_queue);
    err = xTaskCreate(process_receive_queue_task, "recv_task", 8192, NULL, 4, NULL);
    assert(err == pdPASS);
}
/******************************************************************************/ 

/* PRIVATE FUNCTIONS **********************************************************/
static void packet_received_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    static recv_packet_t recv_packet;
    uint8_t * mac_addr = recv_info->src_addr;

    ESP_LOGI(TAG, "%d bytes incoming from "MACSTR" ", len, MAC2STR(mac_addr));

    memcpy(&recv_packet.sender_mac_addr, mac_addr, sizeof(recv_packet.sender_mac_addr));
    memcpy(&recv_packet.command, data, len);
    if (xQueueSend(m_receive_queue, &recv_packet, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Receive queue full - packet discarded");
        return;
    }
}

static void process_receive_queue_task(void *p)
{
    static recv_packet_t recv_packet;

    ESP_LOGI(TAG, "Waiting for data from key fob");
    for(;;)
    {
        if(xQueueReceive(m_receive_queue, &recv_packet, portMAX_DELAY) == pdTRUE)
        {
            switch(recv_packet.command) {
                case TOGGLE_ENGINE_STATE_COMMAND:
                    ESP_LOGI(TAG, "Command received: TOGGLE_ENGINE_STATE_COMMAND");
                    engine_control_toggle_engine_state();
                    break;
                default:
                    ESP_LOGE(TAG, "Unrecognized command received");
            }
        }
    }
}
/******************************************************************************/
