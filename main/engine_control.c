
/* INCLUDES *******************************************************************/
#include "driver/gpio.h"
#include "engine_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/******************************************************************************/

/* DEFINES ********************************************************************/
#define TAG "ENGINE_CONTROL"
#define ENGINE_CONTROL_LOG_LEVEL ESP_LOG_INFO

#define START_RELAY_PIN 27
#define IGNITION_RELAY_PIN 26
#define ACCESSORY_RELAY_PIN 25

#define RELAY_GPIO_MASK ((1 << START_RELAY_PIN) | \
                        (1 << IGNITION_RELAY_PIN) | \
                        (1 << ACCESSORY_RELAY_PIN))
/******************************************************************************/

/* ENUMS **********************************************************************/
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
engine_state_e m_engine_state = OFF;
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
/******************************************************************************/

/* PUBLIC FUNCTIONS ***********************************************************/
void engine_control_init(void)
{
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = RELAY_GPIO_MASK,
        .pull_down_en = 0,
        .pull_up_en = 0
    };

    esp_log_level_set(TAG, ENGINE_CONTROL_LOG_LEVEL);

    ESP_LOGI(TAG, "Initializing engine control");
    
    gpio_config(&io_config);

    gpio_set_level(START_RELAY_PIN, 1);
    gpio_set_level(IGNITION_RELAY_PIN, 1);
    gpio_set_level(ACCESSORY_RELAY_PIN, 1);

    ESP_LOGI(TAG, "Engine control initialized");
}

void engine_control_toggle_engine_state(void)
{
    if (m_engine_state == RUNNING) {
        engine_control_stop_engine();
    } else {
        engine_control_start_engine();
    }
}

void engine_control_start_engine(void)
{
    ESP_LOGI(TAG, "Starting engine");

    m_engine_state = RUNNING;

    gpio_set_level(IGNITION_RELAY_PIN, 0); // turn ignition relay on
    gpio_set_level(ACCESSORY_RELAY_PIN, 0); // turn accessory relay on
    vTaskDelay(pdMS_TO_TICKS(500));

    // turn on starter relay for 2 seconds, then back off
    gpio_set_level(START_RELAY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(START_RELAY_PIN, 1);
}

void engine_control_stop_engine(void)
{
    ESP_LOGI(TAG, "Stopping engine");

    m_engine_state = OFF;

    gpio_set_level(IGNITION_RELAY_PIN, 1); // turn ignition relay off
    gpio_set_level(ACCESSORY_RELAY_PIN, 1); // turn accessory relay off
}
/******************************************************************************/ 

/* PRIVATE FUNCTIONS **********************************************************/
/******************************************************************************/
