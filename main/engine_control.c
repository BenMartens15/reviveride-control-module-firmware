
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

#define STARTER_EN_PIN 47
#define IGNITION1_EN_PIN 1
#define ASSESSORY1_EN_PIN 38

#define RELAY_GPIO_MASK ((1ULL << STARTER_EN_PIN) | \
                        (1ULL << IGNITION1_EN_PIN) | \
                        (1ULL << ASSESSORY1_EN_PIN))
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

    gpio_set_level(STARTER_EN_PIN, 0);
    gpio_set_level(IGNITION1_EN_PIN, 0);
    gpio_set_level(ASSESSORY1_EN_PIN, 0);

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

    gpio_set_level(IGNITION1_EN_PIN, 1); // turn ignition switch on
    gpio_set_level(ASSESSORY1_EN_PIN, 1); // turn accessory switch on
    vTaskDelay(pdMS_TO_TICKS(500));

    // turn on starter switch for 2 seconds, then back off
    gpio_set_level(STARTER_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(STARTER_EN_PIN, 0);
}

void engine_control_stop_engine(void)
{
    ESP_LOGI(TAG, "Stopping engine");

    m_engine_state = OFF;

    gpio_set_level(IGNITION1_EN_PIN, 0); // turn ignition switch off
    gpio_set_level(ASSESSORY1_EN_PIN, 0); // turn accessory switch off
}
/******************************************************************************/ 

/* PRIVATE FUNCTIONS **********************************************************/
/******************************************************************************/
