
/* INCLUDES *******************************************************************/
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "led.h"
/******************************************************************************/

/* DEFINES ********************************************************************/
#define TAG "POWER_MANAGEMENT"
#define POWER_MANAGEMENT_LOG_LEVEL ESP_LOG_INFO

#define ESP_INTR_FLAG_DEFAULT 0

#define HEAD_UNIT_EN_PIN 13
#define BLIND_SPOT_L_EN_PIN 11
#define BLIND_SPOT_R_EN_PIN 9
#define DASH_CAM_EN_PIN 3
#define BACKUP_CAM_EN_PIN 18
#define HEAD_UNIT_SHUTDOWN_PIN 15
#define DASH_CAM_SHUTDOWN_PIN 16

#define OUTPUT_GPIO_MASK ((1ULL << HEAD_UNIT_EN_PIN) | \
                                (1ULL << BLIND_SPOT_L_EN_PIN) | \
                                (1ULL << BLIND_SPOT_R_EN_PIN) | \
                                (1ULL << DASH_CAM_EN_PIN) | \
                                (1ULL << BACKUP_CAM_EN_PIN) | \
                                (1ULL << HEAD_UNIT_SHUTDOWN_PIN) | \
                                (1ULL << DASH_CAM_SHUTDOWN_PIN))

#define ACCESSORY_SENSE_PIN 6
#define ACC_PIN_DEBOUNCE_MS 1000

#define SHUTDOWN_DELAY_MS 10000
/******************************************************************************/

/* ENUMS **********************************************************************/
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
static SemaphoreHandle_t m_acc_semaphore = NULL;
static TimerHandle_t m_shutdown_delay = NULL; // timer for things that need a delayed shutdown like head unit and dash cam
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
static void accessory_change_state_isr(void* arg);
static void accessory_task(void* arg);
static void power_on_system(void);
static void power_off_system(void);
static void delayed_power_off(TimerHandle_t xTimer);
/******************************************************************************/

/* PUBLIC FUNCTIONS ***********************************************************/
void power_management_init(void)
{
    gpio_config_t output_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = OUTPUT_GPIO_MASK,
        .pull_down_en = 0,
        .pull_up_en = 0
    };

    gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << ACCESSORY_SENSE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    esp_log_level_set(TAG, POWER_MANAGEMENT_LOG_LEVEL);

    ESP_LOGI(TAG, "Initializing power management");

    gpio_config(&output_config);

    // disable all load switches on startup (they're active low)
    gpio_set_level(HEAD_UNIT_EN_PIN, 1);
    gpio_set_level(BLIND_SPOT_L_EN_PIN, 1);
    gpio_set_level(BLIND_SPOT_R_EN_PIN, 1);
    gpio_set_level(DASH_CAM_EN_PIN, 1);
    gpio_set_level(BACKUP_CAM_EN_PIN, 1);

    gpio_set_pull_mode(HEAD_UNIT_SHUTDOWN_PIN, GPIO_PULLUP_ONLY);
    gpio_set_level(HEAD_UNIT_SHUTDOWN_PIN, 1); // head unit shutdown pin is active-low, so set it to high by default

    m_acc_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(accessory_task, "accessory_task", 2048, NULL, 10, NULL); // task to handle changes on the accessory sense pin

    gpio_config(&input_config);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ACCESSORY_SENSE_PIN, accessory_change_state_isr, NULL);

    m_shutdown_delay = xTimerCreate("shutdown_delay",
                                pdMS_TO_TICKS(SHUTDOWN_DELAY_MS),
                                pdFALSE,
                                (void*)0,
                                delayed_power_off);

    ESP_LOGI(TAG, "Power management initialized");
}
/******************************************************************************/ 

/* PRIVATE FUNCTIONS **********************************************************/
static void accessory_change_state_isr(void* arg)
{
    gpio_intr_disable(ACCESSORY_SENSE_PIN);
    xSemaphoreGiveFromISR(m_acc_semaphore, NULL);
}

static void accessory_task(void* arg)
{
    uint16_t pin_history = 0b1111111111111111;
    uint8_t check_count = 0;

    while(true) {
        if (xSemaphoreTake(m_acc_semaphore, portMAX_DELAY) == pdTRUE) { 
            check_count = 0;
            pin_history = 0b1111111111111111;

            ESP_LOGD(TAG, "Starting deounce");
            while (check_count < ACC_PIN_DEBOUNCE_MS / 5) { // deboucning the pin
                pin_history = pin_history << 1;
                pin_history |= gpio_get_level(ACCESSORY_SENSE_PIN);

                check_count++;
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            ESP_LOGD(TAG, "Debounce done");
        }
        ESP_LOGD(TAG, "Pin history: %d", pin_history);

        if ((pin_history & 0b0000000000001111) == 0b0000000000000000) { // pin low
            ESP_LOGI(TAG, "Accessory off");
            led_off(); // turn off the accessory LED
            power_off_system();
        } else if((pin_history & 0b0000000000001111) == 0b0000000000001111) { // pin high
            ESP_LOGI(TAG, "Accessory on");
            led_on(); // turn on the accessory LED
            power_on_system();
        }
        gpio_intr_enable(ACCESSORY_SENSE_PIN);
    }
}

static void power_on_system(void)
{
    if (xTimerIsTimerActive(m_shutdown_delay)) { // stopping the shutdown timer if it is active
        xTimerStop(m_shutdown_delay, 0);
    }

    // turn on all the power outputs
    gpio_set_level(HEAD_UNIT_EN_PIN, 0);
    gpio_set_level(DASH_CAM_EN_PIN, 0);
    gpio_set_level(BLIND_SPOT_L_EN_PIN, 0);
    gpio_set_level(BLIND_SPOT_R_EN_PIN, 0);
    gpio_set_level(BACKUP_CAM_EN_PIN, 0);
}

static void power_off_system(void)
{
    // To power off the system, power to the blind spot detectors and backup cam
    // is cut immediately. For the head unit and dash cam, they need some time
    // to shut down properly, so a shutoff signal is sent to them, and then
    // there is a delay before power is cut.
    gpio_set_level(BLIND_SPOT_L_EN_PIN, 1);
    gpio_set_level(BLIND_SPOT_R_EN_PIN, 1);
    gpio_set_level(BACKUP_CAM_EN_PIN, 1);

    // send a shutdown signal to the dash cam an head unit to start the shutdown process
    gpio_set_level(HEAD_UNIT_SHUTDOWN_PIN, 1); // head unit shutdown pin is active-low
    gpio_set_level(DASH_CAM_SHUTDOWN_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(HEAD_UNIT_SHUTDOWN_PIN, 0);
    gpio_set_level(DASH_CAM_SHUTDOWN_PIN, 1);

    xTimerStart(m_shutdown_delay, 0); // start the timer to delay the shutdown of the head unit and dash cam
}

static void delayed_power_off(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Shutdown delay expired - disabling power to head unit and dash cam");
    gpio_set_level(HEAD_UNIT_EN_PIN, 1);
    gpio_set_level(DASH_CAM_EN_PIN, 1);
}
/******************************************************************************/
