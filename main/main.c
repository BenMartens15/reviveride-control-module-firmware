#include <stdio.h>
#include "engine_control.h"
#include "esp_now_core.h"
#include "power_management.h"

void app_main(void)
{
    esp_now_core_init();
    engine_control_init();
    power_management_init();
}
