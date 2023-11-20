#include <stdio.h>
#include "esp_now_core.h"
#include "engine_control.h"

void app_main(void)
{
    esp_now_core_init();
    engine_control_init();
}
