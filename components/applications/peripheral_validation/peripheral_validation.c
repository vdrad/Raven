#include "peripheral_validation.h"
#include <stdbool.h>
#include "rgb_led.h"
#include "buzzer.h"

#define TAG "VALIDATION"

void peripheral_validation(peripheral_to_validate_t peripheral) {
    switch (peripheral) {
    case PERIPHERAL_RGB_LED:
        rgb_led_peripheral_validation();
        break;
    case PERIPHERAL_BUZZER:
        buzzer_peripheral_validation();
        break;

    default:
        break;
    }
}