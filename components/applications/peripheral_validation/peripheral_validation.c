#include "peripheral_validation.h"
#include <stdbool.h>
#include "rgb_led.h"

#define TAG "VALIDATION"
static bool initialized = false;

void peripheral_validation(peripheral_to_validate_t peripheral) {
    switch (peripheral) {
    case PERIPHERAL_RGB_LED:
        rgb_led_peripheral_validation();
        break;

    default:
        break;
    }
}