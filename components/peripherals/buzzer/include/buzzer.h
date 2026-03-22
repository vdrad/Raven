#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t freq_hz;     
    uint32_t duration_ms; 
} buzzer_note_t;

void buzzer_init(void);
void buzzer_stop(void);
void buzzer_play(uint32_t freq_hz, uint32_t duration_ms);
void buzzer_peripheral_validation(void);