#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

typedef struct {
    uint encoder_a_pin;
    uint encoder_b_pin;
    volatile int32_t position;
    volatile int8_t direction;
    volatile float velocity;
    volatile uint64_t prev_time;
} Encoder;

Encoder encoder1 = {2, 3, 0, 0, 0, 0};
Encoder encoder2 = {4, 5, 0, 0, 0, 0};

void gpio_callback(uint gpio, uint32_t events) {
    Encoder* encoder = NULL;

    if (gpio == encoder1.encoder_a_pin)
        encoder = &encoder1;
    else if (gpio == encoder2.encoder_a_pin)
        encoder = &encoder2;

    if (!encoder) return;

    bool a = gpio_get(encoder->encoder_a_pin);
    bool b = gpio_get(encoder->encoder_b_pin);

    int8_t direction = (a ^ b) ? +1 : -1;
    encoder->position += direction;
    encoder->direction = direction;

    uint64_t now = time_us_64();
    if (encoder->prev_time != 0) {
        uint64_t delta_us = now - encoder->prev_time;
        if (delta_us > 200) {  // evita ruído
            encoder->velocity = (1e6f / (float)delta_us) * 0.08875f * direction;
        }
    }
    encoder->prev_time = now;
}

void setup_encoder(Encoder* enc) {
    gpio_init(enc->encoder_a_pin);
    gpio_set_dir(enc->encoder_a_pin, GPIO_IN);
    gpio_pull_up(enc->encoder_a_pin);

    gpio_init(enc->encoder_b_pin);
    gpio_set_dir(enc->encoder_b_pin, GPIO_IN);
    gpio_pull_up(enc->encoder_b_pin);

    gpio_set_irq_enabled(enc->encoder_a_pin, GPIO_IRQ_EDGE_RISE, true);
}

int main() {
    stdio_init_all();

    setup_encoder(&encoder1);
    setup_encoder(&encoder2);

    // Registra o callback global
    gpio_set_irq_enabled_with_callback(encoder1.encoder_a_pin, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(encoder2.encoder_a_pin, GPIO_IRQ_EDGE_RISE, true);  // sem registrar novamente o callback

    while (true) {
        printf("Encoder 1: %.2f RPM\t\tEncoder 2: %.2f RPM\n", encoder1.velocity, encoder2.velocity);
        sleep_ms(50);
    }
}
