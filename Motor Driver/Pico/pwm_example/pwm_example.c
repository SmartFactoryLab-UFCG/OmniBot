#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// ============= GPIO PINS =============

#define PWM_PIN 8      // ENA
#define IN1_PIN 10     // IN1
#define IN2_PIN 11     // IN2
#define ENC1_A 2       // ENCODER 1 - A
#define ENC1_B 3       // ENCODER 1 - B

// ============= PID =============

#define SETPOINT 60
#define KP 18
#define KI 15
#define KD 0

typedef struct {
    float Kp, Ki, Kd;
    float setpoint;

    float integral;
    float previous_error;

    float output_min, output_max;

    uint64_t last_time_us;
} PIDController;

void pid_init(PIDController* pid, float Kp, float Ki, float Kd, float setpoint, float min_out, float max_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_min = min_out;
    pid->output_max = max_out;
    pid->last_time_us = time_us_64();
}

float pid_compute(PIDController* pid, float measured_value) {
    uint64_t now = time_us_64();
    float dt = (now - pid->last_time_us) / 1e6f; // Convert microseconds to seconds
    if (dt <= 0.0f) dt = 1e-6f; // evita divisão por zero
    pid->last_time_us = now;

    float error = pid->setpoint - measured_value;

    pid->integral += error * dt;

    // // Proteção contra wind-up
    // if (pid->Ki != 0) {
    //     float integral_term = pid->integral * pid->Ki;
    //     if (integral_term > pid->output_max) pid->integral = pid->output_max / pid->Ki;
    //     if (integral_term < pid->output_min) pid->integral = pid->output_min / pid->Ki;
    // }

    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}


// ============= ENCODER =============

typedef struct {
    // Configuração
    uint pinA;
    uint pinB;
    volatile float velocity;
    volatile int8_t direction;
    volatile uint64_t last_pulse_time_us;

} Encoder;

// Inicialização do encoder em pinos 2 (A) e 3 (B)
Encoder encoder1 = {ENC1_A, ENC1_B, 0.0, 0, 0};

// Função de callback para interrupção do canal A
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio != encoder1.pinA) return;

    bool a = gpio_get(encoder1.pinA);
    bool b = gpio_get(encoder1.pinB);

    int8_t direction = (a ^ b) ? +1 : -1;
    encoder1.direction = direction;

    uint64_t now = time_us_64();
    uint64_t delta_us = now - encoder1.last_pulse_time_us;

    if (encoder1.last_pulse_time_us != 0 && delta_us > 0) {
        // Fator 0.08875 depende do número de pulsos por rotação
        encoder1.velocity = (1e6 / (float)delta_us) * 0.08875f * direction;
    }

    encoder1.last_pulse_time_us = now;    
}

// Configuração dos pinos do encoder
void setup_encoder(Encoder* enc) {
    gpio_init(enc->pinA);
    gpio_set_dir(enc->pinA, GPIO_IN);
    gpio_pull_up(enc->pinA);

    gpio_init(enc->pinB);
    gpio_set_dir(enc->pinB, GPIO_IN);
    gpio_pull_up(enc->pinB);

    gpio_set_irq_enabled(enc->pinA, GPIO_IRQ_EDGE_RISE, true);
}

// ============= MOTOR DRIVER =============

void motor_setup() {
    // Direção
    gpio_init(IN1_PIN);
    gpio_set_dir(IN1_PIN, GPIO_OUT);

    gpio_init(IN2_PIN);
    gpio_set_dir(IN2_PIN, GPIO_OUT);

    // PWM
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM); // habilita função PWM

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_num, 255);  // Resolução de 8 bits
    pwm_set_enabled(slice_num, true);
}

void motor_forward(uint8_t duty) {
    gpio_put(IN1_PIN, 1);
    gpio_put(IN2_PIN, 0);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), duty);
}

void motor_backward(uint8_t duty) {
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 1);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), duty);
}

void motor_stop() {
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 0);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), 0);
}

// ============= MAIN =============

int main() {
    stdio_init_all();
    motor_setup();
    setup_encoder(&encoder1);

    gpio_set_irq_enabled_with_callback(encoder1.pinA, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    PIDController motor_pid;
    pid_init(&motor_pid, KP, KI, KD, SETPOINT, -255.0f, 255.0f); // setpoint: 100 RPM

    while (true) {
        float output = pid_compute(&motor_pid, encoder1.velocity);
        motor_forward((uint8_t)output);
        printf("Setpoint: %.1f | Vel: %.1f RPM | PWM: %.0f\n", motor_pid.setpoint, encoder1.velocity, output);
        sleep_ms(1); 
    }
}
