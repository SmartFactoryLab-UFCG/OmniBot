#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// ============= PID =============

// Inicialmente, considerar que todos os motores são iguais. 
// Por isso, possuem as mesmas constantes.
#define KP 18
#define KI 16
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
    uint pinA;
    uint pinB;
    volatile float velocity;
    volatile int8_t direction;
    volatile uint64_t last_pulse_time_us;

} Encoder;

// Inicialização dos Encoders
#define ENC1_A 15     
#define ENC1_B 14     
#define ENC2_A 13 
#define ENC2_B 12
#define ENC3_A 11
#define ENC3_B 10

Encoder encoder1 = {ENC1_A, ENC1_B, 0, 0, 0};
Encoder encoder2 = {ENC2_A, ENC2_B, 0, 0, 0};
Encoder encoder3 = {ENC3_A, ENC3_B, 0, 0, 0};

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

// Função de callback para interrupção do canal A
void gpio_callback(uint gpio, uint32_t events) {
    Encoder* encoder = NULL;

    if      (gpio == encoder1.pinA) encoder = &encoder1;
    else if (gpio == encoder2.pinA) encoder = &encoder2;
    else if (gpio == encoder3.pinA) encoder = &encoder3;  

    if (!encoder) return;

    bool a = gpio_get(encoder->pinA);
    bool b = gpio_get(encoder->pinB);

    int8_t direction = (a ^ b) ? +1 : -1;
    encoder->direction = direction;

    uint64_t now = time_us_64();
    uint64_t delta_us = now - encoder->last_pulse_time_us;

    if (encoder->last_pulse_time_us != 0 && delta_us > 0) {
        // Fator 0.08875 depende do número de pulsos por rotação
        encoder->velocity = (1e6 / (float)delta_us) * 0.08875f * direction;
    }

    encoder->last_pulse_time_us = now;    
}

// ============= MOTOR GPIO PINS =============

typedef struct {
    int EN;
    int IN1;
    int IN2;
} Motor;

// Inicialização dos Motores
Motor motor01 = {28,27,26};
Motor motor02 = {22,21,20};
Motor motor03 = {7,2,3};


// ============= MOTOR DRIVER =============

void motor_setup(Motor* motor) {
    // Direção
    gpio_init(motor->IN1);
    gpio_set_dir(motor->IN1, GPIO_OUT);

    gpio_init(motor->IN2);
    gpio_set_dir(motor->IN2, GPIO_OUT);

    // PWM
    gpio_set_function(motor->EN, GPIO_FUNC_PWM); // habilita função PWM

    uint slice_num = pwm_gpio_to_slice_num(motor->EN);
    // pwm_set_wrap(slice_num, 255);  // Resolução de 8 bits

    pwm_set_wrap(slice_num, 249);           // wrap + 1 = 250
    pwm_set_clkdiv(slice_num, 500.0f);       // divisor para 1kHz
    pwm_set_enabled(slice_num, true);
}

void motor_forward(Motor* motor, uint8_t duty) {
    gpio_put(motor->IN1, 1);
    gpio_put(motor->IN2, 0);

    uint slice_num = pwm_gpio_to_slice_num(motor->EN);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(motor->EN), duty);
}

void motor_backward(Motor* motor, uint8_t duty) {
    gpio_put(motor->IN1, 0);
    gpio_put(motor->IN2, 1);

    uint slice_num = pwm_gpio_to_slice_num(motor->EN);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(motor->EN), duty);
}

void motor_stop(Motor* motor) {
    gpio_put(motor->IN1, 0);
    gpio_put(motor->IN2, 0);

    uint slice_num = pwm_gpio_to_slice_num(motor->EN);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(motor->EN), 0);
}

// ============= MAIN =============

int main() {
    stdio_init_all();

    motor_setup(&motor01);
    motor_setup(&motor02);
    motor_setup(&motor03);

    setup_encoder(&encoder1);
    setup_encoder(&encoder2);
    setup_encoder(&encoder3);

    // Registra o callback global
    gpio_set_irq_enabled_with_callback(encoder1.pinA, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(encoder2.pinA, GPIO_IRQ_EDGE_RISE, true);  // sem registrar novamente o callback
    gpio_set_irq_enabled(encoder3.pinA, GPIO_IRQ_EDGE_RISE, true);  // sem registrar novamente o callback

    // Definir setpoints
    int setpoint_1 = -36;
    int setpoint_2 = 0;
    int setpoint_3 = 36;

    // Inicializar controlador 
    PIDController motor1_pid, motor2_pid, motor3_pid;
    pid_init(&motor1_pid, KP, KI, KD, abs(setpoint_1), 0.0f, 255.0f);
    pid_init(&motor2_pid, KP, KI, KD, abs(setpoint_2), 0.0f, 255.0f);
    pid_init(&motor3_pid, KP, KI, KD, abs(setpoint_3), 0.0f, 255.0f);

    while (true) {
        //printf("Velocity 1 %.2f | Velocity 2 %.2f | Velocity 3 %.2f\n", encoder1.velocity, encoder2.velocity, encoder3.velocity);
        //Para o Motor 01
        float output_1 = pid_compute(&motor1_pid, abs(encoder1.velocity));
        if (setpoint_1 > 0){
            motor_forward(&motor01, output_1);
            printf("Setpoint: %.1f | Vel: %.1f RPM | PWM: %.0f\n", motor1_pid.setpoint, encoder1.velocity, output_1);
        }
        else if (setpoint_1 < 0)
        {
            motor_backward(&motor01, output_1);
            printf("Setpoint:- %.1f | Vel: %.1f RPM | PWM: %.0f\n", motor1_pid.setpoint, encoder1.velocity, output_1);
        }

        //Para o Motor 02
        float output_2 = pid_compute(&motor2_pid, abs(encoder2.velocity));
        if (setpoint_2 > 0){
            motor_forward(&motor02, output_2);
            printf("Setpoint: %.1f | Vel: %.1f RPM | PWM: %.0f\n", motor2_pid.setpoint, encoder2.velocity, output_2);
        }
        else if (setpoint_2 < 0)
        {
            motor_backward(&motor02, output_2);
            printf("Setpoint:- %.1f | Vel: %.1f RPM | PWM: %.0f\n", motor2_pid.setpoint, encoder2.velocity, output_2);
        }
 
        // // Para o Motor 03
        float output_3 = pid_compute(&motor3_pid, abs(encoder3.velocity));
        if (setpoint_3 > 0){
            motor_forward(&motor03, output_3);
            printf("Setpoint: %.1f | Vel: %.1f RPM | PWM: %.0f\n", motor3_pid.setpoint, encoder3.velocity, output_3);
        }
        else if (setpoint_3 < 0)
        {
            motor_backward(&motor03, output_3);
            printf("Setpoint:- %.1f | Vel: %.1f RPM | PWM: %.0f\n", motor3_pid.setpoint, encoder3.velocity, output_3);
        }
        sleep_ms(1); 
    }
}
