#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// ============= PID (Sem alterações) =============
#define KP 18.0f
#define KI 16.0f
#define KD 0.0f
// ... (struct PIDController, pid_init, pid_compute continuam os mesmos) ...
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
    float dt = (now - pid->last_time_us) / 1e6f;
    if (dt <= 0.0f) dt = 1e-6f;
    pid->last_time_us = now;

    float error = pid->setpoint - measured_value;
    pid->integral += error * dt;

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

#define ENC1_A 15     
#define ENC1_B 14     
#define ENC2_A 13 
#define ENC2_B 12
#define ENC3_A 11
#define ENC3_B 10

Encoder encoder_array[3] = {
    {ENC1_A, ENC1_B, 0, 0, 0},
    {ENC2_A, ENC2_B, 0, 0, 0},
    {ENC3_A, ENC3_B, 0, 0, 0}
};

void setup_encoder(Encoder* enc) {
    gpio_init(enc->pinA);
    gpio_set_dir(enc->pinA, GPIO_IN);
    gpio_pull_up(enc->pinA);
    gpio_init(enc->pinB);
    gpio_set_dir(enc->pinB, GPIO_IN);
    gpio_pull_up(enc->pinB);
}

void gpio_callback(uint gpio, uint32_t events) {
    Encoder* encoder = NULL;
    if (gpio == encoder_array[0].pinA) encoder = &encoder_array[0];
    else if (gpio == encoder_array[1].pinA) encoder = &encoder_array[1];
    else if (gpio == encoder_array[2].pinA) encoder = &encoder_array[2];  
    if (!encoder) return;

    bool a = gpio_get(encoder->pinA);
    bool b = gpio_get(encoder->pinB);
    int8_t direction = (a ^ b) ? +1 : -1;
    encoder->direction = direction;
    uint64_t now = time_us_64();
    uint64_t delta_us = now - encoder->last_pulse_time_us;
    if (encoder->last_pulse_time_us != 0 && delta_us > 0) {
        encoder->velocity = (1e6 / (float)delta_us) * 0.08875f * direction;
    }
    encoder->last_pulse_time_us = now;    
}

// ============= MOTOR DRIVER =============
typedef struct {
    int EN;
    int IN1;
    int IN2;
} Motor;

Motor motor_array[3] = {
    {28, 27, 26},
    {22, 21, 20},
    {7, 2, 3}
};

void motor_setup(Motor* motor) {
    gpio_init(motor->IN1);
    gpio_set_dir(motor->IN1, GPIO_OUT);
    gpio_init(motor->IN2);
    gpio_set_dir(motor->IN2, GPIO_OUT);
    gpio_set_function(motor->EN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(motor->EN);
    pwm_set_wrap(slice_num, 249);
    pwm_set_clkdiv(slice_num, 500.0f);
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

// ============= COMUNICAÇÃO SERIAL =============

// Array para armazenar os setpoints recebidos do ROS
float setpoints[3] = {0.0f, 0.0f, 0.0f};

// Função para processar uma única mensagem de setpoint (ex: "1p30.0")
void parse_setpoint_token(char* token, PIDController* pids) {
    if (strlen(token) < 4) return;

    int motor_id;
    char sign;
    float value;

    // sscanf extrai os dados do token. Ex: "1p30.0" -> motor_id=1, sign='p', value=30.0
    if (sscanf(token, "%1d%c%f", &motor_id, &sign, &value) == 3) {
        int motor_index = motor_id - 1;
        if (motor_index >= 0 && motor_index < 3) {
            setpoints[motor_index] = (sign == 'p') ? value : -value;
            pids[motor_index].setpoint = value; // PID trabalha com valor absoluto
        }
    }
}

// Função para ler e processar todos os dados da porta serial
void read_serial_commands(PIDController* pids) {
    static char serial_buffer[128];
    static int buffer_pos = 0;
    
    int c = getchar_timeout_us(0); // Lê um caractere sem bloquear
    while (c != PICO_ERROR_TIMEOUT) {
        if (c == ',' || c == '\n') { // Fim de um token ou mensagem
            serial_buffer[buffer_pos] = '\0'; // Termina a string
            parse_setpoint_token(serial_buffer, pids);
            buffer_pos = 0; // Reinicia o buffer
        } else if (buffer_pos < sizeof(serial_buffer) - 1) {
            serial_buffer[buffer_pos++] = (char)c;
        }
        c = getchar_timeout_us(0);
    }
}

// Função para enviar as velocidades medidas para o ROS
void send_encoder_feedback() {
    // Formato: 1pAA.A,2nBB.B,3pCC.C,\n
    for (int i = 0; i < 3; ++i) {
        float vel = encoder_array[i].velocity;
        char sign = (vel >= 0) ? 'p' : 'n';
        float abs_vel = (vel >= 0) ? vel : -vel;
        
        // Adiciona um zero à esquerda se necessário
        if (abs_vel < 10.0) {
            printf("%d%c0%.1f,", i + 1, sign, abs_vel);
        } else {
            printf("%d%c%.1f,", i + 1, sign, abs_vel);
        }
    }
    printf("\n"); // Envia uma nova linha para indicar o fim da transmissão
}

// ============= MAIN ATUALIZADO =============

int main() {
    stdio_init_all();

    // Inicializa todos os motores e encoders usando loops
    for (int i = 0; i < 3; ++i) {
        motor_setup(&motor_array[i]);
        setup_encoder(&encoder_array[i]);
    }

    // Configura as interrupções dos encoders
    gpio_set_irq_enabled_with_callback(encoder_array[0].pinA, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(encoder_array[1].pinA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(encoder_array[2].pinA, GPIO_IRQ_EDGE_RISE, true);

    // Inicializa os 3 controladores PID
    PIDController pid_controllers[3];
    for (int i = 0; i < 3; ++i) {
        pid_init(&pid_controllers[i], KP, KI, KD, 0.0f, 0.0f, 255.0f);
    }

    // Timers para controlar a frequência das tarefas
    uint64_t last_feedback_time = time_us_64();
    const uint64_t feedback_interval_us = 50000; // 50ms -> 20Hz

    while (true) {
        // Tarefa 1: Ler comandos do ROS (executa o mais rápido possível)
        read_serial_commands(pid_controllers);
        
        // Tarefa 2: Loop de controle PID (executa o mais rápido possível)
        for (int i = 0; i < 3; ++i) {
            float measured_vel = encoder_array[i].velocity;
            // O PID trabalha com a velocidade absoluta
            float output = pid_compute(&pid_controllers[i], (measured_vel >= 0 ? measured_vel : -measured_vel));

            // A direção é definida pelo setpoint original
            if (setpoints[i] > 0) {
                motor_forward(&motor_array[i], (uint8_t)output);
            } else if (setpoints[i] < 0) {
                motor_backward(&motor_array[i], (uint8_t)output);
            } else {
                motor_stop(&motor_array[i]);
            }
        }

        // Tarefa 3: Enviar feedback para o ROS (executa periodicamente)
        uint64_t now = time_us_64();
        if (now - last_feedback_time > feedback_interval_us) {
            last_feedback_time = now;
            send_encoder_feedback();
        }
    }
}