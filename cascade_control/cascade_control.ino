// Código para realizar controle em cascata dos motores do Omnibot

// Importar bibliotecas
#include <util/atomic.h>

// -------------- DC Motor --------------

class DCMotor {  
  int spd = 255, pin1, pin2;
  
  public:  
  
    void Pinout(int in1, int in2){ // Pinout é o método para a declaração dos pinos que vão controlar o objeto motor
      pin1 = in1;
      pin2 = in2;
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      }   
    void Speed(int in1){ // Speed é o método que irá ser responsável por salvar a velocidade de atuação do motor
      spd = in1;
      }     
    void Forward(){ // Forward é o método para fazer o motor girar para frente
      analogWrite(pin1, spd);
      digitalWrite(pin2, LOW);
      }   
    void Backward(){ // Backward é o método para fazer o motor girar para trás
      digitalWrite(pin1, LOW);
      analogWrite(pin2, spd);
      }
    void Stop(){ // Stop é o metodo para fazer o motor ficar parado.
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      }
   };

// Instaciar objeto Motor
DCMotor Motor3;

// Definir Pinos Encoder
#define ENCA 3
#define ENCB 2

// Definir configurações sensor de corrente
const int ACS712_PIN = A0;       // Pino analógico conectado ao sensor
const float ACS712_SENSITIVITY = 0.185; // Sensibilidade do sensor (185 mV/A para 5A)
const float VOLTAGE_REF = 5.0;   // Tensão de referência do Arduino (5V)
const int ADC_RESOLUTION = 1023; // Resolução do ADC (10 bits)
float zeroCurrentOffset = 0.0;

// Variáveis globais para estimação da velocidade

long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
// Filtro passa-baixas na velocidade
float v2Filt = 0;
float v2Prev = 0;

// Parâmetros do controlador
float integral_vel = 0.0;       // Termo integral velocidade
float integral_cur = 0.0;       // Termo integral corrente
float prev_error_vel = 0.0;     // Erro anterior velocidade
float prev_error_cur = 0.0;     // Erro anterior corrente
// Controlador de Velocidade (PI)
float Kp_vel = 0.00023;         // Ganho proporcional
float Ki_vel = 0.00056;          // Ganho integral
float vel_ref = 65.0;           // Velocidade de referência (RPM)
float vel_max = 120.0;          // Velocidade máxima (RPM)
float vel_min = -120.0;         // Velocidade mínima (RPM)
// Controlador de Corrente (PI)
float Kp_cur = 0.387;         // Ganho proporcional
float Ki_cur = 16.6818;     // Ganho integral
float cur_max = 2.0;            // Limite superior anti-windup (A)
float cur_min = -2.0;           // Limite inferior anti-windup (A)
float volt_max = 6;          // Tensão máxima (V)
float volt_min = -6;         // Tensão mínima (V)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  // Setup current sensor
  analogReference(DEFAULT); // Usa a referência padrão (5V)
  calibrateZeroCurrent();   // Calibração inicial

  // Setup Motors
  Motor3.Pinout(6,7);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Medir o tempo gasto entre loops
  unsigned long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  // Update the previous time measurement with current time
  prevT = currT; 
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Calcular velocidade em RPM
  float v2 = velocity2/676.0*60.0;

  // Calcular velocidade filtrada
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // if (v2Filt < 7){
  //   v2Filt = 0;
  // } 

  float current = readCurrentACS712();

// =============================================
// CONTROLE EM CASCATA
// =============================================

  // Malha externa (velocidade)
  float current_ref = velocityController(vel_ref, v2Filt, deltaT);
  
  // Malha interna (corrente)
  float voltage_ref = currentController(current_ref, current, deltaT);
  
  // 3. Aplicação ao motor
  int pwmOutput = voltageToPWM(voltage_ref);

  int pwr = (int) fabs(pwmOutput);
  if(pwr > 255){
    pwr = 255;
  }
  // Setup motor velocity with pwm
  
  Motor3.Speed(pwr); 

  if(pwmOutput > 0){
    // Motor moving forward
    Motor3.Forward();
  }
  else{
    // Motor moving forward
    Motor3.Backward();
  }

  Serial.print(v2Filt);
  Serial.print(" ");
  Serial.print(current_ref);
  Serial.print(" ");
  Serial.print(voltage_ref);
  Serial.println();
  delay(1);
}

// Função para leitura de encoder e estimação velocidade
void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
// Função para medir corrente pelo sensor
float readCurrentACS712() {
  int rawValue = analogRead(ACS712_PIN);
  float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
  float current = (voltage - (VOLTAGE_REF / 2) - zeroCurrentOffset) / ACS712_SENSITIVITY;
  return -current;
}
// Função para calibrar o offset (execute sem corrente)
void calibrateZeroCurrent() {
  Serial.println("Calibrando offset... (NÃO aplique corrente)");
  delay(2000); // Tempo para estabilização
  
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    int rawValue = analogRead(ACS712_PIN);
    float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
    sum += (voltage - (VOLTAGE_REF / 2)); // Remove o offset teórico (2.5V)
    delay(10);
  }
  
  zeroCurrentOffset = sum / 100; // Offset médio
  Serial.print("Offset calibrado: ");
  Serial.println(zeroCurrentOffset, 5);
}
// Controlador PI de Velocidade com anti-windup
float velocityController(float ref, float meas, float deltaT) {
  float error = ref - meas;
  
  // Termo proporcional
  float P = Kp_vel * error;
  
  // Termo integral com anti-windup
  integral_vel += Ki_vel * error * deltaT;
  
  // Limite anti-windup (1 a -1 A como especificado)
  // integral_vel = constrain(integral_vel, cur_min, cur_max);
  
  // Saída do controlador (referência de corrente)
  float output = P + integral_vel;
  
  // Saturação da saída - Limite anti-windup (1 a -1 A como especificado)
  //output = constrain(output, cur_min, cur_max);
  
  return output;
}
// Controlador PI de Corrente com anti-windup
float currentController(float ref, float meas, float deltaT) {
  float error = ref - meas;
  
  // Termo proporcional
  float P = Kp_cur * error;
  
  // Termo integral com anti-windup
  integral_cur += Ki_cur * error * deltaT;
  
  // Limite anti-windup (12 a -12 V como especificado)
  // integral_cur = constrain(integral_cur, volt_min, volt_max);
  
  // Saída do controlador (tensão PWM)
  float output = P + integral_cur;
  
  // Saturação da saída
  //output = constrain(output, volt_min, volt_max);
  
  return output;
}

int voltageToPWM(float voltage) {
  // Limitar tensão aos limites do driver
  voltage = constrain(voltage, -6, 6);
  
  // Converter para PWM
  int pwmValue = (int)((voltage / 6.0) * 255.0);

  return pwmValue;
}
