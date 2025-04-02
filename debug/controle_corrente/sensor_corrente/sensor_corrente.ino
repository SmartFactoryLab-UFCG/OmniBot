#include <util/atomic.h>

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


DCMotor Motor1;

// Pins
#define ENCA 2
#define ENCB 3
// 5A - 185mV/A
// 20A - 100mV/A
// 30A - 66mV/A
const int ACS712_PIN = A0;       // Pino analógico conectado ao sensor
const float ACS712_SENSITIVITY = 0.185; // Sensibilidade do sensor (185 mV/A para 5A)
const float VOLTAGE_REF = 5.0;   // Tensão de referência do Arduino (5V)
const int ADC_RESOLUTION = 1023; // Resolução do ADC (10 bits)

//Offset de calibração (ajuste quando não houver corrente)
float zeroCurrentOffset = 0.0;

// Variáveis para filtro de média móvel
const int NUM_SAMPLES = 10;      // Número de amostras para filtro
float samples[NUM_SAMPLES];
int sampleIndex = 0;

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

const unsigned long PERIOD = 3000; // Período de 5 segundos (5000ms)

// Variáveis para controle de tempo
unsigned long previousTime = 0;
bool isMotorOn = true;      // Estado atual do motor

// float eintegral = 0;

void setup() {
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  // Setup current sensor
  analogReference(DEFAULT); // Usa a referência padrão (5V)
  calibrateZeroCurrent();   // Calibração inicial

  // Setup Motors
  Motor1.Pinout(12,13);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {
  unsigned long startTime = micros();

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // // Compute velocity with method 1
  // long currT = micros();
  // float deltaT = ((float) (currT-prevT))/1.0e6;
  // float velocity1 = (pos - posPrev)/deltaT;
  // posPrev = pos;
  // prevT = currT;

  // Convert count/s to RPM
  //float v1 = velocity1/676.0*60.0;
  float v2 = velocity2/676.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  //v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  //v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  if (v2Filt < 7){
    v2Filt = 0;
  } 

  unsigned long currentTime = millis();
  // // Verifica se é hora de alternar o estado do motor
  // if (currentTime - previousTime >= PERIOD) {
  //   previousTime = currentTime; // Reseta o contador de tempo
    
  //   if (isMotorOn) {
  //     // Desliga o motor
  //     Motor1.Speed(0);
  //     Motor1.Forward();
  //   } else {
  //     // Liga motor
  //     Motor1.Speed(255); // Máxima velocidade
  //     Motor1.Forward(); // Comando para o motor ir para frente
  //   }
    
  //   isMotorOn = !isMotorOn; // Inverte o estado
  // }

      Motor1.Speed(255); // Máxima velocidade
      Motor1.Forward(); // Comando para o motor ir para frente

  float current = readCurrentACS712();
  
  //Serial.print("Corrente: ");
  // Serial.print(0.5);
  // Serial.print(" ");
  // Serial.print(-0.5);
  // Serial.print(" ");
  Serial.print(current, 3); // 3 casas decimais
  //Serial.print(" ");
  //Serial.print(v2Filt);
  Serial.println();
  delay(1);

  unsigned long endTime = micros();
  float loopTime = (endTime - startTime) / 1000.0; // Tempo em ms
  //Serial.print("Tempo do loop: "); Serial.println(loopTime);  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

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

float readCurrentACS712() {
  int rawValue = analogRead(ACS712_PIN);
  float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
  float current = (voltage - (VOLTAGE_REF / 2) - zeroCurrentOffset) / ACS712_SENSITIVITY;
  return current;
}

// Função para ler corrente com filtro de média móvel
float readCurrentACS712_filter() {
  float sum = 0;
  
  // Adquire NUM_SAMPLES amostras
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int rawValue = analogRead(ACS712_PIN);
    float voltage = (rawValue * VOLTAGE_REF) / ADC_RESOLUTION;
    float current = (voltage - (VOLTAGE_REF / 2) - zeroCurrentOffset) / ACS712_SENSITIVITY;
    samples[i] = current;
    sum += current;
    delayMicroseconds(100); // Pequeno delay entre amostras
  }
  
  // Retorna a média das amostras
  return sum / NUM_SAMPLES;
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
