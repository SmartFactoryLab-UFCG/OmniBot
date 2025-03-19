// Implement the velocity estimation for one motor using Quadracture Encoder

// ------------------- DC MOTOR -------------------
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


DCMotor Motor1, Motor2, Motor3;

// Yellow	encoder A output
// White	encoder B output

#define ENCODER_1_A 21
#define ENCODER_1_B 20
#define ENCODER_2_A 2
#define ENCODER_2_B 3
// #define ENCODER_3_A 18
// #define ENCODER_3_B 19

volatile long totalPulses_1 = 0;
// volatile long totalPulses_3 = 0;
void countPulses_1(){
  if (digitalRead(ENCODER_1_B) > 0){
    totalPulses_1++;
  }
  else{
    totalPulses_1--;
  }

}

volatile long totalPulses_2 = 0;
// timeInitial = 0
// tempo_atual = 0
void countPulses_2(){ // get_time_between_pulses

  
  if (digitalRead(ENCODER_2_B) > 0){
    totalPulses_2++; 
  }
  else{
    totalPulses_2--;
  }

  // tempo_atual = micros()
  // dt = tempo_atual - tempo_inicial
  // tempo_inicial = tempo_atual

  // dt_minutes = dt * 1e-6  * 0.01666
}

// Cálculo da velocidade rpm
int interval = 1000; //  interval in ms
int countsPerRevolutation = 52*13;
long previousMillis = 0;
long currentMillis = 0;
int rpm_2 = 0;

// -------------------SETUP -------------------

void setup() {
  // Initialize serial port
  Serial.begin(115200);

  // Setup Motors
  Motor1.Pinout(4, 5);

  // Setupt encoders
  pinMode(ENCODER_2_A, INPUT);
  pinMode(ENCODER_2_B, INPUT);

  // Interrupçao dos encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_A),countPulses_2, RISING);
  totalPulses_2 = 0;
  previousMillis = millis();
}

void loop() {
  // rpm = 0.0014792899 / dt_minutes

  // put your main code here, to run repeatedly:
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    
    // Revolutions per minute (RPM) = (total encoder pulse in 1s / encoder ppr) x 60s
    rpm_2 = (float)(totalPulses_2 * 60 / countsPerRevolutation);

    // Only update display when there have readings
    if ( rpm_2 > 0) 
    {
      Serial.print(totalPulses_2);
      Serial.print(" pulse / ");
      Serial.print(countsPerRevolutation);
      Serial.print(" pulse per rotation x 60 seconds = ");
      Serial.print(rpm_2);
      Serial.println(" RPM");
    }
    else
    {
      Serial.print(totalPulses_2);
      Serial.print(" pulse / ");
      Serial.print(countsPerRevolutation);
      Serial.print(" pulse per rotation x 60 seconds = ");
      Serial.print(rpm_2);
      Serial.println(" RPM");
    }
    totalPulses_2 = 0;
  }

  Motor1.Speed(200);
  
  Motor1.Forward(); // Comando para o motor ir para frente

}
