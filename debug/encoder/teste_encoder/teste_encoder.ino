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


DCMotor Motor1;


#define ENCODER_1_A 2 // Yellow	encoder A output
#define ENCODER_1_B 3 // White	encoder B output


volatile long totalPulses_1 = 0;

void countPulses_1(){
  if (digitalRead(ENCODER_1_B) > 0){
    totalPulses_1++;
  }
  else{
    totalPulses_1--;
  }

}

  // tempo_atual = micros()
  // dt = tempo_atual - tempo_inicial
  // tempo_inicial = tempo_atual

  // dt_minutes = dt * 1e-6  * 0.01666


// Cálculo da velocidade rpm
int interval = 1000; //  interval in ms
int countsPerRevolutation = 52*13;
long previousMillis = 0;
long currentMillis = 0;
int rpm_1 = 0;

// -------------------SETUP -------------------

void setup() {
  // Initialize serial port
  Serial.begin(115200);

  // Setup Motors
  Motor1.Pinout(4, 5);

  // Setupt encoders
  pinMode(ENCODER_1_A, INPUT);
  pinMode(ENCODER_1_B, INPUT);

  // Interrupçao dos encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_A),countPulses_1, RISING);
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
    rpm_1 = (float)(totalPulses_1 * 60 / countsPerRevolutation);

    // Only update display when there have readings
    if ( rpm_1 > 0) 
    {
      Serial.print(rpm_1);
      Serial.println(" RPM");
    }
    else
    {
      Serial.print(rpm_1);
      Serial.println(" RPM");
    }
    totalPulses_1 = 0;
  }

  Motor1.Speed(255); // Máxima velocidade
  
  Motor1.Forward(); // Comando para o motor ir para frente

}
