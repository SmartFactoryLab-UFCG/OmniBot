// Teste de conexão dos motores do Omnibot com Ponte H


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

// ------------------- SETUP -------------------

void setup() {

  // Setup Motor
  Motor1.Pinout(4, 5);
  Motor2.Pinout(6, 7);
  Motor3.Pinout(8, 9);

}

void loop() {

  // Setar velocidade do motor
  Motor1.Speed(255); 
  Motor2.Speed(255);
  Motor3.Speed(255);
  // Motor girando para frente indefinidamente
  Motor1.Forward();
  Motor2.Forward(); 
  Motor3.Forward();  

}
