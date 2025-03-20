// Implementation of velocity control for DC Motor

#include <util/atomic.h>

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

// Pins Encoders
#define ENCA_1 2
#define ENCB_1 3

// Global variables for velocity estimation

// previous time mearusement 
long prevT = 0;
// previous encoder position 
int posPrev_1 = 0;
// Use the "volatile" directive for variables used in an interrupt
// encoder position
volatile int pos_i_1 = 0;

// Variables used in filtered velocity
float v1Filt = 0;
float v1Prev = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Setup Encoders 1,2 and 3
  pinMode(ENCA_1,INPUT);
  pinMode(ENCB_1,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_1), readEncoder_1, RISING);

  // Setup Motors
  Motor1.Pinout(12,13);
}

void loop() {
  // read the encoder position in an atomic block to avoid potential misreads
  int pos_1 = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos_1 = pos_i_1;
  }

  // ------- Compute velocity with method 1 -----------

  // Compute time elapsed between loops
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;

  // Encoder 1
  float velocity1 = (pos_1 - posPrev_1)/deltaT;
  posPrev_1 = pos_1;

  // Update the previous time measurement with current time
  prevT = currT; 

  // ------- Convert count/s to RPM -------

  // Encoder 1
  float v1 = velocity1/676.0*60.0;

  // ------- Low-pass filter (25 Hz cutoff) -------
  // Review the implementation!!!
  // Encoder 1
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;

  // Setup motor velocity with pwm
  Motor1.Speed(255); 

  // Motor moving forward
  Motor1.Forward();

  Serial.print(v1Filt);
  Serial.println();
  delay(1);

}

void readEncoder_1(){

  int increment_1 = 0;
  // Read encoder B when ENCA rises
  if(digitalRead(ENCB_1)>0){
    // If B is high, increment forward
    increment_1 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_1 = -1;
  }
  pos_i_1 = pos_i_1 + increment_1;
}