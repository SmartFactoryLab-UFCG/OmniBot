// Estimate the velocity of the Motors using Quadracture Encoders

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

DCMotor Motor1, Motor2, Motor3; 

// Pins Encoders
#define ENCA_1 2
#define ENCB_1 3
#define ENCA_2 18
#define ENCB_2 19
#define ENCA_3 20
#define ENCB_3 21

// Global variables for velocity estimation

// previous time mearusement 
long prevT = 0;
// previous encoder position 
int posPrev_1 = 0;
int posPrev_2 = 0;
int posPrev_3 = 0;
// Use the "volatile" directive for variables used in an interrupt
// encoder position
volatile int pos_i_1 = 0;
volatile int pos_i_2 = 0;
volatile int pos_i_3 = 0;

// Variables used in filtered velocity
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float v3Filt = 0;
float v3Prev = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Setup Encoders 1,2 and 3
  pinMode(ENCA_1,INPUT);
  pinMode(ENCB_1,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_1), readEncoder_1, RISING);

  pinMode(ENCA_2,INPUT);
  pinMode(ENCB_2,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_2), readEncoder_2, RISING);

  pinMode(ENCA_3,INPUT);
  pinMode(ENCB_3,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_3), readEncoder_3, RISING);

  // Setup Motors
  Motor1.Pinout(12,13);
  Motor2.Pinout(10,11);
  Motor3.Pinout(9,8);
}

void loop() {
  // read the encoder position in an atomic block to avoid potential misreads
  int pos_1 = 0;
  int pos_2 = 0;
  int pos_3 = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos_1 = pos_i_1;
    pos_2 = pos_i_2;
    pos_3 = pos_i_3;
  }

  // ------- Compute velocity with method 1 -----------

  // Compute time elapsed between loops
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;

  // Encoder 1
  float velocity1 = (pos_1 - posPrev_1)/deltaT;
  posPrev_1 = pos_1;
  // Encoder 2
  float velocity2 = (pos_2 - posPrev_2)/deltaT;
  posPrev_2 = pos_2;
  // Encoder 3
  float velocity3 = (pos_3 - posPrev_3)/deltaT;
  posPrev_3 = pos_3;

  // Update the previous time measurement with current time
  prevT = currT; 

  // ------- Convert count/s to RPM -------

  // Encoder 1
  float v1 = velocity1/676.0*60.0;
  // Encoder 2
  float v2 = velocity2/676.0*60.0;
  // Encoder 3
  float v3 = velocity3/676.0*60.0;

  // ------- Low-pass filter (25 Hz cutoff) -------
  // Review the implementation!!!
  // Encoder 1
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  // Encoder 2
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;
  // Encoder 3
  v3Filt = 0.854*v3Filt + 0.0728*v3 + 0.0728*v3Prev;
  v3Prev = v3;

  // Setup motor velocity with pwm
  Motor1.Speed(255); 
  Motor2.Speed(255);
  Motor3.Speed(255);

  // Motor moving forward
  Motor1.Forward();
  Motor2.Forward(); 
  Motor3.Forward();


  Serial.print(v1);
  Serial.print(" ");
  Serial.print(v2);
  Serial.print(" ");
  Serial.print(v3);
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

void readEncoder_2(){

  int increment_2 = 0;
  // Read encoder B when ENCA rises
  if(digitalRead(ENCB_2)>0){
    // If B is high, increment forward
    increment_2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_2 = -1;
  }
  pos_i_2 = pos_i_2 + increment_2;
}

void readEncoder_3(){

  int increment_3 = 0;
  // Read encoder B when ENCA rises
  if(digitalRead(ENCB_3)>0){
    // If B is high, increment forward
    increment_3 = 1;
  }
  else{
    // Otherwise, increment backward
    increment_3 = -1;
  }
  pos_i_3 = pos_i_3 + increment_3;
}
