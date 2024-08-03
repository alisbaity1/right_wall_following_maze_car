#include <SR04.h> 
// int ledRight = 12; //akhdar
// int ledFront = 12; //asfar
// int ledError = 


int interruptPin = 2;
// int trigF = 2;    // Trig - Front sensor
// int echoF = 6;    // Echo - Front sensor
int trigR = 10;    // Trig - Right sensor
int echoR = 11;    // Echo - Right sensor
SR04 rightDistance = SR04(echoR,trigR);
// SR04 frontDistance = SR04(echoF,trigF);


// M1 (right motor)
int enA = 9;
int in1 = 8;
int in2 = 7;
// M2 (Left motor)
int enB = 3; 
int in3 = 5;
int in4 = 4;

int motorSpeed = 127;//127

float targetDist = 20; // init in setup 
float actualDist = 0;
float lastDist = 0;

const float Kp = 1.8; //2
const int Ki = 0;
const float  Kd = 47; // 30 37 

float distError = 0;
float prevDistError = 0;
float derivative = 0;
float integral = 0;

float correction = 0;
float maxCorrection = 127;
float VoltageCorrection = 0;

// const float safeDist = 20;
bool emergencyStopFlag = 0 ;


void setup() {
  // pinMode(trigF,OUTPUT) ; 
  // pinMode(echoF,INPUT) ;
  pinMode(trigR,OUTPUT) ; 
  pinMode(echoR,INPUT) ;
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);  

  // pinMode(ledRight,OUTPUT);
  // pinMode(ledFront,OUTPUT);
  // pinMode(interruptPin,INPUT);
  // attachInterrupt(digitalPinToInterrupt(interruptPin), interFunc , FALLING);
  // Serial.begin(57600);

  // delay(1000);
  delay(60);

  for (int i=0; i<10; i++){
    actualDist = rightDistance.Distance();
    lastDist = actualDist;
    targetDist = actualDist;
    delay(60);
    }
  // delay(5000);
}

void loop() {
if(digitalRead(interruptPin) == LOW){ // ir sensor front 
// if(frontDistance.Distance() < safeDist){ // ultrasonic front 

  emergencyStop();
  turnLeft();

  }
else{

  actualDist = rightDistance.Distance();
  // if (actualDist == 0){
  //   digitalWrite(ledRight,1);
  // }

  // if (abs(actualDist - lastDist) >=50){
  //   emergencyStop();
  //   turnRight();
  // }
  // else{
  //   lastDist = actualDist;
  // }

  distError = actualDist - targetDist;
  derivative = distError - prevDistError;

  correction = Kp*distError + Kd* derivative;
  prevDistError = distError;

  // PID correction constant saturation and mapping 
  if (correction > maxCorrection){ correction  = maxCorrection;};
  if (correction < -1*maxCorrection){ correction  = -1*maxCorrection;};


  //VoltageCorrection = correction*25/maxCorrection;
  VoltageCorrection = correction;
  // //
  // Serial.print("Target = ");
  // Serial.print(targetDist);
  // Serial.print("  Dist = ");
  // Serial.print(actualDist);
  // Serial.print("    VoltageCorrection = ");
  // Serial.print(VoltageCorrection);
  // Serial.print("    RawCorr = ");
  // Serial.println(correction);

  steeringForward(VoltageCorrection);
    
  }
 delay(20);

}
 

void steeringForward(int correctionVoltage){
  if(emergencyStopFlag==1){
    emergencyStopFlag=0;}
  // Positive -> to the right
  digitalWrite(in1,0) ; 
  digitalWrite(in2,1) ; 
  digitalWrite(in3 , 1 ) ; 
  digitalWrite(in4 , 0) ;  

  analogWrite(enA,motorSpeed - correctionVoltage) ;
  analogWrite(enB,motorSpeed + correctionVoltage) ; 
  
}

void motorsStop(){
 
  analogWrite(enA,0) ;
  analogWrite(enB,0) ;
  }

void motorsBack(){
  digitalWrite(in1,1) ; 
digitalWrite(in2,0) ; 
digitalWrite(in3 , 0 ) ; 
digitalWrite(in4 , 1) ;  

analogWrite(enA,255) ;
analogWrite(enB,255) ; 

}
void emergencyStop(){ 
  if(emergencyStopFlag==0){
      // motorsStop();
  //   Serial.println("Stop Kleb ");
    motorsBack();
    
    // delayMicroseconds(16000);
    // delayMicroseconds(16000);
    // delayMicroseconds(16000);

    delay(50);

    emergencyStopFlag=1;
    }
    motorsStop();

}

// void turnLeftPID(int controlMotorSpeed){ 
//   // right +1 and left -1
//   digitalWrite(in1,0) ; 
//   digitalWrite(in2,1) ; 
//   digitalWrite(in3,1) ; 
//   digitalWrite(in4,0) ;
  
//   analogWrite(enA,controlMotorSpeed) ;
//   analogWrite(enB,0) ;  
//   // for (int i = 0; i<10;i++){
//   //   delayMicroseconds(15000);
//   // }
// }

void turnLeft(){ 
  // right +1 and left -1
  digitalWrite(in1,0) ; 
  digitalWrite(in2,1) ; 
  digitalWrite(in3,1) ; 
  digitalWrite(in4,0) ;
  
  analogWrite(enA,motorSpeed) ;
  analogWrite(enB,0) ;  
  delay(150);
  // for (int i = 0; i<10;i++){
  //   delayMicroseconds(15000);
  // }
}

void turnRight(){ 
  // right +1 and left -1
  digitalWrite(in1,0) ; 
  digitalWrite(in2,1) ; 
  digitalWrite(in3,1) ; 
  digitalWrite(in4,0) ;
  
  analogWrite(enA,0) ;
  analogWrite(enB,motorSpeed) ; 
  delay(150) ; 

}


