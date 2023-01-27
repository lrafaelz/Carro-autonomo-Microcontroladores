//Carrega a biblioteca do sensor ultrassonico
#include <Ultrasonic.h>
#include <Servo.h> //INCLUSÃO DA BIBLIOTECA NECESSÁRIA
 
//Define os pinos para o trigger e echo (Sensor ultrasonico)
#define pino_trigger 7
#define pino_echo 6
// Define os pinos de utilização do Driver L298 (Driver do motor do carro)
#define motorA1 9    // Pin  5 of L293
#define motorA2 3    // Pin  6 of L293
#define motorB1 11   // Pin 10 of L293
#define motorB2 10   // Pin 9 of L293
#define pinoServo 5 //PINO DIGITAL UTILIZADO PELO SERVO  
 
Servo s; //OBJETO DO TIPO SERVO
int pos = 90; //POSIÇÃO DO SERVO
int speed = 180;   // Define velocidade padrão 0 < x < 255.
float US = 0;
 
//Inicializa o sensor ultrasonico nos pinos definidos acima
Ultrasonic ultrasonic(pino_trigger, pino_echo);
 
void setup(){
  cli();
  Serial.begin(9600);
  Serial.println("Lendo dados do sensor ultrasonico...");
  // Inicializa as portas como entrada e saída do carro.
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  s.attach(pinoServo); //ASSOCIAÇÃO DO PINO DIGITAL AO OBJETO DO TIPO SERVO
  s.write(pos); //INICIA O MOTOR NA POSIÇÃO 0º
  double getInit = getUltrasonicData();
  Serial.println(getInit);
  sei();
}
 
void loop(){
  stop();
  while(US > 25 && pos == 90){
    front(0);
    US = getUltrasonicData();
    Serial.println(US);
  }
  front(120);
  pos = 180;
  s.write(pos);
  delay(250);
  US = getUltrasonicData();
  if(US < 30 || US > 1000){
    Serial.print("direita: ");    
    Serial.println(US);
    stop();
    right(65);
    delay(180);
  }
  else{
    US = getUltrasonicData();
    Serial.print("esquerda: "); 
    Serial.println(US);
    stop();
    left(75);
    delay(180);
  }
  pos = 90;
  s.write(pos);  
  delay(180);
  US = getUltrasonicData();
}
void front(int decress){
  int vSpeed = speed - decress;
  analogWrite(motorA1, vSpeed); 
  analogWrite(motorA2, 0);
  analogWrite(motorB1, vSpeed);    
  analogWrite(motorB2, 0);
}

void right(int incress){
  int vSpeed = speed + incress;  
  analogWrite(motorA1, 0); 
  analogWrite(motorA2, vSpeed);
  analogWrite(motorB1, vSpeed);    
  analogWrite(motorB2, 0);
  // Serial.println("direitaaaa");  
}

void left(int incress){
  int vSpeed = speed + incress;
  Serial.println(vSpeed);
  analogWrite(motorA1, vSpeed); 
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 0);    
  analogWrite(motorB2, vSpeed);
}

void stop(){
  analogWrite(motorA1, 0); 
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 0);    
  analogWrite(motorB2, 0);
}

float getUltrasonicData(){
  //Le as informacoes do sensor em cm
  float cmMsec[3];
  long microsec = ultrasonic.timing();
  cmMsec[0] = ultrasonic.convert(microsec, Ultrasonic::CM);
  cmMsec[1] = ultrasonic.convert(microsec, Ultrasonic::CM);
  cmMsec[2] = (cmMsec[0] + cmMsec[0])/2;
  //Exibe informacoes do sensor ultrasonico no serial monito
  // Serial.println(cmMsec[2]);


  return cmMsec[2];
}