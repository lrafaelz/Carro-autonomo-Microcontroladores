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
int speed = 170;   // Define velocidade padrão 0 < x < 255.
float US = 0;
 
//Inicializa o sensor ultrasonico nos pinos definidos acima
Ultrasonic ultrasonic(pino_trigger, pino_echo);
 
void setup()
{
  cli();
  Serial.begin(9600);
  Serial.println("Lendo dados do sensor ultrasonico...");
  // Inicializa as portas como entrada e saída do carro.
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  s.attach(pinoServo); //ASSOCIAÇÃO DO PINO DIGITAL AO OBJETO DO TIPO SERVO
  s.write(pos); //INICIA O MOTOR NA POSIÇÃO 0º]
  double getInit = getUltrasonicData();
  Serial.println(getInit);
  sei();
}
 
void loop(){
  s.write(pos);
  do{
    front(0);
    US = getUltrasonicData();
    Serial.println(US);
  }
  while(US > 20 && pos == 90 && US < 2000);
  // Serial.println("indo para frente < 25");
  front(80);
  pos = 180;
  s.write(pos);
  delay(250);
  stop();
  US = getUltrasonicData();
  if(US < 30 || US > 1500){
    // Serial.println(US);
    right(84);
    delay(300);
  }
  else{
    US = getUltrasonicData();
    // Serial.println(US);
    left(85);
    delay(300);
  }
  US = getUltrasonicData();
  pos = 90;
  delay(180);
}
void front(int decress){
  analogWrite(motorB1, speed - decress);
  analogWrite(motorA1, speed - decress);
  analogWrite(motorA2, 0);
  analogWrite(motorB2, 0);
}

void back(){
  analogWrite(motorB1, 0);
  analogWrite(motorA1, 0);
  analogWrite(motorA2, speed);
  analogWrite(motorB2, speed);
}

void right(int incress){
  analogWrite(motorA1, 0); 
  analogWrite(motorA2, speed + incress);
  analogWrite(motorB1, speed + incress);    
  analogWrite(motorB2, 0);
}

void left(int incress){
  analogWrite(motorA1, speed + incress); 
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 0);    
  analogWrite(motorB2, speed + incress);
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