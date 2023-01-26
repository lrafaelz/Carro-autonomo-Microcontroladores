//Carrega a biblioteca do sensor ultrassonico
#include <Ultrasonic.h>
 
//Define os pinos para o trigger e echo (Sensor ultrasonico)
#define pino_trigger 7
#define pino_echo 6
// Define os pinos de utilização do Driver L298 (Driver do motor do carro)
#define motorA1 9    // Pin  5 of L293
#define motorA2 3    // Pin  6 of L293
#define motorB1 11   // Pin 10 of L293
#define motorB2 10   // Pin 9 of L293

int speed = 255;   // Define velocidade padrão 0 < x < 255.
 
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
  sei();
}
 
void loop(){
  do{
    front();
  }
  while(getUltrasonicData() > 17);
    stop();
    delay(20);
  do{
    right();
    delay(50);
  }
  while(getUltrasonicData() < 20);
  if(getUltrasonicData() >1000 || getUltrasonicData() < 20){
    left();
    delay(250);
  }
}

void front(){
  analogWrite(motorB1, speed);
  analogWrite(motorA1, speed);
  analogWrite(motorA2, 0);
  analogWrite(motorB2, 0);
}

void back(){
  analogWrite(motorB1, 0);
  analogWrite(motorA1, 0);
  analogWrite(motorA2, speed);
  analogWrite(motorB2, speed);
}

void right(){
  analogWrite(motorA1, 0); 
  analogWrite(motorA2, speed);
  analogWrite(motorB1, speed);    
  analogWrite(motorB2, 0);
}

void left(){
  analogWrite(motorA1, speed); 
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 0);    
  analogWrite(motorB2, speed);
}

void stop(){
  analogWrite(motorA1, 0); 
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 0);    
  analogWrite(motorB2, 0);
}

float getUltrasonicData(){
  //Le as informacoes do sensor em cm
  float cmMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  //Exibe informacoes do sensor ultrasonico no serial monitor
  Serial.print("Distancia em cm: ");
  Serial.println(cmMsec);
  return cmMsec;
}