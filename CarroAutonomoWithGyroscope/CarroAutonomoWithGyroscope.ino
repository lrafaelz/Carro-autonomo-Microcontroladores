//Carrega a biblioteca do sensor ultrassonico
#include <Ultrasonic.h>
#include <Servo.h> //INCLUSÃO DA BIBLIOTECA NECESSÁRIA
#include "Wire.h"       
#include "I2Cdev.h"     
#include "MPU6050.h"  
 
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
int speed = 200;   // Define velocidade padrão 0 < x < 255.
float US = 0;

// MPU
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int G = 56;
double nGz;
unsigned long TOTAL_MillisA, Millis = 0;
 void Solic_MPU6050();
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
  s.write(90); //INICIA O MOTOR NA POSIÇÃO 0º]
  double getInit = getUltrasonicData();
  Serial.println(getInit);
  sei();
  Wire.begin();
  mpu.initialize();
}
 
void loop(){
  pos = 90;
  s.write(pos);
  delay(180);
  do{
    front(0);
    US = getUltrasonicData();
    Serial.println(US);
  }
  while(US > 20 && pos == 90 && US < 2000);
  // Serial.println("indo para frente < 25");
  front(90);
  pos = 180;
  s.write(pos);
  delay(180);
  stop();
  US = getUltrasonicData();
  if(US < 30 || US > 1500){
    // Serial.println(US);
    right(55);
    Contagem_giro();
  }
  else{
    US = getUltrasonicData();
              // Serial.println(US);
    left(55);
    Contagem_giro();
  }
  US = getUltrasonicData();

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
  Serial.println("DIREITAA");
}

void left(int incress){
  analogWrite(motorA1, speed + incress); 
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 0);    
  analogWrite(motorB2, speed + incress);
  Serial.println("ESQUERDAA");

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

void Contagem_giro() {
  int Ctt = 0;
  float Z = 0;
  float Giro_Z = 0;
  float Giro_ZA = 0;
  unsigned long TOTAL_MillisB = 0;
  while(Giro_ZA <= G){
    TOTAL_MillisA = millis(); //pega o valor inicial em ms.
    while(Millis <= 25){// enquanto não se passar 100ms ele pega valores do MPU6050
      Ctt++; //contador para média dos graus por segundo em 100ms
       mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
       nGz = gz/131;
       if(nGz < 0)
        nGz = nGz * -1;
      Z = Z + nGz; //faz somatório dos valores de º/s para tirar média depois
      TOTAL_MillisB = millis();//pega valor em ms atual
      Millis = TOTAL_MillisB - TOTAL_MillisA; //vai atualizando Millis até estourar em 100ms 
    }// após passar os 100ms, vamos calcular o quanto já girou
    Millis = 0;// zera millis para a proxima rotina
    Z = Z/Ctt; // tiramos a média dos valores em º/s
    Giro_Z = Z * 0.025; // multiplica o valor da média que é em º/s
                        // e multiplica por 0,1 que equivale a 0,1s ou 100ms
    Giro_ZA = Giro_ZA + Giro_Z; //a cada 100ms um valor já deslocado é adicionado ao giro.
    Ctt = 0; //zera o contador para média
    TOTAL_MillisA = millis(); //adiciona o novo valor para a próxima contagem dos 100ms
    Serial.print("GiroZA: ");
    Serial.println(Giro_ZA);
  }
  Serial.println("saiu do while");
  stop();
  Giro_ZA = 0;
}
