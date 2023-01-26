// Define os pinos de utilização do Driver L298.
#define motorA1 9    // Pin  5 of L293
#define motorA2 3    // Pin  6 of L293
#define motorB1 11   // Pin 10 of L293
#define motorB2 10   // Pin 9 of L293
#include <Servo.h>

// Variáveis Úteis
int vSpeed = 254;   // Define velocidade padrão 0 < x < 255.

void setup() {
  // Inicializa as portas como entrada e saída.
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
}

void loop() {
  analogWrite(motorB1, vSpeed);
  analogWrite(motorA1, vSpeed);
  analogWrite(motorA2, 0);
  analogWrite(motorB2, 0);
}
