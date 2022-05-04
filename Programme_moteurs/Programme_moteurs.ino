

#include <Servo.h>

#define PIN_IN1 4  // IN1 Module L298N
#define PIN_IN2 3  // IN2 Module L298N
#define PIN_ENA 5  // ENA Module L298N

#define PIN_IN3 2  // IN1 Module L298N
#define PIN_IN4 6  // IN2 Module L298N
#define PIN_ENB 9  // ENA Module L298N

#define POT_PIN1 A0 // A0 ADC Potentiometre
#define POT_PIN2 A1 // A1 ADC Potentiometre
#define POT_PIN3 A2 // A0 ADC Potentiometre
#define POT_PIN4 A3 // A1 ADC Potentiometre

Servo moteurAvant;  
Servo moteurArriere;
unsigned int moveDelay = 2000; // Temps en mouvement avant changement de direction
unsigned long startMoveAt; // Pour stocker la valeur en MS au moment du start

enum DIRECTIONS {
  FORWARD = 0,
  BACKWARD
};

DIRECTIONS nextDirection = FORWARD;

// Marche avant
void forward() {
  digitalWrite(PIN_IN1, HIGH);  // IN1 à l'état HAUT
  digitalWrite(PIN_IN3, HIGH);

  digitalWrite(PIN_IN2, LOW); // IN2 à l'état BAS
  digitalWrite(PIN_IN4, LOW);
}

// Marche arrière
void backward() {
  digitalWrite(PIN_IN1, LOW);  // IN1 à l'état BAS
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN2, HIGH); // IN2 à l'état HAUT
  digitalWrite(PIN_IN4, HIGH);
}

// Roue libre (avec un delay)
void freeWheel(unsigned int delayMs) {
  pinMode(PIN_ENA, OUTPUT); // ENA en sortie (pour déactiver le mode PWM)
  digitalWrite(PIN_IN1, LOW);  // IN1 à l'état BAS
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN2, LOW);  // IN2 à l'état BAS
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENA, LOW);  // ENA à l'état BAS
  digitalWrite(PIN_ENB, LOW);

  delay(delayMs);
}

void setup() {
  moteurAvant.attach(10); 
  moteurArriere.attach(11);
  
  pinMode(PIN_IN1, OUTPUT); // In1 en sortie
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN2, OUTPUT); // In2 en sortie
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); // ENA en sortie
  pinMode(PIN_ENB, OUTPUT);

  pinMode(POT_PIN1, INPUT);
  pinMode(POT_PIN2, INPUT);
  pinMode(POT_PIN3, INPUT);
  pinMode(POT_PIN4, INPUT);

  
  // Moteur en roue libre
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_ENB, LOW);
  Serial.begin(9600);
}

void loop() {
  // Récupération de la valeur du potentiometre
  uint16_t potValue1 = analogRead(POT_PIN1);
  uint16_t potValue2 = analogRead(POT_PIN2);
  uint16_t potValue3 = analogRead(POT_PIN3);
  uint16_t potValue4 = analogRead(POT_PIN4);
  Serial.print("Pot1 : ");
  Serial.println(potValue1);
  Serial.print("Pot2 : ");
  Serial.println(potValue2);
  Serial.print("Pot3 : ");
  Serial.println(potValue3);
  Serial.print("Pot4 : ");
  Serial.println(potValue4);
  Serial.println(" ");

  moteurAvant.write(map(potValue3, 0, 1023, 20, 170));
  moteurArriere.write(map(potValue4, 0, 1023, 20, 170));
  // Génération du signal PWM vers la PIN ENA du module L298N
  analogWrite(PIN_ENA, map(potValue1, 0, 1023, 0, 255));
  analogWrite(PIN_ENB, map(potValue2, 0, 1023, 0, 255));
  if ( (millis() - startMoveAt) >= moveDelay ) {
    switch (nextDirection) {
      case FORWARD:
        freeWheel(500); // en roue libre pendant 500ms
        // Marche avant pendant 2 secondes
        moveDelay = 2000;
        forward();
        startMoveAt = millis();
        nextDirection = BACKWARD;
        break;
      case BACKWARD:
        freeWheel(500); // en roue libre pendant 500ms
        // Marche arrière pendant 5 secondes
        moveDelay = 5000;
        backward();
        startMoveAt = millis();
        nextDirection = FORWARD;
        break;
    }
  }
}
