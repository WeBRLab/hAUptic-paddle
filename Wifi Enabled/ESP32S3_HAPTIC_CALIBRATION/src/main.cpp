#include <Arduino.h>

// Pin assignments
const int stdby = 1;
const int a1 = 2;
const int a2 = 4;
const int pwma = 26;
const int PIN = 5;


void setup() {
  Serial.begin(9600);

  Serial.println("System starting...");

  pinMode(PIN, OUTPUT);
  pinMode(stdby, OUTPUT);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(pwma, OUTPUT);
  digitalWrite(stdby, HIGH);
}

void loop() {
  int current_position = analogRead(a1);

  // Print raw sensor reading
  Serial.print("Sensor reading: ");
  Serial.println(current_position);

  
  delay(200); // slow down prints
}