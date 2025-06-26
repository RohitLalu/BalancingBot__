#include <Arduino.h>
#include <Wire.h>
#define PIN1 49 //left
#define PIN2 53 // mid
#define PIN3 51 // right


void setup() {
  Serial.begin(115200);
  pinMode(PIN1, INPUT);
  pinMode(PIN2, INPUT);
  pinMode(PIN3, INPUT);
  Serial.println("Setup complete. Pins initialized.");
}

void loop(){
  int v1 = digitalRead(PIN1);
  int v2 = digitalRead(PIN2);
  int v3 = digitalRead(PIN3);
  Serial.print(v1);
  Serial.print("\t");
  Serial.print(v2);
  Serial.print("\t");
  Serial.print(v3);
  Serial.println();
  delay(200);
}
