#include "../include/led.h"

led::led() {
}

void led::setup() {
  pinMode(LED_DEBUG_PIN, OUTPUT);
  blink();
}

void led::blink() {
  for (int i = 0; i < 3; i++) { 
    digitalWrite(LED_DEBUG_PIN, HIGH);
    delay(50);
    digitalWrite(LED_DEBUG_PIN, LOW);
    delay(50);
  }
}

void led::on() {
  analogWrite(LED_DEBUG_PIN, 10);
}

void led::off() {
  analogWrite(LED_DEBUG_PIN, 0);
}