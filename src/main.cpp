#include <Arduino.h>

#include "control_system.h"

ControlSystem g_controlSystem;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== Thegiller-PT12 Boot ===");

  g_controlSystem.begin();
}

void loop() { g_controlSystem.loop(); }

