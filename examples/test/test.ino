#include "Omron2SMPB02E.h"

Omron2SMPB02E prs;
// Omron2SMPB02E prs(0); // in case of SDO=0 configuration

void setup() {
  prs.begin();
  Serial.begin(9600);
  prs.set_mode(MODE_NORMAL);
  delay(1000);
  BigNumber tmp = prs.read_temp();
  Serial.println("temperature [degC]");
  Serial.println(tmp);
  Serial.println((float)tmp);
  BigNumber pressure = prs.read_pressure();
  Serial.println("pressure [Pa]");
  Serial.println(pressure);
  Serial.println((float)pressure);
}

void loop() {
}

