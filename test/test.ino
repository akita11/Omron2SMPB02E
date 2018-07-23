#include "Omron2SMPB02E.h"

Omron2SMPB02E prs;
// Omron2SMPB02E prs(0); // in case of SDO=0 configuration

void setup() {
  prs.begin();
  Serial.begin(9600);

  prs.set_mode(MODE_NORMAL);
  delay(1000);
  Serial.println(prs.read_raw_temp(), HEX);
  Serial.println(prs.read_raw_pressure(), HEX);
  Serial.println(prs.read_reg(CTRL_MEAS), HEX);
  Serial.println(prs.read_reg(DEVICE_STAT), HEX);
  Serial.println(prs.read_reg(IIR_CNT), HEX);
  Serial.println("----");
}

void loop() {
}
