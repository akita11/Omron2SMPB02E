#include <Arduino.h>
#include <Omron2SMPB02E.h>
#include <Wire.h>

uint8_t Omron2SMPB02E::read_reg(uint8_t addr)
{
  uint8_t d;
  Wire.beginTransmission(i2c_addr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom((int)i2c_addr, 1);
  d = Wire.read();
  Wire.endTransmission();
  return(d);
}

void Omron2SMPB02E::write_reg(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

// read {(@addr):(@addr+1)}
uint16_t Omron2SMPB02E::read_reg16(uint8_t addr)
{
  return((read_reg(addr) << 8) | (read_reg(addr + 1)));
}

//Omron2SMPB02E(uint8_t SDO = 1)
Omron2SMPB02E::Omron2SMPB02E()
{
  //  if (SDO == 0) i2c_addr = 0x70;
  i2c_addr = 0x70;
}

Omron2SMPB02E::~Omron2SMPB02E()
{
}

float Omron2SMPB02E::conv_K0(uint32_t x, float a, float s)
{
  return(a + (s * (float)x) / 32767.0);
}

float Omron2SMPB02E::conv_K1(uint16_t x)
{
  return((float)x / 16.0);
}

void Omron2SMPB02E::begin()
{
  Wire.begin();
  write_reg(IO_SETUP, 0x00); // IO_SETUP
  uint8_t coe_b00_a0_ex = read_reg(COE_b00_a0_ex);
  a0 = conv_K1((read_reg16(COE_a0) << 4) | (coe_b00_a0_ex & 0x0f));
  b00 = conv_K1((read_reg16(COE_b00) << 4) | (coe_b00_a0_ex > 4));
  a1 = conv_K0(read_reg16(COE_a1), -6.3e-3, 4.3e-4);
  a2 = conv_K0(read_reg16(COE_a2), -1.9e-11, 1.2e-10);
  bt1 = conv_K0(read_reg16(COE_bt1), 1.0e-1, 9.1e-2);
  bt2 = conv_K0(read_reg16(COE_bt2), 1.2e-8, 1.2e-6);
  bp1 = conv_K0(read_reg16(COE_bp1), 3.3e-2, 1.9e-2);
  b11 = conv_K0(read_reg16(COE_b11), 2.1e-7, 1.4e-7);
  bp2 = conv_K0(read_reg16(COE_bp2), -6.3e-10, 3.5e-10);
  b12 = conv_K0(read_reg16(COE_b12), 2.9e-13, 7.6e-13);
  b21 = conv_K0(read_reg16(COE_b21), 2.1e-15, 1.2e-14);
  bp3 = conv_K0(read_reg16(COE_bp3), 1.3e-16, 7.9e-17);
}

uint8_t Omron2SMPB02E::read_id()
{
  return(read_reg(CHIP_ID)); // CHIP_ID, would be 0x5c
}

void Omron2SMPB02E::reset()
{
  write_reg(RESET, 0xe6); // software reset
}

long Omron2SMPB02E::read_raw_temp()
{
  return(((read_reg(TEMP_TXD2) << 16)
	  | (read_reg(TEMP_TXD1) <<  8)
	  | (read_reg(TEMP_TXD2)      )) - (1 << 23));
}

long Omron2SMPB02E::read_raw_pressure()
{
  return(
	 ((read_reg(PRESS_TXD2) << 16)
	  | (read_reg(PRESS_TXD1) <<  8)
	  | (read_reg(PRESS_TXD2)      )) - (1 << 23));
}

// read temperature in [degC]
float Omron2SMPB02E::read_temp()
{
  return(read_calc_temp() / 256.0);
}

float Omron2SMPB02E::read_calc_temp()
{
  // Tr = a0 + a1 * Dt + a2 * Dt^2 
  // -> temp = Re / 256 [degC]
  //   Dt : raw temperature value from TEMP_TXDx reg.

  //   a0, b00 : OTP / 16
  //     {19:12}   {11:4}    {3:0}
  // a0  COE_a0_1  COE_a0_0  COE_a0_ex
  // b00 COE_b00_1 COE_b00_0 COE_b00_ex

  //   a1, ... : A + (S * OTP) / 32767
  //        A        S        OTP
  // a1    -6e-03    4.3e-04  [COE_a1_1,COE_a1_0]
  // a2    -1.9e-11  1.2e-10  [COE_a2_1,COE_a2_0]
  // bt1   1.0e-01   9.1e-02  [COE_bt1_1,COE_bt1_0]
  // bt2   1.2e-8    1.2e-06  [COE_bt2_1,COE_bt2_0]
  // bp1   3.3e-02   1.9e-02  [COE_bp1_1,COE_bp1_0]
  // b11   2.1e-07   1.4e-07  [COE_b11_1,COE_b11_0]
  // bp2   -6.3e-10  3.5e-10  [COE_bp2_1,COE_bp2_0]
  // b12   2.9e-13   7.6e-13  [COE_b12_1,COE_b12_0]
  // b21   2.1e-15   1.2e-14  [COE_b21_1,COE_b21_0]
  // bp3   1.3e-16   7.9e-17  [COE_bp3_1,COE_bp3_0]

  float temp;
  long dt;
  dt = read_raw_temp();
  //  temp = a0 + a1 * dt + a2 * dt * dt;
  temp = a0 + (a1 + a2 * dt) * dt;
  return(temp / 256.0);
}

// read pressure in [Pa]
float Omron2SMPB02E::read_pressure()
{
  // Pr = b00 + (bt1 * Tr) + (bp1 * Dp) + (b11 * Dp * Tr) + (bt2 * Tr-2)
  //      + (bp2 * Dp^2) + (b12 * Dp * Tr^2) + (b21 * Dp^2 * Tr) + (bp3 * Dp^3)
  //   Tr : raw temperature from TEMP_TXDx reg.
  //   Dp : raw pressure from PRESS_TXDx reg.
  float pressure;
  long tr, dp;
  tr = read_calc_temp();
  dp = read_raw_pressure();
  //  pressure = b00 + bt1 * tr + bp1 * dp + b11 * dp * tr
  //    + bt2 * tr * tr + bp2 * dp * dp + b12 * dp * tr * tr
  //    + b21 * dp * dp * tr + bp3 * dp * dp * dp;
  pressure = b00
    + tr * (bt1 + b11 * bp1 + bt2 * tr)
    + dp * (bp1 + b12 * tr * tr
	    + dp * (bp2 + b21 * tr + bp3 * dp));
  return(pressure);
}

void Omron2SMPB02E::set_averate(uint8_t temp_avg, uint8_t pressure_avg)
{
  uint8_t r = read_reg(CTRL_MEAS) & 0x03;
  r = r | (temp_avg << 5);
  r = r | (pressure_avg << 2);
  write_reg(CTRL_MEAS, r);
}

void Omron2SMPB02E::set_mode(uint8_t mode)
{
  uint8_t r = read_reg(CTRL_MEAS) & 0xfc;
  r = r | mode;
  write_reg(CTRL_MEAS, r);
}

uint8_t Omron2SMPB02E::is_busy()
{
  if ((read_reg(DEVICE_STAT) & 0x08) == 0) return(0);
  else return(1); // busy
}

void Omron2SMPB02E::set_filter(uint8_t mode)
{
  write_reg(IIR_CNT, mode);
}

