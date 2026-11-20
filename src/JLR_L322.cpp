/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2021 Damien Maguire
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This library supports CAN messages for MY08 Range Rover L322 for driving dash
 * gauges, putting out malf lights, etc
 */

#include "JLR_L322.h"
#include "digio.h"
#include "my_math.h"
#include "stm32_can.h"
#include "utils.h"

static uint16_t tempValue = 0; // Not implemented in JLR_L322 yet

void JLR_L322::Task10Ms() {
  // Each frame goes out at the period the real ECM/TCM uses (2008 3.6 TDV8
  // captures). tick counts 10 ms steps over 1.2 s (the common multiple of 400
  // and 600 ms). Even ticks are kept for the three 20 ms frames, and the
  // slower frames are spread over odd ticks, so no more than 3 frames are
  // queued in one tick. The CAN driver has 3 hardware mailboxes and
  // a small software buffer that drops frames when full, so this keeps
  // everything in the mailboxes.
  if (SendCAN) {
    if ((tick % 2) == 0) {
      Msg0979BAB0(); // TCM: selector position, 20 ms
      Msg071C0AA0(); // ECM: torques, 20 ms
      Msg17A9FFE0(); // ECM: pedal and torques, 20 ms (real ECM ~24 ms)
    }
    if ((tick % 6) == 1)
      Msg17BDFFE0(); // ECM: RPM, 60 ms
    if ((tick % 6) == 3)
      Msg17C404B0(); // TCM: 60 ms
    if ((tick % 6) == 5)
      Msg17D80420(); // ECM: engine lamp, 60 ms
    if ((tick % 20) == 7)
      Msg17E49220(); // ECM: battery lamp, 200 ms
    if ((tick % 40) == 9)
      Msg17EC0010(); // TCM: 400 ms
    if ((tick % 60) == 11)
      Msg17E80420(); // ECM: glow plug lamp, 600 ms
  }

  tick++;
  if (tick >= 120)
    tick = 0;
}

void JLR_L322::Task100Ms() {
  // Send the ECM/TCM frames while the ignition (T15) is on, plus a short
  // run-on after key-off. This used to follow the ABS frame as well, which
  // could keep the network awake after key-off: our frames keep the ABS awake,
  // and the ABS frame kept our frames going.
  if (DigIo::t15_digi.Get())
    runOnCount = RUN_ON_100MS;
  else if (runOnCount > 0)
    runOnCount--;

  SendCAN = (runOnCount > 0);
}

void JLR_L322::SetCanInterface(CanHardware *c) {
  can = c;
  // can->RegisterUserMessage(0x153);    // E39/E46 ASC1 message. Carry over
  // from DM BMW E39 code for now
  can->RegisterUserMessage(0x0BD5FDF0); // JLR L322 ABS CAN
  can->RegisterUserMessage(0x17E012B0); // Terrain Response switch
}

void JLR_L322::SetTemperatureGauge(
    float temp) // Not implemented in JLR_L322 yet
{
  tempValue = utils::change(
      temp, 15, 80, 88,
      254); // Map to e39 temp gauge. Carry over from DM BMW E39 for now
}

bool JLR_L322::Ready() { return DigIo::t15_digi.Get(); }

bool JLR_L322::Start() { return Param::GetBool(Param::din_start); }

// These messages go out on vehicle CAN and are specific to driving the JLR L322
// instrument cluster

//////////////////// JLR L322 CAN Messages ////////////////////
void JLR_L322::Msg17EC0010() // TCM: ?
{
  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  bytes[0] = 0x03;
  bytes[1] = 0xE0;
  bytes[2] = 0x23;

  can->Send(0x17EC0010, data, 3);
}

void JLR_L322::Msg17C404B0() // TCM: gear state and output shaft speed
{
  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  // Byte 0 follows the lever: 0x10 in P/R/N, 0x38 in D (3.6 TCM captures)
  bytes[0] = (GetLever() == LEVER_D) ? 0x38 : 0x10;
  bytes[1] = 0x32;
  bytes[2] = 0x00;
  bytes[3] = 0x0B;
  bytes[4] = 0xFF;
  bytes[5] = 0xC0;
  bytes[6] = 0x00;

  can->Send(0x17C404B0, data, 7);
}

void JLR_L322::Msg17BDFFE0() // ECM: Dash RPM gauge
{
  // Dash RPM (Decimal) = 8274 + (speed_input * 0.96)

  // Sent from ignition-on, as the real ECM does. Before Run it reports
  // 0 rpm (raw 0x2000, as captured from the 3.6 ECM with the ignition on
  // and the engine stopped), so the other modules see the ECM present.

  int opmode = Param::GetInt(Param::opmode);
  uint16_t speed_input = 0x2000; // 0 rpm, engine not running

  if (opmode == MOD_RUN) {
    // Limit tachometer range from 750 RPMs - 6000 RPMs at max
    // These limits ensure the vehicle thinks engine is alive and within the
    // max allowable RPM
    speed_input = speed;
    speed_input = MAX(750, speed_input);
    speed_input = MIN(6000, speed_input);
    speed_input = (((speed_input * 480) / 500) + 8274);
  }

  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  bytes[0] = 0x40;
  bytes[1] = 0x00;
  bytes[2] = (speed_input >> 8) & 0xFF; // RPM MSB
  bytes[3] = (speed_input) & 0xFF;      // RPM LSB
  bytes[4] = 0x07;

  can->Send(0x17BDFFE0, data, 5);
}

void JLR_L322::Msg071C0AA0() // ECM: engine torques
{
  // Read by the ABS and transfer box. Without it and 17A9FFE0, the ABS
  // shows the same flags as with the real ECM unplugged (108CF1F0 byte 1,
  // 0BD5FDF0 bytes 2-3), the likely cause of the HDC fault.
  // Three 11-bit torque fields at bits 2, 18 and 34. 0x3FF = 0 Nm, as the
  // real ECM sends with the engine stopped. Static at 0 Nm until the scaling
  // is verified, because the ABS uses engine torque for traction control.
  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  bytes[0] = 0x03;
  bytes[1] = 0xFF;
  bytes[2] = 0x03;
  bytes[3] = 0xFF;
  bytes[4] = 0x33;
  bytes[5] = 0xFF;

  can->Send(0x071C0AA0, data, 6);
}

void JLR_L322::Msg17A9FFE0() // ECM: accelerator pedal and torques
{
  // Read by the ABS, gearbox and transfer box. Byte 3 = pedal x0.5 %, byte 0 =
  // virtual pedal, byte 4 bit 3 = kickdown, torque fields at bits 10, 34 and
  // 50. Static at 0 % pedal and 0 Nm, as captured from the 3.6 ECM at
  // ignition on with the engine stopped.
  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  bytes[0] = 0x00;
  bytes[1] = 0x03;
  bytes[2] = 0xFF;
  bytes[3] = 0x00;
  bytes[4] = 0xE3;
  bytes[5] = 0xFF;
  bytes[6] = 0xFB;
  bytes[7] = 0xFF;

  can->Send(0x17A9FFE0, data, 8);
}

void JLR_L322::Msg17E49220() // ECM: Dash battery red fault light, engine system
                             // fault message
{
  // Turn on if 12V battery supply (uaux) drops below 11.6V

  float battV = Param::GetFloat(Param::uaux);
  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  if (battV < 11.6) // Dash battery red fault light (0x00 Off, 0x80 On)
  {
    bytes[2] = 0x80;
  } else {
    bytes[2] = 0x00;
  }
  bytes[3] = 0x00;
  bytes[4] = 0x19;
  bytes[5] = 0xC8;
  bytes[6] = 0x00;
  bytes[7] = 0x79;

  can->Send(0x17E49220, data, 8);
}

void JLR_L322::Msg17D80420() // ECM: Dash engine orange fault lamp
{
  // Turns on at ignition on, stays on for pre-charge, turns off on successful
  // pre-charge when Opmode is Run

  int opmode = Param::GetInt(Param::opmode);
  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  bytes[0] = 0x3C;
  bytes[1] = 0x7D;
  bytes[2] = 0x40;
  bytes[3] = 0xFB;
  bytes[4] = 0xFF;
  if (opmode == MOD_RUN) // Dash engine orange light (0x80 Off, 0xA0 On)
  {
    bytes[5] = 0x80;
  }

  else {
    bytes[5] = 0xA0;
  }
  // Bits 2-0: Terrain Response mode echo (as the 3.6 ECM does)
  bytes[5] |= (TerrainMode & 0x07);

  can->Send(0x17D80420, data, 6);
}

void JLR_L322::Msg17E80420() // ECM: Dash glow plug orange lamp
{
  // Turned off permanently

  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;

  bytes[0] = 0x3F;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x28;
  bytes[4] = 0x00;
  bytes[5] = 0x00; // Dash glow plug orange light (0x00 Off, 0x80 On)

  can->Send(0x17E80420, data, 6);
}

// Selector lever position, read from the physical inputs rather than the
// VCU's accepted direction (dir), so the transfer box and dash always see
// where the lever really is. Park comes from the GS450H 12 V Park output on
// the GP 12 V input (din_12Vgp; leave GP12VInFunc at 0 so it has no other
// function). Forward and reverse are the normal direction inputs.
JLR_L322::Lever JLR_L322::GetLever() {
  bool fwd = Param::GetBool(Param::din_forward);
  bool rev = Param::GetBool(Param::din_reverse);

  if (Param::GetBool(Param::din_12Vgp))
    return LEVER_P;
  if (fwd && !rev)
    return LEVER_D;
  if (rev && !fwd)
    return LEVER_R;
  return LEVER_N;
}

void JLR_L322::Msg0979BAB0() // TCM: selector position and brake
{
  // Values from the 3.6 TCM (2021 captures). Byte 0 = lever position.
  // Bytes 3 and 5 follow the brake: pressed 0x87/0xF7, released 0xC7/0x77
  // (bit 6 of byte 3 set and bit 7 of byte 5 clear when released). Byte 7 =
  // 0x0C in P/R/N, 0x3C in D, 0x7C in R with the engine running.
  uint32_t data[2] = {0, 0}; // 8 bytes, word aligned, unused bytes zero
  uint8_t *bytes = (uint8_t *)data;
  Lever lever = GetLever();
  bool run = (Param::GetInt(Param::opmode) == MOD_RUN);

  bytes[1] = 0x80;
  bytes[2] = 0x00;
  bytes[3] = 0x87;
  bytes[4] = 0x1F;
  bytes[5] = 0xF7;
  bytes[6] = 0xFF;
  bytes[7] = 0x0C;

  switch (lever) {
  case LEVER_P:
    bytes[0] = 0x88;
    break;
  case LEVER_R:
    bytes[0] = 0x87;
    if (run)
      bytes[7] = 0x7C;
    break;
  case LEVER_N:
    bytes[0] = 0x80;
    break;
  case LEVER_D:
    bytes[0] = 0x89;
    bytes[3] = 0xB7;
    bytes[7] = 0x3C;
    break;
  }

  if (!BrakeOn) {
    bytes[3] |= 0x40; // brake released
    bytes[5] &= 0x7F;
  }

  can->Send(0x0979BAB0, data, 8);
}

void JLR_L322::DecodeCAN(int id, uint32_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  if (id == 0x0BD5FDF0) // L322 ABS CAN. Contains brake status
  {
    // Dual purpose. ABS CAN comes alive with ignition on. This ABS CAN ID also
    // contains brake state info Modify TCM Gear CAN accordingly

    if (bytes[6] &
        1) { // To be tested. Brake pedal pressed 11101111 (off 11101110)
      BrakeOn = true;
    } else {
      BrakeOn = false;
    }

    // Alternative method to check brake status, monitor brake input pin
    /*
     bool brake = Param::GetBool(Param::din_brake);

     if(brake){  // To be tested
         BrakeOn = true;
     }else{
         BrakeOn = false;
     }
     */
  }

  if (id == 0x17E012B0) // Terrain Response switch module
  {
    // Byte 1 low nibble = requested mode: 0 General, 1 Grass/Gravel/Snow,
    // 3 Sand, 4 Mud & Ruts, 5 Rock Crawl, 7 fail-safe. Every powertrain module
    // must echo the mode back; without the ECM echo the switch module flags
    // invalid ECM data (U0401) after about 1 s and goes to fail-safe. In
    // fail-safe the other modules return to General, so we do the same.
    uint8_t mode = bytes[1] & 0x0F;
    TerrainMode = (mode <= 5) ? mode : 0;
  }
}
