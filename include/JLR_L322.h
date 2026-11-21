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

#ifndef JLR_L322_h
#define JLR_L322_h

#include "vehicle.h"
#include <stdint.h>

class JLR_L322 : public Vehicle {

public:
  void SetCanInterface(CanHardware *c);
  void Task10Ms();
  void Task100Ms();
  void SetRevCounter(int s) { speed = s; }
  void SetTemperatureGauge(float temp);
  void DecodeCAN(int id, uint32_t *data);
  bool Ready();
  bool Start();
  // void SetE46(bool e46) { isE46 = e46; }

private:
  enum Lever { LEVER_P, LEVER_R, LEVER_N, LEVER_D };
  Lever GetLever();
  void Msg0979BAB0(); // TCM
  void Msg17EC0010(); // TCM
  void Msg17C404B0(); // TCM
  void Msg17BDFFE0(); // ECM
  void Msg17E49220(); // ECM
  void Msg17D80420(); // ECM
  void Msg17E80420(); // ECM
  void Msg071C0AA0(); // ECM
  void Msg17A9FFE0(); // ECM

  uint16_t speed;
  // Set in DecodeCAN, which runs in the CAN receive interrupt
  volatile bool BrakeOn;
  volatile uint8_t TerrainMode = 0; // requested by the Terrain Response switch
  bool SendCAN = false;
  // Frames keep going for this long after key-off (in 100 ms steps)
  static const uint8_t RUN_ON_100MS = 20;
  uint8_t runOnCount = 0;
  uint8_t tick = 0;       // 10 ms step counter for the frame schedule (0-119)
  int16_t GaugeTemp = 20; // dash temperature gauge, C
  // Below this in Run, the DC-DC is taken as not charging (battery lamp on)
  static constexpr float DCDC_LOW_V = 12.8f;
  uint8_t dcdcLowCount = 0;
};

#endif /* Can_L322_h */
