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
 * This library supports CAN messages for MY08 Range Rover L322 for driving dash gauges, putting out malf lights, etc
 */

#ifndef JLR_L322_h
#define JLR_L322_h

#include <stdint.h>
#include "vehicle.h"

class JLR_L322: public Vehicle
{

public:
   void SetCanInterface(CanHardware* c);
   void Task10Ms();
   void Task100Ms();
   void SetRevCounter(int s) { speed = s; }
   void SetTemperatureGauge(float temp);  // Not implemented in JLR_L322 yet
   void DecodeCAN(int id, uint32_t* data);
   bool Ready();
   bool Start();
   // void SetE46(bool e46) { isE46 = e46; }

private:
   void Msg0979BAB0(int8_t gear);   // TCM(0x43F)
   void Msg17EC0010(); // TCM
   void Msg17C404B0(); // TCM
   void Msg17BDFFE0();  // ECM
   void Msg17E49220();  // ECM
   void Msg17D80420();  // ECM
   void Msg17E80420();  // ECM

   uint16_t speed;
   bool BrakeOn;
   bool AbsCANalive;
   bool SendCAN;
};

#endif /* Can_L322_h */

