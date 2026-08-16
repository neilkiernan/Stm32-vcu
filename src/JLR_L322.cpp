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

#include "JLR_L322.h"
#include "stm32_can.h"
#include "utils.h"
#include "digio.h"
#include "my_math.h"

static uint16_t tempValue = 0;   // Not implemented in JLR_L322 yet

void JLR_L322::Task10Ms()
{
    // Review L322 CAN message frequency requirements - gear CAN to Task100Ms()?
    if(SendCAN == true)
    {
        Msg17EC0010();  // TCM: ? Blanked to test if Transfer Case stops activating
        Msg17C404B0();  // TCM: ? Blanked to test if Transfer Case stops activating
        Msg17BDFFE0();  // ECM: Dash RPM gauge. Only transmit CAN when in Run mode
        Msg17E49220();  // ECM: Dash battery red fault lamp, engine system fault message
        Msg17D80420();  // ECM: Dash engine orange fault lamp
        Msg17E80420();  // ECM: Dash glow plug orange lamp

        // if (Param::GetBool(Param::Transmission))     
        Msg0979BAB0(Param::GetInt(Param::dir));  // TCM: Gear selection (0x43F)
    }
}

void JLR_L322::Task100Ms()
{
    if(AbsCANalive == true || DigIo::t15_digi.Get())    // Check if 100ms if ABS can frame 0x0BD5FDF0 (0x1F3) has been recieved to say CAN is on
    {
        SendCAN = true;
    }
    else
    {
        SendCAN = false;
    }
    AbsCANalive = false;    // Reset flag for next check
}

void JLR_L322::SetCanInterface(CanHardware* c)
{
    can = c;
    // can->RegisterUserMessage(0x153);    // E39/E46 ASC1 message. Carry over from DM BMW E39 code for now
    can->RegisterUserMessage(0x0BD5FDF0);    // JLR L322 ABS CAN
}

void JLR_L322::SetTemperatureGauge(float temp)  // Not implemented in JLR_L322 yet
{
    tempValue = utils::change(temp,15,80,88,254);   // Map to e39 temp gauge. Carry over from DM BMW E39 for now
}

bool JLR_L322::Ready()
{
    return DigIo::t15_digi.Get();
}

bool JLR_L322::Start()
{
    return Param::GetBool(Param::din_start);
}

// These messages go out on vehicle CAN and are specific to driving the JLR L322 instrument cluster

//////////////////// JLR L322 CAN Messages ////////////////////
void JLR_L322::Msg17EC0010()    // TCM: ?
{
    uint8_t bytes[3];

    bytes[0]=0x03;
    bytes[1]=0xE0;
    bytes[2]=0x23;

    can->Send(0x17EC0010, (uint32_t*)bytes,3);
}

void JLR_L322::Msg17C404B0()    // TCM: ?
{
    uint8_t bytes[7];

    bytes[0]=0x10;
    bytes[1]=0x32;
    bytes[2]=0x00;
    bytes[3]=0x0B;
    bytes[4]=0xFF;
    bytes[5]=0xC0;
    bytes[6]=0x00;
    
    can->Send(0x17C404B0, (uint32_t*)bytes,7);
}

void JLR_L322::Msg17BDFFE0()    // ECM: Dash RPM gauge. Only transmit CAN when in Run mode
{
    // Dash RPM (Decimal) = 8274 + (speed_input * 0.96)

    int opmode = Param::GetInt(Param::opmode);
    uint16_t speed_input = speed;

    // Limit tachometer range from 750 RPMs - 6000 RPMs at max
    // These limits ensure the vehicle thinks engine is alive and within the max allowable RPM

    speed_input = MAX(750, speed_input);
    speed_input = MIN(6000, speed_input);
    speed_input = (((speed_input * 480)/500) + 8274);
    
    uint8_t bytes[5];

    bytes[0]=0x40;
    bytes[1]=0x00;
    bytes[2]=(speed_input >> 8) & 0xFF;  // RPM MSB
    bytes[3]=(speed_input) & 0xFF;  // RPM LSB
    bytes[4]=0x07;

    if(opmode==MOD_RUN)  // Only transmit CAN when in Run mode
    {
      can->Send(0x17BDFFE0, bytes, 5);
    }
    
}

void JLR_L322::Msg17E49220()    // ECM: Dash battery red fault light, engine system fault message
{
    // Turn on if 12V battery supply (uaux) drops below 11.6V
    
    float battV = Param::GetFloat(Param::uaux);
    uint8_t bytes[8];

    bytes[0]=0x00;
    bytes[1]=0x00;
    if(battV < 11.6)  // Dash battery red fault light (0x00 Off, 0x80 On)
    {
        bytes[2] = 0x80;
    }
    else{
        bytes[2] = 0x00;
    }
    bytes[3]=0x00;
    bytes[4]=0x19;
    bytes[5]=0xC8;
    bytes[6]=0x00;
    bytes[7]=0x79;

    can->Send(0x17E49220, bytes, 8);
}

void JLR_L322::Msg17D80420()    // ECM: Dash engine orange fault lamp 
{
    // Turns on at ignition on, stays on for pre-charge, turns off on successful pre-charge when Opmode is Run

    int opmode = Param::GetInt(Param::opmode);
    uint8_t bytes[6];

    bytes[0]=0x3C;
    bytes[1]=0x7D;
    bytes[2]=0x40;
    bytes[3]=0xFB;
    bytes[4]=0xFF;
    if(opmode==MOD_RUN)  // Dash engine orange light (0x80 Off, 0xA0 On)
    {
      bytes[5]=0x80;
    }

   else
   {
     bytes[5]=0xA0;
   }

    can->Send(0x17D80420, (uint32_t*)bytes,6);
}

void JLR_L322::Msg17E80420()    // ECM: Dash glow plug orange lamp
{
    // Turned off permanently

    uint8_t bytes[6];

    bytes[0]=0x3F;
    bytes[1]=0x00;
    bytes[2]=0x00;
    bytes[3]=0x28;
    bytes[4]=0x00;
    bytes[5]=0x00;  // Dash glow plug orange light (0x00 Off, 0x80 On)

    can->Send(0x17E80420, bytes, 6);
}


// Swap in L322 TCM CAN data here for gear selection. How to accommodate Park selection also? GS450H gear selector
// provides a 12V signal when in Park, integrate as an input? 
void JLR_L322::Msg0979BAB0(int8_t gear)  // TCM: Gear selection (0x43F)
{
    uint8_t bytes[8];

    switch (gear)
    {
    case -1:    // Reverse
        bytes[0] = 0x87;
        bytes[1] = 0x80;
        bytes[2] = 0x00;
        bytes[4] = 0x1F;
        bytes[6] = 0xFF;
        bytes[7] = 0x0C;

        if(BrakeOn){
            bytes[3] = 0x87;
            bytes[5] = 0xF7;
        }else{
            bytes[3] = 0xC7;
            bytes[5] = 0x77;      
        }
        break;
    case 0:   // Neutral
        bytes[0] = 0x80;
        bytes[1] = 0x80;
        bytes[2] = 0x00;
        bytes[4] = 0x1F;
        bytes[6] = 0xFF;
        bytes[7] = 0x0C;

        if(BrakeOn){
            bytes[3] = 0x87;
            bytes[5] = 0xF7;
        }else{
            bytes[3] = 0xC7;
            bytes[5] = 0x77;    
        }
        break;
    case 1: // Drive
        bytes[0] = 0x89;
        bytes[1] = 0x80;
        bytes[2] = 0x00;
        bytes[4] = 0x1F;
        bytes[6] = 0xFF;
        bytes[7] = 0x0C;

        if(BrakeOn){
            bytes[3] = 0x87;
            bytes[5] = 0xF7;
        }else{
            bytes[3] = 0xC7;
            bytes[5] = 0x77;     
        }
        break;
    case 2:  // Park
        bytes[0] = 0x88;
        bytes[1] = 0x80;
        bytes[2] = 0x00;
        bytes[4] = 0x1F;
        bytes[6] = 0xFF;
        bytes[7] = 0x0C;

        if(BrakeOn){
            bytes[3] = 0x87;
            bytes[5] = 0xF7;
        }else{
            bytes[3] = 0xC7;
            bytes[5] = 0x77;      
        }
        break;
    case 3: // Sport
        bytes[0] = 0x89;
        bytes[1] = 0x80;
        bytes[2] = 0x00;
        bytes[4] = 0x1F;
        bytes[6] = 0xFF;
        bytes[7] = 0x3C;

        if(BrakeOn){
            bytes[3] = 0x87;
            bytes[5] = 0xF7;
        }else{
            bytes[3] = 0xB7;
            bytes[5] = 0x77;      
        }
        break;
    default:
        bytes[0] = 0x00;
        bytes[1] = 0x00;
        bytes[2] = 0x00;
        bytes[3] = 0x00;
        bytes[4] = 0x00;
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;
        break;
    }

    can->Send(0x0979BAB0, bytes, 8);
}

 
void JLR_L322::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;

    if (id == 0x0BD5FDF0)   // L322 ABS CAN. Contains brake status
    {
        // Dual purpose. ABS CAN comes alive with ignition on. This ABS CAN ID also contains brake state info
        // Modify TCM Gear CAN accordingly       

        if(bytes[6] & 1){  // To be tested. Brake pedal pressed 11101111 (off 11101110)
            BrakeOn = true;   
        }else{ 
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

        AbsCANalive = true;
    }
}   

