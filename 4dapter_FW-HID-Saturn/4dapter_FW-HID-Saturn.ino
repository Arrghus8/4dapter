
/*
 *  GNU GENERAL PUBLIC LICENSE
 *  Version 3, 29 June 2007
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *  
 */

#include "Gamepad.h"
#include "N64_Controller.h"

// ATT: 20 chars max (including NULL at the end) according to Arduino source code.
// Additionally serial number is used to differentiate arduino projects to have different button maps!
const char *gp_serial = "4DAPTER_SATURN";

#define N64MapJoyToMax  true  // 'true' to map value to DInput Max (-128 to +127), set to false to use controller value directly
#define N64JoyMax       80     // N64 Joystick Maximum Travel Range (0-127, typically between 75-85 on OEM controllers)
#define N64JoyDeadzone  3      // Deadzone to return 0, minimizes drift

#define SATURN_SELECT_PAUSE 20 // How many microseconds to wait after setting select lines? (2µs is enough according to the Saturn developer's manual)
                               // 20µs is a "safe" value that seems to work for original Saturn controllers and Retrobit wired controllers
//#define RETROBIT_WL          // Uncomment to support the Retro Bit 2.4GHz wireless controller (this will increase lag a lot)

N64Controller       n64_controller;
N64_status_packet   N64Data;
int8_t LeftX = 0;
int8_t LeftY = 0;

#define NES       0
#define SNES      1
#define SATURN    2

#define BUTTONS   0
#define AXES      1

#define UP        0x01
#define DOWN      0x02
#define LEFT      0x04
#define RIGHT     0x08

#define NTT_BIT   0x00
#define NODATA    0x00

void sendLatch();
void sendClock();
void sendState();
void SaturnRead1();
void SaturnRead2();
void SaturnRead3();
void SaturnRead4();

/* -------------------------------------------------------------------------
Saturn controller socket (looking face-on at the front of the socket):
___________________
/ 1 2 3 4 5 6 7 8 9 \
|___________________|

Saturn controller plug (looking face-on at the front of the controller plug):
___________________
/ 9 8 7 6 5 4 3 2 1 \
|___________________|

------------------------------------------------------------------------- */
//
// Wire it all up according to the following table:
//
// Contr. pin name     Contr. pin     Arduino Pin
// ----------------------------------------------
// VCC                                 VCC ()
// GND                                 GND ()
// NES/SNES-LATCH          3           2   (PD1)
// NES/SNES-CLOCK          2           3   (PD0)
// NES-DATA                4           5   (PC6)
// NES-DATAD4              5           9   (PB5)
// NES-DATAD3              6           8   (PB4)
// SNES-DATA               4           7   (PE6)
// SNES-DATAD2             5           TX  (PD2)
// SNES-DATAD3             6           RX  (PD3)
// N64-3.3V                1           N/A
// N64-DATA                2           10  (PB6)
// N64-GND                 3           GND ()
// Saturn-VCC              1           VCC ()
// Saturn-DATA1            2           A2  (PF5)
// Saturn-DATA0            3           A3  (PF4)
// Saturn-SELECT0          4           15  (PB1)
// Saturn-SELECT1          5           14  (PB3)
// Saturn-TL               6           6   (PD7)
// Saturn-DATA3            7           A0  (PF7)
// Saturn-DATA2            8           A1  (PF6)
// Saturn-GND              9           GND ()

/* Power Pad Number Guide
 * C/O: http://www.neshq.com/hardgen/powerpad.txt

+---------/          \-----------+
|                       SIDE B   |
|   __      __       __     __   |
|  /  \    /  \     /  \   /  \  |
| | 1  |  | 2  |   | 3  | | 4  | |
|  \__/    \__/     \__/   \__/  |
|   __      __       __     __   |
|  /  \    /  \     /  \   /  \  |
| | 5  |  | 6  |   | 7  | | 8  | |
|  \__/    \__/     \__/   \__/  |
|   __      __       __     __   |
|  /  \    /  \     /  \   /  \  |
| | 9  |  | 10 |   | 11 | | 12 | |
|  \__/    \__/     \__/   \__/  |
|                                |
+--------------------------------+

           __________
+---------/          \-----------+
|                       SIDE A   |
|           __       __          |
|          /  \     /  \         |
|         |B 3 |   |B 2 |        |
|          \__/     \__/         |
|   __      __       __     __   |
|  /  \    /  \     /  \   /  \  |
| |B 8 |  |R 7 |   |R 6 | |B 5 | |
|  \__/    \__/     \__/   \__/  |
|           __       __          |
|          /  \     /  \         |
|         |B 11|   |B 10|        |
|          \__/     \__/         |
|                                |
+--------------------------------+
*/

// Set up USB HID gamepads
Gamepad_ Gamepad[3];

// Controllers
uint32_t  controllerData[2][2] = {{0,0},{0,0}};
uint32_t  axisIndicator[32] = {0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
bool      nttActive = false;

// Saturn controller buttons
uint8_t SaturnButtons[2]     = {0,0};
uint8_t SaturnButtonsPrev[2] = {0,0};

uint32_t  dataMaskNES[8] =        {0x02,   // A
                                   0x01,   // B
                                   0x40,   // Start 
                                   0x80,   // Select
                                   UP,     // D-Up
                                   DOWN,   // D-Down
                                   LEFT,   // D-Left
                                   RIGHT   // D-Right
                                   }; 

// Power Pad D4
uint32_t  dataMaskPowerPadD4[8] = {0x08,    // PowerPad #4
                                   0x04,    // PowerPad #3
                                   0x800,   // PowerPad #12
                                   0x80,    // PowerPad #8
                                   NODATA,  // No Data
                                   NODATA,  // No Data
                                   NODATA,  // No Data
                                   NODATA   // No Data
                                   }; 

// Power Pad D3
uint32_t  dataMaskPowerPadD3[8] = {0x02,   // PowerPad #2
                                   0x01,   // PowerPad #1
                                   0x10,   // PowerPad #5
                                   0x100,  // PowerPad #9
                                   0x20,   // PowerPad #6
                                   0x200,  // PowerPad #10
                                   0x400,  // PowerPad #11
                                   0x40    // PowerPad #7
                                   }; 

uint32_t  dataMaskSNES[32] =      {0x01,    // B
                                   0x04,    // Y
                                   0x40,    // Start   
                                   0x80,    // Select
                                   UP,      // D-Up
                                   DOWN,    // D-Down
                                   LEFT,    // D-Left
                                   RIGHT,   // D-Right
                                   0x02,    // A
                                   0x08,    // X
                                   0x10,    // L
                                   0x20,    // R
                                   NODATA,  // SNES Control Bit
                                   NTT_BIT, // NTT Indicator Bit
                                   NODATA,  // SNES Control Bit
                                   NODATA,  // SNES Control Bit
                                   0x100,   // NTT 0
                                   0x200,   // NTT 1
                                   0x400,   // NTT 2
                                   0x800,   // NTT 3
                                   0x1000,  // NTT 4
                                   0x2000,  // NTT 5
                                   0x4000,  // NTT 6
                                   0x8000,  // NTT 7
                                   0x10000, // NTT 8
                                   0x20000, // NTT 9
                                   0x40000, // NTT *
                                   0x80000, // NTT #
                                   0x100000,// NTT .
                                   0x200000,// NTT C
                                   NODATA,  // NTT No Data
                                   0x800000,// NTT End Comms
                                   };

void setup()
{
  n64_controller.N64_init();

  // N64 Data pin setup
  DDRD  &= ~B00010000; // inputs
  PORTD |=  B00010000; // enable internal pull-ups

  // Setup NES / SNES latch and clock pins (2/3 or PD1/PD0)
  DDRD  |=  B00000011; // output
  PORTD &= ~B00000011; // low

  // Setup NES / SNES data pins (5/7 or PC6/PE6)
  DDRC  &= ~B01000000; // 5 to input
  PORTC |=  B01000000; // 5 enable internal pull-up
  DDRE  &= ~B01000000; // 7 to input
  PORTE |=  B01000000; // 7 enable internal pull-up

  // Setup NES PowerPad data pins (8/9 or PB4/PB5)
  DDRB  &= ~B00110000; // inputs
  PORTB |=  B00110000; // enable internal pull-ups

  // Setup Saturn data pins (A3/A2/A1/A0 or PF4-PF7)
  DDRF  &= ~B11110000; // inputs
  PORTF |=  B11110000; // enable internal pull-ups

  // Setup Saturn TL pin (6 or PD7)
  DDRD  &= ~B10000000; // input
  PORTD |=  B10000000; // enable internal pull-up

  // Set Saturn Select pins (15/14 or PB1/PB3)
  PORTB |=  B00001010; // outputs
  DDRB  |=  B00001010; // high

  delay(250);
}

void loop() 
{ 
  while(true)
  {
    //Saturn controller
    // Clear button data
    SaturnButtons[0]=0; SaturnButtons[1]=0;

    // Read all button and axes states
    SaturnRead3();
    SaturnRead2();
    SaturnRead1();
    SaturnRead4();

    // Invert the readings so a 1 means a pressed button
    SaturnButtons[0] = ~SaturnButtons[0]; SaturnButtons[1] = ~SaturnButtons[1];

    // Send data to USB if values have changed
    if (SaturnButtons[0] != SaturnButtonsPrev[0] || SaturnButtons[1] != SaturnButtonsPrev[1] )
    {
      Gamepad[1]._GamepadReport.buttons = SaturnButtons[1] | ((SaturnButtons[0] & 0x80)<<1);
      
      if      ((SaturnButtons[0] & DOWN) >> 1)  Gamepad[1]._GamepadReport.Y = 0x7F;
      else if ((SaturnButtons[0] & UP)       )  Gamepad[1]._GamepadReport.Y = 0x80;
      else    Gamepad[1]._GamepadReport.Y = 0;

      if      ((SaturnButtons[0] & RIGHT) >> 3)  Gamepad[1]._GamepadReport.X = 0x7F;
      else if ((SaturnButtons[0] & LEFT ) >> 2)  Gamepad[1]._GamepadReport.X = 0x80;
      else    Gamepad[1]._GamepadReport.X = 0;

      SaturnButtonsPrev[0] = SaturnButtons[0];
      SaturnButtonsPrev[1] = SaturnButtons[1];
    }

    #ifdef RETROBIT_WL
      // This delay is needed for the retro bit 2.4GHz wireless controller, making it more or less useless with this adapter
      delay(17);
    #endif

    for(uint8_t j = 0; j < 1; j++)
    {
      sendLatch();

      controllerData[NES][BUTTONS] = 0;
      controllerData[NES][AXES] = 0;
      
      controllerData[SNES][BUTTONS] = 0;
      controllerData[SNES][AXES] = 0;

      nttActive = false;
  
      for(uint8_t dataBitCounter = 0; dataBitCounter < 32; dataBitCounter++)
      {
        // If no NTT controller, end the loop early
        if(!nttActive && dataBitCounter > 13)
        {
          break;
        }

        //NES Power Pad Controller
        if((dataBitCounter < 8) && ((PINB & B00100000) == 0)) //Power Pad Pin D4 (bottom)
        { 
          controllerData[NES][BUTTONS] |= dataMaskPowerPadD4[dataBitCounter];
        }

        if((dataBitCounter < 8) && ((PINB & B00010000) == 0)) //Power Pad Pin D3 (middle)
        { 
          controllerData[NES][BUTTONS] |= dataMaskPowerPadD3[dataBitCounter];
        }

        // NES Controller
        if((dataBitCounter < 8) && ((PINC & B01000000) == 0)) //If NES data line is low (indicating a press)
        { 
          if(axisIndicator[dataBitCounter])
          {
            controllerData[NES][AXES] |= dataMaskNES[dataBitCounter];
          }
          else
          {
            controllerData[NES][BUTTONS] |= dataMaskNES[dataBitCounter];
          }
        }

        // SNES / NTT Controller 
        if((PINE & B01000000) == 0) //If SNES data line is low (indicating a press)
        {
          if(dataBitCounter == 13)
          {
            nttActive = true;
          }
          
          if(axisIndicator[dataBitCounter])
          {
            controllerData[SNES][AXES] |= dataMaskSNES[dataBitCounter];
          }
          else
          {
            controllerData[SNES][BUTTONS] |= dataMaskSNES[dataBitCounter];
          }
        }
        
        sendClock();
      }
  
      Gamepad[0]._GamepadReport.buttons = controllerData[NES][BUTTONS] | controllerData[SNES][BUTTONS];
      
      if      ( ((controllerData[NES][AXES] & DOWN) >> 1) | ((controllerData[SNES][AXES] & DOWN) >> 1))  Gamepad[0]._GamepadReport.Y = 0x7F;
      else if (  (controllerData[NES][AXES] & UP  )       |  (controllerData[SNES][AXES] & UP  )      )  Gamepad[0]._GamepadReport.Y = 0x80;
      else    Gamepad[0]._GamepadReport.Y = 0;

      if      ( ((controllerData[NES][AXES] & RIGHT) >> 3) | ((controllerData[SNES][AXES] & RIGHT) >> 3))  Gamepad[0]._GamepadReport.X = 0x7F;
      else if ( ((controllerData[NES][AXES] & LEFT ) >> 2) | ((controllerData[SNES][AXES] & LEFT ) >> 2))  Gamepad[0]._GamepadReport.X = 0x80;
      else    Gamepad[0]._GamepadReport.X = 0;
      
      n64_controller.getN64Packet();
      N64Data = n64_controller.N64_status;

      Gamepad[2]._GamepadReport.X = 0;
      Gamepad[2]._GamepadReport.Y = 0;
      Gamepad[2]._GamepadReport.buttons = 0;
      
      if(N64Data.stick_x >= -N64JoyDeadzone && N64Data.stick_x <= N64JoyDeadzone)
      {
        LeftX = 0;
      }
      else
      {
        if(N64MapJoyToMax)
        {
          if(N64Data.stick_x > N64JoyMax)   N64Data.stick_x = N64JoyMax;
          if(N64Data.stick_x < -N64JoyMax)  N64Data.stick_x = -N64JoyMax;
          LeftX = map(N64Data.stick_x, -N64JoyMax, N64JoyMax, -128, 127);
        }
        else
        {
          LeftX = (int8_t)N64Data.stick_x;
        }
      }

      if(N64Data.stick_y >= -N64JoyDeadzone && N64Data.stick_y <= N64JoyDeadzone)
      {
        LeftY = 0;
      }
      else
      {
        if(N64MapJoyToMax)
        {
          if(N64Data.stick_y > N64JoyMax)   N64Data.stick_y = N64JoyMax;
          if(N64Data.stick_y < -N64JoyMax)  N64Data.stick_y = -N64JoyMax;
          LeftY = map(-N64Data.stick_y, -N64JoyMax, N64JoyMax, -128, 127); 
        }
        else
        {
          LeftY = (int8_t) -N64Data.stick_y;
        }
      }


      Gamepad[2]._GamepadReport.buttons |= (N64Data.data2 & 0x20 ? 1:0) << 4;  // L 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data2 & 0x10 ? 1:0) << 5;  // R
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data2 & 0x08 ? 1:0) << 13; // C-Uup
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data2 & 0x04 ? 1:0) << 3;  // C-Down 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data2 & 0x02 ? 1:0) << 2;  // C-Left 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data2 & 0x01 ? 1:0) << 6;  // C-Right

      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x80 ? 1:0) << 1;  // A 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x40 ? 1:0) << 0;  // B   
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x20 ? 1:0) << 8;  // Z
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x10 ? 1:0) << 7;  // Start 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x08 ? 1:0) << 9;  // D-Up 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x04 ? 1:0) << 10; // D-Down 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x02 ? 1:0) << 11; // D-Left 
      Gamepad[2]._GamepadReport.buttons |= (N64Data.data1 & 0x01 ? 1:0) << 12; // D-Right


      Gamepad[2]._GamepadReport.buttons &= 0x0000FFFF;
      
      Gamepad[2]._GamepadReport.X = LeftX;
      Gamepad[2]._GamepadReport.Y = LeftY;
    }    

  sendState();
 }
}

// Read R, X, Y, Z
void SaturnRead1()
{
  PORTB &= ~B00001010; // Set select outputs to 00
  delayMicroseconds(SATURN_SELECT_PAUSE);
  SaturnButtons[1] |= (PINF & 0xf0);
}
  
// Read ST, A, C, B
void SaturnRead2()
{
  PORTB ^= B00001010; // Toggle select outputs (01->10 or 10->01)
  delayMicroseconds(SATURN_SELECT_PAUSE);
  SaturnButtons[1] |= (PINF & 0xf0)>>4;
}

// Read DR, DL, DD, DU  
void SaturnRead3()
{
  PORTB ^= B00000010; // Set select outputs to 10 from 11 (toggle)
  delayMicroseconds(SATURN_SELECT_PAUSE);
  SaturnButtons[0] |= (PINF & 0xf0) >> 4;
}
  
// Read L, *, *, *
void SaturnRead4()
{
  PORTB |= B00001010; // Set select outputs to 11
  delayMicroseconds(SATURN_SELECT_PAUSE);
  SaturnButtons[0] |= (PINF & 0xf0);
}

void sendLatch()
{
  // Send a latch pulse to NES/SNES
  PORTD |=  B00000010; // Set HIGH
  __builtin_avr_delay_cycles(192);
  PORTD &= ~B00000010; // Set LOW
  __builtin_avr_delay_cycles(72);
}

void sendClock()
{
  // Send a clock pulse to NES/SNES
  PORTD |=  B00000001; // Set HIGH
  __builtin_avr_delay_cycles(96);
  PORTD &= ~B00000001; // Set LOW
  __builtin_avr_delay_cycles(72);
}

void sendState()
{
  Gamepad[0].send();
  Gamepad[1].send();
  Gamepad[2].send();
  __builtin_avr_delay_cycles(16000);
}
