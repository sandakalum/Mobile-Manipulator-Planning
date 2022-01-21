/**********************************************************************************
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005-2008 Neuronics AG
 *   Check out the AUTHORS file for detailed contact information.
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 **********************************************************************************/
//////////////////////////////////////////////////////////////////////////////////
// commands.cpp
// demo program for KNI libraries
//////////////////////////////////////////////////////////////////////////////////
#include "kniBase.h"
#include "../../include/KNI/kmlMotBase.h"
#include <iostream>
#include <cstdio>
#include <memory>
//////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
#include <conio.h>
#else  // LINUX
#include "keyboard.h"
#endif
//////////////////////////////////////////////////////////////////////////////////
// Katana object
std::auto_ptr< CLMBase > katana;
//////////////////////////////////////////////////////////////////////////////////
void DisplayHelp()
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------" << std::endl;
  std::cout << "Esc: quit program" << std::endl;
  std::cout << "?: Display this help" << std::endl;
  std::cout << "j: Calibrate the Katana" << std::endl;
  std::cout << "c: Set target position, freeze or switch motor off (C Command)" << std::endl;
  std::cout << "C: Set encoder offset (C Command (motNr. + 128))" << std::endl;
  std::cout << "d: Read the position, velocity and pwm duty cycle of a motor (D Command)" << std::endl;
  std::cout << "s: Set and read parameters and limits (S Command)" << std::endl;
  std::cout << "n: Get all motors command flags or positions at once (N Command)" << std::endl;
  std::cout << "g: Set the single polynomial movement parameters (G Command)" << std::endl;
  std::cout << "G: Start the single polynomial movement (G+128 Command)" << std::endl;
  std::cout << "a: Switch collision detection On and Off (A Command)" << std::endl;
  std::cout << "z: Read Echo (Z Command)" << std::endl;
  std::cout << "y: Read the Identification String (Y Command)" << std::endl;
  std::cout << "x: Read the Katana Command Table (X Command)" << std::endl;
  std::cout << "e: Read Gripper Sensor Data (E Command)" << std::endl;
  std::cout << "b: Read the Master firmware-version (B Command)" << std::endl;
  std::cout << "v: Read the Slave firmware-version (V Command)" << std::endl;
  std::cout << "i: Get motCommand from slave (I Command)" << std::endl;
  std::cout << "m: Modbus read and write (M Command)" << std::endl;
  std::cout << "t: Katana 1.2 I/O (T Command)" << std::endl;
  std::cout << "o: Navigation Control (O Command)" << std::endl;
  std::cout << "-------------------------------------------" << std::endl;
}
//////////////////////////////////////////////////////////////////////////////////
// convert two bytes (hi and lo) to short
short b2s(byte hi, byte lo)
{
  return (((short)hi) << 8) + ((short)lo);
}
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "usage for socketcommands: socketcommands CONFIGFILE"
              << " IP_ADDR" << std::endl;
    return 0;
  }

  std::cout << "---------------------------" << std::endl;
  std::cout << "SOCKETCOMMANDS DEMO STARTED" << std::endl;
  std::cout << "---------------------------" << std::endl;

  //----------------------------------------------------------------//
  // open device: a serial port is opened in this case
  //----------------------------------------------------------------//

  std::auto_ptr< CCdlSocket > device;
  std::auto_ptr< CCplSerialCRC > protocol;

  try
  {
    int port = 5566;
    device.reset(new CCdlSocket(argv[2], port));

    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "success:  port " << port << " open" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    //--------------------------------------------------------//
    // init protocol:
    //--------------------------------------------------------//

    protocol.reset(new CCplSerialCRC());
    protocol->init(device.get());
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "success: protocol initiated" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    //--------------------------------------------------------//
    // init robot:
    //--------------------------------------------------------//

    katana.reset(new CLMBase());
    katana->create(argv[1], protocol.get());
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
    return -1;
  }
  std::cout << "-------------------------------------------" << std::endl;
  std::cout << "success: katana initiated" << std::endl;
  std::cout << "-------------------------------------------" << std::endl;

  DisplayHelp();

  // declare variables used in loop
  byte packet[32];   // packet
  byte buffer[256];  // readbuf
  byte size = 0;     // readbuf size
  short param1, param2, param3, param4, param5, param6, param7;
  char p1, p2;
  int i;

  bool loop = true;

  while (loop)
  {
    int input = _getch();

    try
    {
      switch (input)
      {
        case 27:  // VK_ESCAPE
          loop = false;
          break;

        case '?':
          DisplayHelp();
          break;

        case 'j':  // VK_J (Calibration)
          std::cout << std::endl;
          std::cout << "Calibrating Katana... ";
          katana->calibrate();
          std::cout << "finished." << std::endl;
          break;

        case 'c':  // VK_C (C Command)
          std::cout << std::endl;
          std::cout << "Set target position of a motor, freeze or "
                    << "switch motors off (C Command)" << std::endl;

          // get parameters
          std::cout << " motor number: ";
          std::cin >> param1;
          if ((param1 < 1) || (param1 > 6))
          {
            break;
          }
          std::cout << " motor command flag: ";
          std::cin >> param2;
          std::cout << " targetposition: ";
          std::cin >> param3;

          // create packet
          packet[0] = 'C';
          packet[1] = (byte)param1;
          packet[2] = (byte)param2;
          packet[3] = (byte)(param3 >> 8);
          packet[4] = (byte)param3;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 3))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "," << (short)buffer[2] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'C':  // VK_C (C Command (motNr. + 128))
          std::cout << std::endl;
          std::cout << "Set encoder offset (C Command (motNr. + 128))" << std::endl;

          // get parameters
          std::cout << " motor number: ";
          std::cin >> param1;
          if ((param1 < 1) || (param1 > 6))
          {
            break;
          }
          param1 += 128;
          std::cout << " motor command flag: ";
          std::cin >> param2;
          std::cout << " offset position: ";
          std::cin >> param3;

          // create packet
          packet[0] = 'C';
          packet[1] = (byte)param1;
          packet[2] = (byte)param2;
          packet[3] = (byte)(param3 >> 8);
          packet[4] = (byte)param3;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 3))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] - 128 << "," << (short)buffer[2] << "<"
                      << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'd':  // VK_D (D Command)
          std::cout << std::endl;
          std::cout << "Read the position, velocity and pwm duty cycle "
                    << "of a motor (D Command)" << std::endl;

          // get parameters
          std::cout << " motor number: ";
          std::cin >> param1;
          if ((param1 < 1) || (param1 > 6))
          {
            break;
          }

          // create packet
          packet[0] = 'D';
          packet[1] = (byte)param1;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 8))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "," << (short)buffer[2] << ","
                      << b2s(buffer[3], buffer[4]) << "," << b2s(buffer[5], buffer[6]) << "," << (short)buffer[7] << "<"
                      << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 's':  // VK_S (S Command)
          std::cout << std::endl;
          std::cout << "Set and read parameters and limits (S Command)" << std::endl;

          // get parameters
          std::cout << " motor number: ";
          std::cin >> param1;
          if ((param1 < 1) || (param1 > 6))
          {
            break;
          }
          std::cout << " subcommand: ";
          std::cin >> param2;
          std::cout << " P1: ";
          std::cin >> param3;
          std::cout << " P2: ";
          std::cin >> param4;
          std::cout << " P3: ";
          std::cin >> param5;

          // create packet
          packet[0] = 'S';
          packet[1] = (byte)param1;
          packet[2] = (byte)param2;
          packet[3] = (byte)param3;
          packet[4] = (byte)param4;
          packet[5] = (byte)param5;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 6))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "," << (short)buffer[2] << ","
                      << (short)buffer[3] << "," << (short)buffer[4] << "," << (short)buffer[5] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'n':  // VK_N (N Command)
          std::cout << std::endl;
          std::cout << "Get all motors command flags or positions at "
                    << "once (N Command)" << std::endl;

          // get parameters
          std::cout << " subcommand: ";
          std::cin >> param1;
          std::cout << " subsubcommand: ";
          std::cin >> param2;

          // create packet
          packet[0] = 'N';
          packet[1] = (byte)param1;
          packet[2] = (byte)param2;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size < 2))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0];
            for (i = 1; i < (short)size; i++)
            {
              std::cout << "," << (short)buffer[i];
            }
            std::cout << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'g':  // VK_G (G Command)
          std::cout << std::endl;
          std::cout << "Set the single polynomial movement parameters "
                    << "(G Command)" << std::endl;

          // get parameters
          std::cout << " motor number: ";
          std::cin >> param1;
          if ((param1 < 1) || (param1 > 6))
          {
            break;
          }
          std::cout << " target pos: ";
          std::cin >> param2;
          std::cout << " time: ";
          std::cin >> param3;
          std::cout << " P10: ";
          std::cin >> param4;
          std::cout << " P11: ";
          std::cin >> param5;
          std::cout << " P12: ";
          std::cin >> param6;
          std::cout << " P13: ";
          std::cin >> param7;

          // create packet
          packet[0] = 'G';
          packet[1] = (byte)param1;
          packet[2] = (byte)(param2 >> 8);
          packet[3] = (byte)param2;
          packet[4] = (byte)(param3 >> 8);
          packet[5] = (byte)param3;
          packet[6] = (byte)(param4 >> 8);
          packet[7] = (byte)param4;
          packet[8] = (byte)(param5 >> 8);
          packet[9] = (byte)param5;
          packet[10] = (byte)(param6 >> 8);
          packet[11] = (byte)param6;
          packet[12] = (byte)(param7 >> 8);
          packet[13] = (byte)param7;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 2))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'G':  // VK_G (G+128 Command)
          std::cout << std::endl;
          std::cout << "Start the single polynomial movement "
                    << "(G+128 Command)" << std::endl;

          // get parameters
          std::cout << " subcommand: ";
          std::cin >> param1;
          std::cout << " exact flag: ";
          std::cin >> param2;

          // create packet
          packet[0] = 'G' | 0x80;
          packet[1] = (byte)param1;
          packet[2] = (byte)param2;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 2))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << " (g+128)," << (short)buffer[1] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'a':  // VK_A (A Command)
          std::cout << std::endl;
          std::cout << "Switch collision detection On and Off "
                    << "(A Command)" << std::endl;

          // get parameters
          std::cout << " master: ";
          std::cin >> param1;
          std::cout << " slave: ";
          std::cin >> param2;

          // create packet
          packet[0] = 'A';
          packet[1] = (byte)param1;
          packet[2] = (byte)param2;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 2))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'z':  // VK_Z (Z Command)
          std::cout << std::endl;
          std::cout << "Read Echo (Z Command)" << std::endl;

          // create packet
          packet[0] = 'Z';

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 1))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'y':  // VK_Y (Y Command)
          std::cout << std::endl;
          std::cout << "Read the Identification String (Y Command)" << std::endl;

          // create packet
          packet[0] = 'Y';

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0])
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            buffer[82] = 0;  // terminate c-string before \r\n
            std::cout << " answer:" << std::endl
                      << "  >" << buffer << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

          buffer[0] = 0x00;
          break;

        case 'x':  // VK_X (X Command)
          std::cout << std::endl;
          std::cout << "Read the Katana Command Table (X Command)" << std::endl;

          // create packet
          packet[0] = 'X';

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size < 7) || (((short)size) % 6 != 1))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "(command table)<" << std::endl
                      << " command table:" << std::endl;
            for (i = 0; i < (((short)size - 1) / 6); i++)
            {
              std::cout << "  " << (short)buffer[6 * i + 1] << ",";
              if ((short)buffer[6 * i + 2] < 128)
              {
                std::cout << buffer[6 * i + 2];
              }
              else
              {
                std::cout << (char)(buffer[6 * i + 2] & 0x7f) << "+128";
              }
              std::cout << "," << (short)buffer[6 * i + 3] << "," << (short)buffer[6 * i + 4] << ","
                        << (short)buffer[6 * i + 5] << ",";
              if ((short)buffer[6 * i + 6] < 128)
              {
                std::cout << buffer[6 * i + 6];
              }
              else
              {
                std::cout << (char)(buffer[6 * i + 6] & 0x7f) << "+128";
              }
              std::cout << std::endl;
            }
          }
          buffer[0] = 0x00;
          break;

        case 'e':  // VK_E (E Command)
          std::cout << std::endl;
          std::cout << "Read Gripper Sensor Data (E Command)" << std::endl;

          // get parameters
          std::cout << " sensor controller number: ";
          std::cin >> param1;
          if (param1 != 15)
          {
            break;
          }

          // create packet
          packet[0] = 'E';
          packet[1] = (byte)param1;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 18))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1];
            for (i = 2; i < (short)size; i++)
            {
              std::cout << "," << (short)buffer[i];
            }
            std::cout << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'b':  // VK_B (B Command)
          std::cout << std::endl;
          std::cout << "Read the Master firmware-version (B Command)" << std::endl;

          // create packet
          packet[0] = 'B';

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 3))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "," << (short)buffer[2] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'v':  // VK_V (V Command)
          std::cout << std::endl;
          std::cout << "Read the Slave firmware-version (V Command)" << std::endl;

          // get parameters
          std::cout << " motor number: ";
          std::cin >> param1;
          if ((param1 < 1) || (param1 > 6))
          {
            break;
          }

          // create packet
          packet[0] = 'V';
          packet[1] = (byte)param1;
          packet[2] = 32;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 13))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0];
            for (i = 1; i < (short)size; i++)
            {
              std::cout << "," << (short)buffer[i];
            }
            std::cout << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'i':  // VK_I (I Command)
          std::cout << std::endl;
          std::cout << "Get motCommand from slave (I Command)" << std::endl;

          // get parameters
          std::cout << " motor number: ";
          std::cin >> param1;
          if ((param1 < 1) || (param1 > 6))
          {
            break;
          }

          // create packet
          packet[0] = 'I';
          packet[1] = (byte)param1;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 3))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "," << (short)buffer[2] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'm':  // VK_M (M Command)
          std::cout << std::endl;
          std::cout << "Modbus read and writes (M Command)" << std::endl;

          // get parameters
          std::cout << " 'R'ead or 'W'rite: ";
          std::cin >> p1;
          if ((p1 != 'R') && (p1 != 'W'))
          {
            break;
          }
          std::cout << " register: ";
          std::cin >> param2;
          std::cout << " value: ";
          std::cin >> param3;

          // create packet
          packet[0] = 'M';
          packet[1] = (byte)p1;
          packet[2] = (byte)param2;
          packet[3] = (byte)(param3 >> 8);
          packet[4] = (byte)param3;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 4))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << b2s(buffer[1], buffer[2]) << "," << (short)buffer[3] << "<"
                      << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 't':  // VK_T (T Command)
          std::cout << std::endl;
          std::cout << "Katana 1.2 I/O (T Command)" << std::endl;

          // get parameters
          std::cout << " read (r), write (w) or set LED (l): ";
          std::cin >> p1;
          if ((p1 != 'r') && (p1 != 'w') && (p1 != 'l'))
          {
            break;
          }
          if (p1 == 'w')
          {
            std::cout << " value to write: ";
            std::cin >> param2;
          }
          else if (p1 == 'l')
          {
            std::cout << " set led to 'r'ed, 'g'reen or '0'(off): ";
            std::cin >> p2;
            if ((param2 == 'r') || (param2 == 'g'))
            {
              param2 = (short)p2;
            }
            else
            {
              param2 = 0;
            }
          }
          else
          {
            param2 = 0;
          }

          // create packet
          packet[0] = 'T';
          packet[1] = (byte)p1;
          packet[2] = (byte)param2;
          packet[3] = 0;
          packet[4] = 0;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 2))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        case 'o':  // VK_O (O Command)
          std::cout << std::endl;
          std::cout << "Navigation Control (O Command)" << std::endl;

          // get parameters
          std::cout << " command: ";
          std::cin >> param1;
          if ((param1 < 0) || (param1 > 3))
          {
            break;
          }
          // std::cout << " CVar1: ";
          // std::cin >> param2;
          param2 = 0;

          // create packet
          packet[0] = 'O';
          packet[1] = (byte)param1;
          packet[2] = (byte)param2;

          // communicate packet
          protocol->comm(packet, buffer, &size);
          if (!buffer[0] || ((short)size != 3))
          {
            std::cout << " command failed!" << std::endl;
          }
          else
          {
            std::cout << " answer:" << std::endl
                      << "  >" << buffer[0] << "," << (short)buffer[1] << "," << (short)buffer[2] << "<" << std::endl;
          }
          buffer[0] = 0x00;
          break;

        default:  // Error message
          std::cout << "\n'" << input << "' is not a valid command.\n" << std::endl;
          break;
      }
    }
    catch (Exception &e)
    {
      std::cout << "\nERROR: " << e.message() << std::endl;
    }
  }
  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
