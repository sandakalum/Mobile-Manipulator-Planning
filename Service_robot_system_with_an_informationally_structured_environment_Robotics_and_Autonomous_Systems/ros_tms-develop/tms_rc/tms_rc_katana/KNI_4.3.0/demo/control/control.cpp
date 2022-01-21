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
// control.cpp
// demo program for KNI libraries
//////////////////////////////////////////////////////////////////////////////////
#include "kniBase.h"
#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <fstream>
#include <pthread.h>
//////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
#include <conio.h>
#else  // LINUX
#include "keyboard.h"
#endif
#define LEFT false
#define RIGHT true
//////////////////////////////////////////////////////////////////////////////////
// Thread structs:
pthread_mutex_t mutex;

struct TPoint
{
  double X, Y, Z;
  double phi, theta, psi;
};

struct TCurrentMot
{
  int idx;
  bool running;
  bool dir;
};

struct Tpos
{
  static std::vector< int > x, y, z, u, v, w;
  static const int xArr[], yArr[], zArr[], uArr[], vArr[], wArr[];
};
// Katana obj.
std::auto_ptr< CLMBase > katana;
// std::vector<int> Foo::vec(array, array + sizeof(array)/sizeof(*array));
// positionen, hard-coded. Use values from file instead
const int Tpos::xArr[] = {30206, -23393, -3066, 14454, 30000, 30000};
const int Tpos::yArr[] = {24327, -7837, -16796, 5803, 30290, 31000};
const int Tpos::zArr[] = {24327, -7837, -16796, 5802, 30290, 10924};
const int Tpos::uArr[] = {5333, -13791, -9985, 11449, 30996, 12063};
const int Tpos::vArr[] = {-3799, -5703, -11676, 8210, 30995, 12063};
const int Tpos::wArr[] = {-3799, -5703, -11676, 8210, 30995, 30992};
std::vector< int > Tpos::x(xArr, xArr + sizeof(xArr) / sizeof(*xArr));
std::vector< int > Tpos::y(yArr, yArr + sizeof(yArr) / sizeof(*yArr));
std::vector< int > Tpos::z(zArr, zArr + sizeof(zArr) / sizeof(*zArr));
std::vector< int > Tpos::u(uArr, uArr + sizeof(uArr) / sizeof(*uArr));
std::vector< int > Tpos::v(vArr, vArr + sizeof(vArr) / sizeof(*vArr));
std::vector< int > Tpos::w(wArr, wArr + sizeof(wArr) / sizeof(*wArr));
std::vector< TPoint > points(0);
void StartPointlistMovement();
void StartProgram(int index);
pthread_t tid;
void* RunProgram(void*);
pid_t threadPid;
int retVal = 0;
bool progRunning = false;
const double PI = 3.14159265358979323846;
//////////////////////////////////////////////////////////////////////////////////
void DisplayHelp()
{
  std::cout << "-------------------------------------------	\n";
  std::cout << "?: Display this help\n";
  std::cout << "c: Calibrate the Katana\n";
  std::cout << "b: Read current controller types\n";
  std::cout << "e: Read the current encoder values\n";
  std::cout << "E: Read the current force values\n";
  std::cout << "o: Switch motors off/on (Default: On)\n";
  std::cout << "r: Switch angle format: Radian/Degree (Default: Rad)\n";
  std::cout << "x: Read the current position\n";
  std::cout << "v: Set the velocity limits for all motors seperately\n";
  std::cout << "V: Set the velocity limits for all motors (or for the TCP if in linear movement mode)\n";
  std::cout << "a: Set the acceleration limits for all motors seperately\n";
  std::cout << "A: Set the acceleration limits for all motors\n";
  std::cout << ",: Set the force limits for all motors\n";
  std::cout << "w: Read the velocity limits of all motors	\n";
  std::cout << "W: Read the acceleration limits of all motors	\n";
  std::cout << "q: Read the Sensors\n";
  std::cout << "y: Set a new position using IK\n";
  std::cout << "l: Switch on/off linear movements\n";
  std::cout << "<: Add a point to the point list\n";
  std::cout << ">: Move to a specific point\n";
  std::cout << " : (space) Move to the next point in the point list\n";
  std::cout << "=: write pointlist to file\n";
  std::cout << "(: calculate DK from any encoders\n";
  std::cout << "f: read pointlist from file\n";
  std::cout << "g: Open Gripper\n";
  std::cout << "h: Close Gripper\n";
  std::cout << "n: Set the speed collision limit for all motors seperately\n";
  std::cout << "N: Set the speed collision limit for all motors\n";
  std::cout << "s: Set the position collision limit for all motors seperately\n";
  std::cout << "S: Set the position collision limit for all motors\n";
  std::cout << "t: Switch collision limit on\n";
  std::cout << "T: Switch collision limit off\n";
  std::cout << "u: Unblock motors after crash\n";
  std::cout << "d: Move motor to degrees\n";
  std::cout << "z: Set TCP offset\n\n\n";
  std::cout << "Keyboard of Katana, use the following keys:\n\n";
  std::cout << "1: Move motor1 left\n";
  std::cout << "2: Move motor1 right\n";
  std::cout << "3: Move motor2 left\n";
  std::cout << "4: Move motor2 right\n";
  std::cout << "5: Move motor3 left\n";
  std::cout << "6: Move motor3 right\n";
  std::cout << "7: Move motor4 left\n";
  std::cout << "8: Move motor4 right\n";
  std::cout << "9: Move motor5 left\n";
  std::cout << "0: Move motor5 right\n";
  std::cout << "/: Move motor6 left\n";
  std::cout << "*: Move motor6 right\n";
  std::cout << ".: Toggle Step mode\n";
  std::cout << "+: Increase step size\n";
  std::cout << "-: Decrease step size\n\n";
  std::cout << "$: Start/Stop Program\n";
  std::cout << "p: Start/Stop movement through points list\n\n";
}
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  if (argc != 3)
  {
    std::cout << "---------------------------------\n";
    std::cout << "usage: control CONFIGFILE IP_ADDR\n";
    std::cout << "---------------------------------\n";
    return 0;
  }

  std::cout << "--------------------\n";
  std::cout << "CONTROL DEMO STARTED\n";
  std::cout << "--------------------\n";

  //----------------------------------------------------------------//
  // open device: a serial port is opened in this case
  //----------------------------------------------------------------//

  std::auto_ptr< CCdlSocket > device;
  std::auto_ptr< CCplSerialCRC > protocol;

  try
  {
    int port = 5566;
    device.reset(new CCdlSocket(argv[2], port));

    std::cout << "-------------------------------------------\n";
    std::cout << "success:  port " << port << " open\n";
    std::cout << "-------------------------------------------\n";

    //--------------------------------------------------------//
    // init protocol:
    //--------------------------------------------------------//

    protocol.reset(new CCplSerialCRC());
    std::cout << "-------------------------------------------\n";
    std::cout << "success: protocol class instantiated\n";
    std::cout << "-------------------------------------------\n";
    protocol->init(device.get());  // fails if no response from Katana
    std::cout << "-------------------------------------------\n";
    std::cout << "success: communication with Katana initialized\n";
    std::cout << "-------------------------------------------\n";

    //--------------------------------------------------------//
    // init robot:
    //--------------------------------------------------------//
    katana.reset(new CLMBase());
    katana->create(argv[1], protocol.get());
  }
  catch (Exception& e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
    return -1;
  }
  std::cout << "-------------------------------------------\n";
  std::cout << "success: katana initialized\n";
  std::cout << "-------------------------------------------\n";

  DisplayHelp();

  short counter = 0;
  bool loop = true;
  int pos = 0;
  bool DispInRad = true;
  bool IsOff = false;
  bool useLinearMode = false;
  double RadToDeg = 1.0;
  std::vector< int > encoders(katana->getNumberOfMotors(), 0);
  const TKatMOT* motors;
  TCurrentMot mot[6];
  for (int i = 0; i < 6; i++)
  {
    mot[i].running = false;
    mot[i].idx = i;
    mot[i].dir = true;
    mot[i].dir = RIGHT;
  }
  bool stepMode = false;
  int stepSize = 100;
  bool runProgram = false;
  // pthread_mutex_init(&mutex, 0);
  // pthread_mutex_lock(&mutex);
  // pthread_mutex_unlock(&mutex);
  // set sensor controller values
  /*TSctDesc  sctdesc[1] = {{15, 8, 16}};		//sctID,resol,count
  CSctBase* sctarr     = new CSctBase[1];		//create sensor-ctrl class
  TKatSCT   katsct     = {1, sctarr, sctdesc};	//fill TKatSCT structure
  */
  CSctBase* sensctrl = &katana->GetBase()->GetSCT()->arr[0];
  int limit;

  // set linear velocity to 60
  katana->setMaximumLinearVelocity(60);

  while (loop)
  {
    double arr_pos[6];

    int input = _getch();
    if (progRunning)
    {
      // Thread killen:
      progRunning = false;
      continue;
    }

    try
    {
      switch (input)
      {
        /*			case 'i': //VK_I: Test routine, connected P2P movement
                {
                double posX = points[counter].X;
                double posY = points[counter].Y;
                double posZ = points[counter].Z;
                double posPhi = points[counter].phi;
                double posTheta = points[counter].theta;
                double posPsi = points[counter].psi;
                katana->moveRobotTo(posX, posY, posZ, posPhi, posTheta, posPsi, false);
                counter++;
                counter = counter % ((short) points.size());
                katana->movP2P(posX, posY, posZ, posPhi, posTheta, posPsi, points[counter].X, points[counter].Y,
           points[counter].Z, points[counter].phi, points[counter].theta, points[counter].psi, true, 20, true);
                counter++;
                counter = counter % ((short) points.size());
                break;
                }
        */
        case '(':
        {
          std::vector< double > pose_result(6, 0);
          std::vector< int > etc;
          etc.push_back(20000);
          etc.push_back(-20000);
          etc.push_back(-20000);
          etc.push_back(20000);
          etc.push_back(20000);
          etc.push_back(20000);
          katana->getCoordinatesFromEncoders(pose_result, etc);
          std::cout.precision(6);
          std::cout << "\n------------------------------------\n";
          std::cout << "X: " << pose_result.at(0) << "\n";
          std::cout << "Y: " << pose_result.at(1) << "\n";
          std::cout << "Z: " << pose_result.at(2) << "\n";
          std::cout << "phi: " << RadToDeg* pose_result.at(3) << "\n";
          std::cout << "theta: " << RadToDeg* pose_result.at(4) << "\n";
          std::cout << "psi: " << RadToDeg* pose_result.at(5) << "\n";
          std::cout << "------------------------------------\n";
        }
        break;

        case '.':
          if (stepMode == false)
          {
            stepMode = true;
            std::cout << "Step Mode ON \n" << std::endl;
          }
          else
          {
            stepMode = false;
            std::cout << "Step Mode OFF \n" << std::endl;
          }
          break;
        case '+':
          if (stepSize <= 3000)
          {
            stepSize += 300;
            std::cout << "Step Size =  " << stepSize << std::endl;
          }
          break;
        case '-':
          if (stepSize >= 330)
          {
            stepSize -= 300;
            std::cout << "Step Size =  " << stepSize << std::endl;
          }
          break;
        case '1':
          if ((stepMode == true) || (mot[0].running == false || (mot[0].running == true && mot[0].dir == LEFT)))
          {
            mot[0].idx = 0;
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[0].idx].GetEncoderMinPos();
            if (motors->arr[mot[0].idx].checkEncoderInRange(mot[0].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(0) + stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[0].idx, pos);
              mot[0].running = true;
              mot[0].dir = RIGHT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(0);
            mot[0].running = false;
          }
          break;
        case '2':
          if ((stepMode == true) || (mot[0].running == false || (mot[0].running == true && mot[0].dir == RIGHT)))
          {
            mot[0].idx = 0;
            // motors = katana->GetBase()->GetMOT();
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[0].idx].GetEncoderMaxPos();
            if (motors->arr[mot[0].idx].checkEncoderInRange(mot[0].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(0) - stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[0].idx, pos);
              mot[0].running = true;
              mot[0].dir = LEFT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(0);
            mot[0].running = false;
          }
          break;
        case '3':
          if ((stepMode == true) || (mot[1].running == false || (mot[1].running == true && mot[1].dir == LEFT)))
          {
            mot[1].idx = 1;
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[1].idx].GetEncoderMinPos();
            if (motors->arr[mot[1].idx].checkEncoderInRange(mot[1].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(1) + stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[1].idx, pos);
              mot[1].running = true;
              mot[1].dir = RIGHT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(1);
            mot[1].running = false;
          }
          break;
        case '4':
          if ((stepMode == true) || (mot[1].running == false || (mot[1].running == true && mot[1].dir == RIGHT)))
          {
            mot[1].idx = 1;
            // motors = katana->GetBase()->GetMOT();
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[1].idx].GetEncoderMaxPos();
            if (motors->arr[mot[1].idx].checkEncoderInRange(mot[1].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(1) - stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[1].idx, pos);
              mot[1].running = true;
              mot[1].dir = LEFT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(1);
            mot[1].running = false;
          }
          break;
        case '5':
          if ((stepMode == true) || (mot[2].running == false || (mot[2].running == true && mot[2].dir == LEFT)))
          {
            mot[2].idx = 2;
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[2].idx].GetEncoderMinPos();
            if (motors->arr[mot[2].idx].checkEncoderInRange(mot[2].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(2) + stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[2].idx, pos);
              mot[2].running = true;
              mot[2].dir = RIGHT;
            }
            else
            {
              std::cout << "\nErunProgram =ncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(2);
            mot[2].running = false;
          }
          break;
        case '6':
          if ((stepMode == true) || (mot[2].running == false || (mot[2].running == true && mot[2].dir == RIGHT)))
          {
            mot[2].idx = 2;
            // motors = katana->GetBase()->GetMOT();
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[2].idx].GetEncoderMaxPos();
            if (motors->arr[mot[2].idx].checkEncoderInRange(mot[2].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(2) - stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[2].idx, pos);
              mot[2].running = true;
              mot[2].dir = LEFT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(2);
            mot[2].running = false;
          }
          break;
        case '7':
          if ((stepMode == true) || (mot[3].running == false || (mot[3].running == true && mot[3].dir == LEFT)))
          {
            mot[3].idx = 3;
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[3].idx].GetEncoderMinPos();
            if (motors->arr[mot[3].idx].checkEncoderInRange(mot[3].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(3) + stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[3].idx, pos);
              mot[3].running = true;
              mot[3].dir = RIGHT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(3);
            runProgram = mot[3].running = false;
          }
          break;
        case '8':
          if ((stepMode == true) || (mot[3].running == false || (mot[3].running == true && mot[3].dir == RIGHT)))
          {
            mot[3].idx = 3;
            // motors = katana->GetBase()->GetMOT();
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[3].idx].GetEncoderMaxPos();
            if (motors->arr[mot[3].idx].checkEncoderInRange(mot[3].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(3) - stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[3].idx, pos);
              mot[3].running = true;
              mot[3].dir = LEFT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(3);
            mot[3].running = false;
          }
          break;
        case '9':
          if ((stepMode == true) || (mot[4].running == false || (mot[4].running == true && mot[4].dir == LEFT)))
          {
            mot[4].idx = 4;
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[4].idx].GetEncoderMinPos();
            if (motors->arr[mot[4].idx].checkEncoderInRange(mot[4].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(4) + stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[4].idx, pos);
              mot[4].running = true;
              mot[4].dir = RIGHT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(4);
            mot[4].running = false;
          }
          break;
        case '0':
          if ((stepMode == true) || (mot[4].running == false || (mot[4].running == true && mot[4].dir == RIGHT)))
          {
            mot[4].idx = 4;
            // motors = katana->GetBase()->GetMOT();
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[4].idx].GetEncoderMaxPos();
            if (motors->arr[mot[4].idx].checkEncoderInRange(mot[4].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(4) - stepSize;
              }
              std::cout << "\nMoving to max encoder position " << pos << std::endl;
              katana->moveMotorToEnc(mot[4].idx, pos);
              mot[4].running = true;
              mot[4].dir = LEFT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(4);
            mot[4].running = false;
          }
          break;
        case '/':
          if ((stepMode == true) || (mot[5].running == false || (mot[5].running == true && mot[5].dir == LEFT)))
          {
            mot[5].idx = 5;
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[0].idx].GetEncoderMinPos();
            if (motors->arr[mot[0].idx].checkEncoderInRange(mot[5].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(5) + stepSize;
                std::cout << "\nMoving to max encoder position " << pos + 1000 << std::endl;
                katana->moveMotorToEnc(mot[5].idx, pos);
              }
              else
              {
                katana->openGripper();
              }
              mot[5].running = true;
              mot[5].dir = RIGHT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(5);
            mot[5].running = false;
          }
          break;
        case '*':
          if ((stepMode == true) || (mot[5].running == false || (mot[5].running == true && mot[5].dir == RIGHT)))
          {
            mot[5].idx = 5;
            // motors = katana->GetBase()->GetMOT();
            motors = katana->GetBase()->GetMOT();
            pos = motors->arr[mot[5].idx].GetEncoderMaxPos();
            if (motors->arr[mot[5].idx].checkEncoderInRange(mot[5].idx))
            {
              if (stepMode)
              {
                pos = katana->getMotorEncoders(5) - stepSize;
                std::cout << "\nMoving to max encoder position " << pos << std::endl;
                katana->moveMotorToEnc(mot[5].idx, pos);
              }
              else
              {
                katana->closeGripper();
              }
              mot[5].running = true;
              mot[5].dir = LEFT;
            }
            else
            {
              std::cout << "\nEncoder target value out of range! \n" << std::endl;
            }
          }
          else
          {
            katana->freezeMotor(5);
            mot[5].running = false;
          }
          break;
        case '$':
          // 				loop = false;
          // 				runProgram = true;
          // 				StartProgram(0);
          std::cout << "\n Adjust hardcoded pointlist and program for your katana before running the program!!!\n"
                    << std::endl;
          break;
        case '?':
          DisplayHelp();
          break;

        case 'r':
          if (DispInRad)
          {
            DispInRad = false;
            RadToDeg = 180.0 / PI;
            std::cout << "\nAngles in DEGREE !! \n" << std::endl;
          }
          else
          {
            DispInRad = true;
            RadToDeg = 1.0;
            std::cout << "\nAngles in RADIAN !! \n" << std::endl;
          }
          break;

        case 'c':  // VK_C (Calibration)
          std::cout << "\nCalibrating Katana, please wait for termination... \n" << std::flush;
          katana->calibrate();
          break;

        case 'b':  // VK_B (get current controller type)
          std::cout << "\nController types: " << std::endl;
          for (int i = 0; i < katana->getNumberOfMotors(); ++i)
          {
            std::cout << " " << (katana->getCurrentControllerType(i + 1) == 0 ? "position" : "current");
          }
          std::cout << std::endl;
          break;

        case 'e':  // VK_E (read encoder values)
          std::cout << "\nEncoder values: " << std::endl;
          katana->getRobotEncoders(encoders.begin(), encoders.end());
          for (std::vector< int >::iterator i = encoders.begin(); i != encoders.end(); ++i)
          {
            std::cout << *i << " ";
          }
          std::cout << std::endl;
          break;

        case 'E':  // VK_E (read force values)
          std::cout << "\nForce values:" << std::endl;
          for (int i = 0; i < katana->getNumberOfMotors(); ++i)
          {
            std::cout << " " << (int)katana->getForce(i + 1);
          }
          std::cout << std::endl;
          break;

        case 'o':  // VK_O (motors off/on)
          if (IsOff)
          {
            katana->switchRobotOn();
            IsOff = false;
            std::cout << "\nMotors on\n";
          }
          else
          {
            katana->switchRobotOff();
            IsOff = true;
            std::cout << "\nMotors off\n";
          }
          break;

        case 'x':  // VK_O (DK to screen)
        {
          TPoint p;
          katana->getCoordinates(p.X, p.Y, p.Z, p.phi, p.theta, p.psi);

          std::cout.precision(6);
          std::cout << "\n------------------------------------\n";
          std::cout << "X: " << p.X << "\n";
          std::cout << "Y: " << p.Y << "\n";
          std::cout << "Z: " << p.Z << "\n";
          std::cout << "phi: " << RadToDeg* p.phi << "\n";
          std::cout << "theta: " << RadToDeg* p.theta << "\n";
          std::cout << "psi: " << RadToDeg* p.psi << "\n";
          std::cout << "------------------------------------\n";

          break;
        }
        case 'q':  // read sensors:
        {
          std::cout << "\nCurrent Sensor values:" << std::endl;
          const TSctDAT* data = sensctrl->GetDAT();
          bool* change = new bool[data->cnt];
          byte* lastarr = new byte[data->cnt];
          sensctrl->recvDAT();
          for (int k = 0; k < data->cnt; k++)
          {  // init stuff
            change[k] = false;
            lastarr[k] = (byte)data->arr[k];
          }
          // clock_t t = clock();							//init timer
          sensctrl->recvDAT();  // update sensor data
          for (int i = 0; i < data->cnt; i++)
          {
            std::cout.width(5);
            std::cout << data->arr[i] << " ";  // printout data
          }
          std::cout << "\n";
          /*
          while (clock() - t < 25) {}						//wait 25 millisec

          for (int j=0; j<data->cnt; j++) {
            change[j] |= (data->arr[j] != lastarr[j]);
            lastarr[j] = data->arr[j];
          }
          */
          break;
        }
        case 'v':  // VK_V (Set max vel)
        {
          short velocity;
          std::cout << "\n\nSet maximum velocity for motors to: \n";
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
          {
            std::cout << motorNumber + 1 << ": ";
            std::cin >> velocity;
            katana->setMotorVelocityLimit(motorNumber, velocity);
          }
        }
        break;

        case 'V':  // VK_V (Set max vel)
        {
          if (useLinearMode)
          {
            double velocity;
            std::cout << "\n\nSet the TCP velocity to: ";
            std::cin >> velocity;
            katana->setMaximumLinearVelocity(velocity);
            katana->setRobotVelocityLimit(static_cast< short >(velocity));
          }
          else
          {
            short velocity;
            std::cout << "\n\nSet maximum velocity for all motors to: ";
            std::cin >> velocity;
            katana->setRobotVelocityLimit(velocity);
            katana->setMaximumLinearVelocity(static_cast< double >(velocity));
          }
          std::cout << std::endl;
        }
        break;

        case 'a':  // VK_A (Set max acc)
        {
          short acceleration;
          std::cout << "\n\nSet maximum velocity for motors to: \n";
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
          {
            std::cout << motorNumber + 1 << ": ";
            std::cin >> acceleration;
            katana->setMotorAccelerationLimit(motorNumber, acceleration);
          }
        }
        break;
        case 'A':  // VK_A (Set max acc)
        {
          short acceleration;
          std::cout << "\n\nSet maximum acceleration for all motors to: ";
          std::cin >> acceleration;
          std::cout << std::endl;
          katana->setRobotAccelerationLimit(acceleration);
        }
        break;
        case ',':  // set force limits for all motors
        {
          short limit;
          std::cout << "\nSet force limit for all motors to (%): ";
          std::cin >> limit;
          std::cout << std::endl;
          katana->setForceLimit(0, limit);
        }
        break;

        case 'w':  // VK_W (Read current max vel)
        {
          std::cout << "\nCurrent velocity limits:" << std::endl;
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
            std::cout << motorNumber + 1 << ": " << katana->getMotorVelocityLimit(motorNumber) << std::endl;
          std::cout << "linear: " << katana->getMaximumLinearVelocity() << std::endl;
          break;
        }
        case 'W':  // VK_W (Read current max acc)
        {
          std::cout << "\nCurrent acceleration limits:" << std::endl;
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
            std::cout << motorNumber + 1 << ": " << katana->getMotorAccelerationLimit(motorNumber) << std::endl;
          break;
        }

        case 'l':  // VK_L (switch linear mode)
          if (useLinearMode)
          {
            std::cout << "Switching to inverse kinematics movement mode\n";
          }
          else
          {
            std::cout << "Switching to linear movement mode\n";
          }
          useLinearMode = !useLinearMode;
          break;
        case 'y':  // VK_Y (IKGoto)
          std::cout << "\n\nInsert cartesian parameters: \n";
          std::cout << "X: ";
          std::cin >> arr_pos[0];
          std::cout << "Y: ";
          std::cin >> arr_pos[1];
          std::cout << "Z: ";
          std::cin >> arr_pos[2];
          std::cout << "phi: ";
          std::cin >> arr_pos[3];
          std::cout << "theta: ";
          std::cin >> arr_pos[4];
          std::cout << "psi: ";
          std::cin >> arr_pos[5];

          arr_pos[3] = 1 / RadToDeg * arr_pos[3];
          arr_pos[4] = 1 / RadToDeg * arr_pos[4];
          arr_pos[5] = 1 / RadToDeg * arr_pos[5];

          if (useLinearMode)
          {
            katana->moveRobotLinearTo(arr_pos[0], arr_pos[1], arr_pos[2], arr_pos[3], arr_pos[4], arr_pos[5]);
          }
          else
          {
            katana->moveRobotTo(arr_pos[0], arr_pos[1], arr_pos[2], arr_pos[3], arr_pos[4], arr_pos[5]);
          }
          break;

        case '<':
          std::cout.precision(6);
          TPoint point;
          katana->getCoordinates(point.X, point.Y, point.Z, point.phi, point.theta, point.psi);
          std::cout << "Point: ";
          std::cout << "  X=" << point.X;
          std::cout << ", Y=" << point.Y;
          std::cout << ", Z=" << point.Z;
          std::cout << ", phi=" << RadToDeg* point.phi;
          std::cout << ", theta=" << RadToDeg* point.theta;
          std::cout << ", psi=" << RadToDeg* point.psi;
          std::cout << " ... added to point list as number ";
          std::cout << points.size() << std::endl;
          points.push_back(point);
          break;

        case '>':
          std::cout.width(10);
          std::cout.precision(3);
          std::cout << "\nMoving to point? ";
          unsigned int pointNumber;
          std::cin >> pointNumber;
          if (pointNumber >= points.size())
          {
            std::cout << "Invalid point number. You have only " << points.size() << " points in your list" << std::endl;
            break;
          }
          std::cout.width(6);
          std::cout << " x=" << points[pointNumber].X;
          std::cout.width(6);
          std::cout << " y=" << points[pointNumber].Y;
          std::cout.width(6);
          std::cout << " z=" << points[pointNumber].Z;
          std::cout.width(6);
          std::cout << " phi=" << RadToDeg* points[pointNumber].phi;
          std::cout.width(6);
          std::cout << " theta=" << RadToDeg* points[pointNumber].theta;
          std::cout.width(6);
          std::cout << " psi=" << RadToDeg* points[pointNumber].psi;
          std::cout << std::endl;
          if (useLinearMode)
          {
            katana->moveRobotLinearTo(points[pointNumber].X, points[pointNumber].Y, points[pointNumber].Z,
                                      points[pointNumber].phi, points[pointNumber].theta, points[pointNumber].psi);
          }
          else
          {
            katana->moveRobotTo(points[pointNumber].X, points[pointNumber].Y, points[pointNumber].Z,
                                points[pointNumber].phi, points[pointNumber].theta, points[pointNumber].psi);
          }
          break;

        case 3:
        case 4:
        case 27:  // VK_ESCAPE
          loop = false;
          continue;

        case ' ':  // VK_SPACE
          // Move to the next point
          std::cout.width(10);
          std::cout.precision(3);
          std::cout << "Moving to point " << counter << ": ";
          std::cout.width(6);
          std::cout << " x=" << points[counter].X;
          std::cout.width(6);
          std::cout << " y=" << points[counter].Y;
          std::cout.width(6);
          std::cout << " z=" << points[counter].Z;
          std::cout.width(6);
          std::cout << " phi=" << RadToDeg* points[counter].phi;
          std::cout.width(6);
          std::cout << " theta=" << RadToDeg* points[counter].theta;
          std::cout.width(6);
          std::cout << " psi=" << RadToDeg* points[counter].psi;
          std::cout << std::endl;

          if (useLinearMode)
          {
            katana->moveRobotLinearTo(points[counter].X, points[counter].Y, points[counter].Z, points[counter].phi,
                                      points[counter].theta, points[counter].psi);
          }
          else
          {
            katana->moveRobotTo(points[counter].X, points[counter].Y, points[counter].Z, points[counter].phi,
                                points[counter].theta, points[counter].psi);
          }
          counter++;
          counter = counter % ((short)points.size());
          break;

        case 'g':
          std::cout << "Opening gripper...\n";
          katana->openGripper();
          break;
        case 'h':
          std::cout << "Close gripper...\n";
          katana->closeGripper();
          break;
        case 'n':
          std::cout << "Set speed collision limit for motors to: \n";
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
          {
            std::cout << motorNumber + 1 << ": ";
            std::cin >> limit;
            katana->setSpeedCollisionLimit(motorNumber, limit);
          }
          break;
        case 'N':
          std::cout << "Set speed collision limit for all motors to: \n";
          std::cin >> limit;
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
          {
            katana->setSpeedCollisionLimit(motorNumber, limit);
          }
          break;
        case 's':
          std::cout << "Set position collision limit for motors to: \n";
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
          {
            std::cout << motorNumber + 1 << ": ";
            std::cin >> limit;
            katana->setPositionCollisionLimit(motorNumber, limit);
          }
          break;
        case 'S':
          std::cout << "Set position collision limit for all motors to: \n";
          std::cin >> limit;
          for (short motorNumber = 0; motorNumber < katana->getNumberOfMotors(); ++motorNumber)
          {
            katana->setPositionCollisionLimit(motorNumber, limit);
          }
          break;
        case 't':
          std::cout << "Collision detection enabled\n";
          katana->enableCrashLimits();
          break;
        case 'T':
          std::cout << "WARNING: Collision detection disabled\n";
          katana->disableCrashLimits();
          break;
        case 'u':
          std::cout << "Unblocking motors\n";
          katana->unBlock();
          break;

        case 'f':
        {
          using namespace std;

          cout << "Loading which file?\n";
          string filename;
          cin >> filename;

          ifstream listfile(filename.c_str());
          if (!listfile)
          {
            cout << "File not found or access denied." << endl;
            break;
          }

          string line;
          vector< string > tokens;
          const string delimiter = ",";

          int lines = 0;

          while (!listfile.eof())
          {
            listfile >> line;
            string::size_type lastPos = line.find_first_not_of(delimiter, 0);
            string::size_type pos = line.find_first_of(delimiter, lastPos);

            while (string::npos != pos || string::npos != lastPos)
            {
              // Found a token, add it to the vector.
              tokens.push_back(line.substr(lastPos, pos - lastPos));
              // Skip delimiters.  Note the "not_of"
              lastPos = line.find_first_not_of(delimiter, pos);
              // Find next "non-delimiter"
              pos = line.find_first_of(delimiter, lastPos);
            }

            TPoint point;
            point.X = atof((tokens.at(0)).data());
            point.Y = atof((tokens.at(1)).data());
            point.Z = atof((tokens.at(2)).data());
            point.phi = atof((tokens.at(3)).data()) / RadToDeg;
            point.theta = atof((tokens.at(4)).data()) / RadToDeg;
            point.psi = atof((tokens.at(5)).data()) / RadToDeg;
            points.push_back(point);
            ++lines;
            tokens.clear();
          }
          cout << lines << " points loaded.\n";

          break;
        }

        case '=':
        {
          using namespace std;
          cout << "Which file? WARNING: Will be overwritten!\n";
          string filename;
          cin >> filename;

          ofstream listfile(filename.c_str(), ios_base::out);

          int count = 0;

          for (std::vector< TPoint >::iterator iter = points.begin(); iter != points.end(); ++iter)
          {
            listfile.precision(8);
            if (count != 0)
              listfile << endl;
            listfile << iter->X << "," << iter->Y << "," << iter->Z << ",";
            listfile << iter->phi << "," << iter->theta << "," << iter->psi;
            ++count;
          }
          cout << count << " points saved.\n";
          listfile.close();
          break;
        }
        case 'p':
        {
          using namespace std;
          // int available;
          // cout << "How many movements?\n";
          // int moves;
          // cin >> moves;
          std::cout << "Start playback!\n";
          for (int i = 1; i > 0 /*i <= moves*/; ++i)
          {
            if (useLinearMode)
            {
              katana->moveRobotLinearTo(points[i % points.size()].X, points[i % points.size()].Y,
                                        points[i % points.size()].Z, points[i % points.size()].phi,
                                        points[i % points.size()].theta, points[i % points.size()].psi, true, 10000);
            }
            else
            {
              katana->moveRobotTo(points[i % points.size()].X, points[i % points.size()].Y, points[i % points.size()].Z,
                                  points[i % points.size()].phi, points[i % points.size()].theta,
                                  points[i % points.size()].psi, true, 10000);
            }
            if (i % 100 == 0)
            {
              std::cout << i << ", " << std::flush;
            }
          }

          break;
        }

        case 'd':
        {
          long motor;
          double degrees;
          std::cout << "\nMoving motor to degrees\n";
          std::cout << " motor: ";
          std::cin >> motor;
          std::cout << " degrees: ";
          std::cin >> degrees;
          if ((motor > 0) && (motor < 7))
          {
            katana->movDegrees(motor - 1, degrees);
          }
          else
          {
            std::cout << "motor has to be a number from 1 to 6\n";
          }

          break;
        }

        case 'z':
        {
          double xoff, yoff, zoff, psioff;
          std::cout << "X offset (m): ";
          std::cin >> xoff;
          std::cout << "Y offset (m): ";
          std::cin >> yoff;
          std::cout << "Z offset (m): ";
          std::cin >> zoff;
          std::cout << "psi offset around x axis (rad): ";
          std::cin >> psioff;
          katana->setTcpOffset(xoff, yoff, zoff, psioff);

          break;
        }

        default:  // Error message
          std::cout << "\n'" << input << "' is not a valid command.\n" << std::endl;
          break;
      }
    }
    catch (Exception& e)
    {
      std::cout << "\nERROR: " << e.message() << std::endl;
    }
  }
  /*//Q&D test to launch program:
  if(runProgram){
    StartProgram(0);
  }
  */
  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
void StartProgram(int index)
{
  // Q&D test to launch program:
  // std::system("/home/katprog katana6M180.cfg 1");
  progRunning = true;
  pthread_create(&tid, NULL, RunProgram, (void*)&retVal);  //(&tid, NULL, start_func, arg);
  pthread_detach(tid);
}
//////////////////////////////////////////////////////////////////////////////////
void* RunProgram(void*)
{
  // katana->calibrate();
  std::cout << "\nProgram running...type any key to stop after the next cycle\n";
  while (progRunning)
  {
    if (progRunning)
      katana->moveRobotToEnc(Tpos::x, true);
    if (progRunning)
      katana->moveRobotToEnc(Tpos::y, true);
    if (progRunning)
      katana->moveRobotToEnc(Tpos::z, true);
    if (progRunning)
      katana->moveRobotToEnc(Tpos::u, true);
    if (progRunning)
      katana->moveRobotToEnc(Tpos::v, true);
    if (progRunning)
      katana->moveRobotToEnc(Tpos::w, true);
  }
  pthread_exit((void*)&retVal);
  return ((void*)&retVal);
}
//////////////////////////////////////////////////////////////////////////////////
