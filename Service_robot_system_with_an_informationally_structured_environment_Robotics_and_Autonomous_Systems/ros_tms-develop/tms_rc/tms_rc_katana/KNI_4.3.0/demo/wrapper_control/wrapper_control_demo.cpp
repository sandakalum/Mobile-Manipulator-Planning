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
// wrapper_control.cpp
// demo program for the kni C wrapper dll
// PKE/JHA/TPE 2008
//////////////////////////////////////////////////////////////////////////////////
#include "kniBase.h"
#include "kni_wrapper/kni_wrapper.h"
#include "kniBase.h"
#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
//////////////////////////////////////////////////////////////////////////////////
// defines
#ifdef WIN32
#include <conio.h>
#else  // LINUX
#include "keyboard.h"
#endif
//////////////////////////////////////////////////////////////////////////////////
// globals:
const int DEFAULT_ACCELERATION = 2;
const int DEFAULT_VELOCITY = 50;
int acceleration = DEFAULT_ACCELERATION;
int velocity = DEFAULT_VELOCITY;
std::vector< TMovement * > movement_vector;
bool isOff = false;
//////////////////////////////////////////////////////////////////////////////////
// helper functions
TMovement *allocateMovement()
{
  TMovement *movement = (struct TMovement *)malloc(sizeof(TMovement));
  TPos *pos = (struct TPos *)malloc(sizeof(TPos));
  movement->pos = *pos;
  movement_vector.push_back(movement);
  return movement;
}
void freeAllMovements()
{
  for (int i = 0; i < (int)movement_vector.size(); ++i)
  {
    free(movement_vector.at(i));
  }
}
void displayTrajectoryHelp()
{
  std::cout << "--- Trajectory Menu ------------------------------" << std::endl;
  std::cout << "?: Display this help     o: Switch on/off         " << std::endl;
  std::cout << "v: Change velocity       a: Change acceleration   " << std::endl;
  std::cout << "p: Add PTP movement      l: Add LINEAR movement   " << std::endl;
  std::cout << "u: Unblock               Esc: Exit to main menu   " << std::endl;
  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << std::endl;
}
void displayMainHelp()
{
  std::cout << "--- Main Menu ------------------------------------" << std::endl;
  std::cout << "?: Display this help     o: Switch on/off         " << std::endl;
  std::cout << "t: Add trajectory        r: Run trajectory        " << std::endl;
  std::cout << "u: Unblock               f: Flush movebuffers     " << std::endl;
  std::cout << "Esc: Exit program                                 " << std::endl;
  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << std::endl;
}
void addTrajectory()
{
  std::cout << "Name of Trajectory to create / add? ";
  std::string name_str;
  std::cin >> name_str;
  char *name = new char(strlen(name_str.c_str()));
  strcpy(name, name_str.c_str());
  std::cout << std::endl;
  std::cout << "current velocity: " << velocity << ", current acceleration: " << acceleration << std::endl;
  std::cout << std::endl;
  displayTrajectoryHelp();
  bool loop = true;
  int input, new_value;
  TMovement *current_move;
  while (loop)
  {
    input = _getch();
    switch (input)
    {
      case 3:
      case 4:
      case 27:  // VK_ESCAPE
        loop = false;
        continue;
      case '?':
        displayTrajectoryHelp();
        break;
      case 'o':  // VK_O (motors off/on)
        if (isOff)
        {
          allMotorsOn();
          isOff = false;
          std::cout << "Motors on" << std::endl
                    << std::endl;
        }
        else
        {
          allMotorsOff();
          isOff = true;
          std::cout << "Motors off" << std::endl
                    << std::endl;
        }
        break;
      case 'v':  // VK_V (change velocity)
        std::cout << "New velocity: ";
        std::cin >> new_value;
        if ((new_value >= 10) && (new_value <= 200))
        {
          velocity = new_value;
          std::cout << "   ... OK" << std::endl
                    << std::endl;
        }
        else
        {
          std::cout << "   ... FAILED, has to be min. 10 and max. 200" << std::endl
                    << std::endl;
        }
        break;
      case 'a':  // VK_A (change acceleration)
        std::cout << "New acceleration: ";
        std::cin >> new_value;
        if ((new_value >= 1) && (new_value <= 2))
        {
          acceleration = new_value;
          std::cout << "   ... OK" << std::endl
                    << std::endl;
        }
        else
        {
          std::cout << "   ... FAILED, has to be min. 1 and max. 2" << std::endl
                    << std::endl;
        }
        break;
      case 'p':  // VK_P (add PTP movement to here)
        current_move = allocateMovement();
        getPosition(&(current_move->pos));
        current_move->transition = PTP;
        current_move->velocity = velocity;
        current_move->acceleration = acceleration;
        pushMovementToStack(current_move, name);
        std::cout << "Added PTP movement with v=" << velocity << " and a=" << acceleration << " to point:" << std::endl;
        std::cout.precision(3);
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout << " X=" << current_move->pos.X;
        std::cout << ", Y=" << current_move->pos.Y;
        std::cout << ", Z=" << current_move->pos.Z;
        std::cout << ", phi=" << current_move->pos.phi;
        std::cout << ", theta=" << current_move->pos.theta;
        std::cout << ", psi=" << current_move->pos.psi;
        std::cout << std::endl
                  << std::endl;
        break;
      case 'l':  // VK_L (add LINEAR movement to here)
        current_move = allocateMovement();
        getPosition(&(current_move->pos));
        current_move->transition = LINEAR;
        current_move->velocity = velocity;
        current_move->acceleration = acceleration;
        pushMovementToStack(current_move, name);
        std::cout << "Added LINEAR movement with v=" << velocity << " and a=" << acceleration
                  << " to point:" << std::endl;
        std::cout.precision(3);
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout << " X=" << current_move->pos.X;
        std::cout << ", Y=" << current_move->pos.Y;
        std::cout << ", Z=" << current_move->pos.Z;
        std::cout << ", phi=" << current_move->pos.phi;
        std::cout << ", theta=" << current_move->pos.theta;
        std::cout << ", psi=" << current_move->pos.psi;
        std::cout << std::endl
                  << std::endl;
        break;
      case 'u':  // VK_U (unblock)
        unblock();
        std::cout << "Unblocked" << std::endl
                  << std::endl;
        break;
      default:  // Error message
        std::cout << "'" << input << "' is not a valid command." << std::endl
                  << std::endl;
        break;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "usage: wrapper_control CONFIGFILE IP_ADDR" << std::endl;
    return 0;
  }
  try
  {
    initKatana(argv[1], argv[2]);
    calibrate(0);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
    return -1;
  }
  std::cout << "-------------------------------------------" << std::endl;
  std::cout << "success: katana initiated" << std::endl;
  std::cout << "-------------------------------------------" << std::endl;
  /////////////////////////////////////////////////////////////////////////
  try
  {
    displayMainHelp();
    bool loop = true;
    int input, repetitions;
    std::string name_str;
    char *name;
    while (loop)
    {
      input = _getch();
      switch (input)
      {
        case 3:
        case 4:
        case 27:  // VK_ESCAPE
          loop = false;
          continue;
        case '?':
          displayMainHelp();
          break;
        case 'o':  // VK_O (motors off/on)
          if (isOff)
          {
            allMotorsOn();
            isOff = false;
            std::cout << "Motors on" << std::endl
                      << std::endl;
          }
          else
          {
            allMotorsOff();
            isOff = true;
            std::cout << "Motors off" << std::endl
                      << std::endl;
          }
          break;
        case 't':  // VK_O (DK to screen)
          addTrajectory();
          displayMainHelp();
          break;
        case 'r':
          std::cout << "Name of Trajectory to run? ";
          std::cin >> name_str;
          name = new char(strlen(name_str.c_str()));
          strcpy(name, name_str.c_str());
          std::cout << std::endl;
          std::cout << "Number of repetitions? ";
          std::cin >> repetitions;
          std::cout << std::endl;
          runThroughMovementStack(name, repetitions);
          displayMainHelp();
          break;
        case 'u':  // VK_U (unblock)
          unblock();
          std::cout << "Unblocked" << std::endl
                    << std::endl;
          break;
        case 'f':  // VK_F (flush)
          flushMoveBuffers();
          std::cout << "MoveBuffers flushed" << std::endl
                    << std::endl;
          break;
        default:  // Error message
          std::cout << "\n'" << input << "' is not a valid command.\n" << std::endl
                    << std::endl;
          break;
      }
    }

    // simple movement (was first try)
    /*TMovement *current_move = allocateMovement();
    getPosition(&(current_move->pos));
    current_move->transition = PTP;
    current_move->velocity = velocity;
    current_move->acceleration = acceleration;
    current_move->pos.Z += 50;
    executeMovement(current_move);*/
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
    freeAllMovements();
    return -1;
  }
  freeAllMovements();
  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
