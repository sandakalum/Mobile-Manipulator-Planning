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
// kni_wrapper.cpp
// demo program for the kni C wrapper dll
// PKE/JHA/TPE 2008
//////////////////////////////////////////////////////////////////////////////////
#include "kniBase.h"
#include "kni_wrapper/kni_wrapper.h"
#include "kniBase.h"
#include <iostream>
#include <cstdio>
#include <memory>
//////////////////////////////////////////////////////////////////////////////////
// defines
#ifdef WIN32
#include <conio.h>
#else  // LINUX
#include "keyboard.h"
#endif
//////////////////////////////////////////////////////////////////////////////////
// prototypes
void printInterface(std::string _interface);
//////////////////////////////////////////////////////////////////////////////////
// globals:
int enc[10], targetenc[10];
const int ENC_TOLERANCE = 10;
const int DEFAULT_ACCELERATION = 2;
const int DEFAULT_SPEED = 100;
const int POSITIONAL_OFFSET = 10000;
TPos *current_position, *target_position;
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  if (argc != 4)
  {
    std::cout << "usage: kni_wrapper CONFIGFILE IP_ADDR CYCLES" << std::endl;
    return 0;
  }
  int numberOfCycles = atoi(argv[3]);
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
  std::cout << "starting to call KNI interfaces" << std::endl;
  std::cout << "interfaces in brackets[] are only being called implicitly" << std::endl;
  std::cout << "-------------------------------------------" << std::endl;
  /////////////////////////////////////////////////////////////////////////
  try
  {
    int value = 0;
    for (int loops = 0; loops < numberOfCycles; loops++)
    {
      printInterface("IO_readInput(char output, int value)\n");
      IO_readInput(1, value);
      std::cout << "Read Input 1. Value: " << value << std::endl;
      //////////////////////////////////////////////////////////////////
      printInterface("[moveMot(int axis, int enc, int speed, int accel)],\n \
					getEncoder(int axis),\n \
					[waitForMot(int axis, int targetpos=0, int tolerance=0)]\n \
					moveMotAndWait(int axis, int targetpos, int tolerance=0)\n");
      for (int i = 1; i <= getNumberOfMotors(); i++)
      {
        getEncoder(i, value);
        enc[i - 1] = value;
        if (enc[i - 1] > 0)
        {
          targetenc[i - 1] = enc[i - 1] - POSITIONAL_OFFSET;
        }
        else
        {
          targetenc[i - 1] = enc[i - 1] + POSITIONAL_OFFSET;
        }
        moveMotAndWait(i, targetenc[i - 1], ENC_TOLERANCE);
        moveMotAndWait(i, enc[i - 1], ENC_TOLERANCE);
      }
      //////////////////////////////////////////////////////////////////
      printInterface("[motorOff(int axis)],\n \
					[motorOn(int axis)],\n \
					allMotorsOff()\n \
					allMotorsOn()\n");
      allMotorsOff();
      allMotorsOn();
      calibrate(0);
      //////////////////////////////////////////////////////////////////
      printInterface("closeGripper(), openGripper()\n");
      closeGripper();
      openGripper();
      //////////////////////////////////////////////////////////////////
      printInterface("moveToPosEnc(int enc1, int enc2, int enc3, int enc4, int enc5, int enc6, int velocity, int "
                     "acceleration, int tolerance)\n");
      for (int i = 0; i < getNumberOfMotors(); i++)
      {
        std::cout << "startencoder: " << enc[i] << ",\ttargetenc: " << targetenc[i] << std::endl;
      }
      moveToPosEnc(targetenc[0], targetenc[1], targetenc[2], targetenc[3], targetenc[4], targetenc[5], DEFAULT_SPEED,
                   DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
      moveToPosEnc(enc[0], enc[1], enc[2], enc[3], enc[4], enc[5], DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE,
                   true);
      //////////////////////////////////////////////////////////////////
      printInterface("getPosition(struct TPos &pos), moveToPos(struct TPos pos)\n");

      current_position = (struct TPos *)malloc(sizeof(TPos));
      target_position = (struct TPos *)malloc(sizeof(TPos));
      getPosition(current_position);

      moveToPosEnc(targetenc[0], targetenc[1], targetenc[2], targetenc[3], targetenc[4], targetenc[5], DEFAULT_SPEED,
                   DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
      getPosition(target_position);
      moveToPosEnc(enc[0], enc[1], enc[2], enc[3], enc[4], enc[5], DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE,
                   true);
      moveToPos(target_position, 80, 1);
      moveToPos(current_position, 80, 1);
      //////////////////////////////////////////////////////////////////
      printInterface("moveToPosLin(struct TPos startPos, struct TPos targetPos)\n");
      // moveToPos(target_position, 80, DEFAULT_ACCELERATION);
      moveToPosEnc(6355, -13513, -27931, 8500, 19832, 30420, DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
      TPos *tmpPos;
      tmpPos = (struct TPos *)malloc(sizeof(TPos));
      getPosition(tmpPos);
      tmpPos->Y = tmpPos->Y - 160.0;
      moveToPosLin(tmpPos, 100, DEFAULT_ACCELERATION);
      tmpPos->X = tmpPos->X - 300.0;
      moveToPosLin(tmpPos, 100, DEFAULT_ACCELERATION);
      tmpPos->Z = tmpPos->Z + 150;
      moveToPosLin(tmpPos, 100, DEFAULT_ACCELERATION);
      moveToPos(current_position, 100, DEFAULT_ACCELERATION);
      //////////////////////////////////////////////////////////////////
      printInterface("IO_setOutput(char output, int value)\n");
      IO_setOutput(0, 1);
      //////////////////////////////////////////////////////////////////
      printInterface("IO_readInput(char output, int value)\n");
      IO_readInput(1, value);
      std::cout << "Read Input 1. Value: " << value << std::endl;
      //////////////////////////////////////////////////////////////////
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
    return -1;
  }
  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
void printInterface(std::string _interface)
{
  std::cout << "Calling interface: " << _interface << std::endl;
}
//////////////////////////////////////////////////////////////////////////////////
