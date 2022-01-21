/**********************************************************************************
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005-2009 Neuronics AG
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
// test.cpp
// test program for KNI libraries
//////////////////////////////////////////////////////////////////////////////////

#include "kniBase.h"
#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <fstream>
//////////////////////////////////////////////////////////////////////////////////
#define LEFT false
#define RIGHT true
//////////////////////////////////////////////////////////////////////////////////

const double PI = 3.14159265358979323846;

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
//////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "---------------------------------\n";
    std::cout << "usage for socket connection: kni_test CONFIGFILE IP_ADDR\n";
    std::cout << "---------------------------------\n";
    return 0;
  }

  std::cout << "-----------------\n";
  std::cout << "KNI_TEST DEMO STARTED\n";
  std::cout << "-----------------\n";

  //----------------------------------------------------------------//
  // open device:
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
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
    return -1;
  }
  std::cout << "-------------------------------------------\n";
  std::cout << "success: katana initialized\n";
  std::cout << "-------------------------------------------\n";

  // set linear velocity to 60
  katana->setMaximumLinearVelocity(60);

  // declare information variables
  int nOfMot = katana->getNumberOfMotors();  // number of motors
  int controller;                            // controller type: 0 for position, 1 for current
  const char *model;                         // katana model name
  std::vector< TPoint > points(0);           // list of points used for motion

  // declare temporary variables
  std::vector< int > encoders(nOfMot, 0);
  std::vector< double > pose(6, 0.0);
  TCurrentMot mot[6];
  for (int i = 0; i < 6; i++)
  {
    mot[i].running = false;
    mot[i].idx = i;
    mot[i].dir = RIGHT;
  }
  int tmp_int;

  // calibration
  std::cout << "- Calibrating Katana, please wait for termination..." << std::endl;
  katana->calibrate();
  std::cout << "   ...done." << std::endl;

  // get controller information
  std::cout << "- Check if Katana has position or current controllers..." << std::endl;
  controller = katana->getCurrentControllerType(1);
  for (short motorNumber = 1; motorNumber < nOfMot; ++motorNumber)
  {
    tmp_int = katana->getCurrentControllerType(motorNumber + 1);
    if (tmp_int != controller)
    {
      std::cout << "*** ERROR: Katana has mixed controller types on its axes! ***" << std::endl;
      return 1;
    }
  }
  std::cout << "   Katana has all ";
  controller == 0 ? std::cout << "position" : std::cout << "current";
  std::cout << " controllers." << std::endl;
  std::cout << "   ...done." << std::endl;

  // read current force if current controller installed
  if (controller == 1)
  {
    std::cout << "- Read forces..." << std::endl;
    for (short motorNumber = 0; motorNumber < nOfMot; ++motorNumber)
    {
      std::cout << "   Motor " << (motorNumber + 1) << ": " << katana->getForce(motorNumber + 1) << std::endl;
    }
    std::cout << "   ...done." << std::endl;
  }

  // get Katana model name
  std::cout << "- Get Katana model name..." << std::endl;
  model = katana->GetBase()->GetGNL()->modelName;
  std::cout << "   " << model << std::endl;
  std::cout << "   ...done." << std::endl;

  // switch motors off
  std::cout << "- Switch robot off..." << std::endl;
  katana->switchRobotOff();
  std::cout << "   Robot off" << std::endl;
  std::cout << "   ...done." << std::endl;

  // get robot encoders
  std::cout << "- Read robot encoders..." << std::endl;
  std::cout << "   Encoder values:";
  katana->getRobotEncoders(encoders.begin(), encoders.end());
  for (std::vector< int >::iterator i = encoders.begin(); i != encoders.end(); ++i)
  {
    std::cout << " " << *i;
  }
  std::cout << std::endl;
  std::cout << "   ...done." << std::endl;

  // switch motors on
  std::cout << "- Switch motors on..." << std::endl;
  for (short motorNumber = 0; motorNumber < nOfMot; ++motorNumber)
  {
    katana->switchMotorOn((short)motorNumber);
    std::cout << "   Motor " << (motorNumber + 1) << " on" << std::endl;
  }
  std::cout << "   ...done." << std::endl;

  // move single axes (inc, dec, mov, incDegrees, decDegrees, movDegrees, moveMotorByEnc, moveMotorBy, moveMotorToEnc,
  // moveMotorTo)
  std::cout << "- Move single axes..." << std::endl;
  katana->dec(0, 10000, true);
  std::cout << "   Motor 1 decreased by 10000 encoders" << std::endl;
  katana->inc(1, 10000, true);
  std::cout << "   Motor 2 increased by 10000 encoders" << std::endl;
  katana->decDegrees(2, 70, true);
  std::cout << "   Motor 3 decreased by 70 degrees" << std::endl;
  katana->mov(3, 20000, true);
  std::cout << "   "
            << "Motor 4 moved to encoder position 20000" << std::endl;
  katana->movDegrees(4, 90, true);
  std::cout << "   Motor 5 moved to position 90 degrees" << std::endl;
  katana->incDegrees(5, -35, true);
  std::cout << "   Motor 6 increased by -35 degrees" << std::endl;
  katana->moveMotorBy(0, 0.2, true);
  std::cout << "   Motor 1 moved by 0.2 rad" << std::endl;
  katana->moveMotorByEnc(1, -5000, true);
  std::cout << "   Motor 2 moved by -5000 encoders" << std::endl;
  katana->moveMotorTo(2, 3.1, true);
  std::cout << "   Motor 3 moved to 3.1 rad" << std::endl;
  katana->moveMotorToEnc(3, 10000, true);
  std::cout << "   Motor 4 moved to 10000 encoders" << std::endl;
  std::cout << "   ...done." << std::endl;

  // move all axes (moveRobotToEnc)
  std::cout << "- Move all axes..." << std::endl;
  for (short motorNumber = 0; motorNumber < nOfMot; ++motorNumber)
  {
    encoders[motorNumber] = 30500;
    if (motorNumber == 1 || motorNumber == 2)
      encoders[motorNumber] = -30500;
  }
  katana->moveRobotToEnc(encoders, true);
  std::cout << "   Robot moved to encoder position 30500 -30500 -30500 30500 30500 30500" << std::endl;
  std::cout << "   ...done." << std::endl;

  // get coordinates
  std::cout << "- Get coordinates..." << std::endl;
  {
    double x, y, z, phi, theta, psi;
    katana->getCoordinates(x, y, z, phi, theta, psi);
    std::cout << "   Current coordinates: " << x << " " << y << " " << z << " " << phi << " " << theta << " " << psi
              << std::endl;
  }
  std::cout << "   ...done." << std::endl;

  // get coordinates from given encoders
  std::cout << "- Get coordinates from given encoders..." << std::endl;
  {
    encoders[0] = encoders[3] = encoders[4] = encoders[5] = 25000;
    encoders[1] = encoders[2] = -25000;
    katana->getCoordinatesFromEncoders(pose, encoders);
    std::cout << "   Encoders: 25000 -25000 -25000 25000 25000 25000" << std::endl;
    std::cout << "   Coordinates: " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << " " << pose[4]
              << " " << pose[5] << std::endl;
  }
  std::cout << "   ...done." << std::endl;

  // calculate inverse kinematics
  std::cout << "- Calculate inverse kinematics..." << std::endl;
  {
    katana->IKCalculate(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], encoders.begin());
    std::cout << "   encoders.size(): " << encoders.size() << std::endl;
    std::cout << "   Possible encoders: " << encoders[0] << " " << encoders[1] << " " << encoders[2] << " "
              << encoders[3] << " " << encoders[4] << " " << encoders[5] << std::endl;
  }
  std::cout << "   ...done." << std::endl;

  // move robot to pose
  std::cout << "- Move robot to pose..." << std::endl;
  katana->moveRobotTo(pose, true);
  std::cout << "   Coordinates: " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << " " << pose[4]
            << " " << pose[5] << std::endl;
  std::cout << "   ...done." << std::endl;

  // load points file
  {
    using namespace std;
    cout << "- Load points file..." << endl;
    ifstream listfile(model);
    if (!listfile)
    {
      cout << "*** ERROR: File '" << model << "' not found or access denied! ***" << endl;
      return 1;
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
      point.phi = atof((tokens.at(3)).data());
      point.theta = atof((tokens.at(4)).data());
      point.psi = atof((tokens.at(5)).data());
      points.push_back(point);
      ++lines;
      tokens.clear();
    }
    cout << "   " << lines << " points loaded." << endl;
    cout << "   ...done." << endl;
  }

  // move to point in list
  std::cout << "- Move in point list..." << std::endl;
  {
    int i, j;
    bool wait;
    std::cout << "   Single p2p movements..." << std::flush;
    for (i = 0; i < (int)points.size(); ++i)
    {
      katana->moveRobotTo(points[i].X, points[i].Y, points[i].Z, points[i].phi, points[i].theta, points[i].psi);
    }
    std::cout << " ...done." << std::endl;
    std::cout << "   Single linear movements..." << std::flush;
    for (i = 0; i < (int)points.size(); ++i)
    {
      katana->moveRobotLinearTo(points[i].X, points[i].Y, points[i].Z, points[i].phi, points[i].theta, points[i].psi);
    }
    std::cout << " ...done." << std::endl;
    std::cout << "   Concatenated p2p movements..." << std::flush;
    for (i = 0; i < (int)points.size(); ++i)
    {
      j = (i + points.size() - 1) % points.size();
      wait = false;
      if (i == ((int)points.size() - 1))
        wait = true;
      katana->movP2P(points[j].X, points[j].Y, points[j].Z, points[j].phi, points[j].theta, points[j].psi, points[i].X,
                     points[i].Y, points[i].Z, points[i].phi, points[i].theta, points[i].psi, true, 70.0, wait);
    }
    std::cout << " ...done." << std::endl;
    std::cout << "   Concatenated linear movements..." << std::flush;
    for (i = 0; i < (int)points.size(); ++i)
    {
      j = (i + points.size() - 1) % points.size();
      wait = false;
      if (i == ((int)points.size() - 1))
        wait = true;
      katana->movLM2P(points[j].X, points[j].Y, points[j].Z, points[j].phi, points[j].theta, points[j].psi, points[i].X,
                      points[i].Y, points[i].Z, points[i].phi, points[i].theta, points[i].psi, true, 100.0, wait);
    }
    std::cout << " ...done." << std::endl;
  }
  std::cout << "   ...done." << std::endl;

  // move all axes (moveRobotToEnc)
  std::cout << "- Move all axes..." << std::endl;
  for (short motorNumber = 0; motorNumber < nOfMot; ++motorNumber)
  {
    encoders[motorNumber] = 30500;
    if (motorNumber == 1 || motorNumber == 2)
      encoders[motorNumber] = -30500;
  }
  katana->moveRobotToEnc(encoders, true);
  std::cout << "   Robot moved to encoder position 30500 -30500 -30500 30500 30500 30500" << std::endl;
  std::cout << "   ...done." << std::endl;

  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
