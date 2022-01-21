//////////////////////////////////////////////////////////////////////////////////
// mykni.cpp
// demo program training
//////////////////////////////////////////////////////////////////////////////////
#include "kniBase.h"
#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <fstream>
//////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
#include <conio.h>
#else  // LINUX
#include "keyboard.h"
#endif
#define LEFT false
#define RIGHT true
//////////////////////////////////////////////////////////////////////////////////
// Katana structs:
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
//////////////////////////////////////////////////////////////////////////////////
// Katana obj.
std::auto_ptr< CLMBase > katana;
int retVal = 0;
const double PI = 3.14159265358979323846;
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "---------------------------------\n";
    std::cout << "usage for socket connection: socketcontrol CONFIGFILE IP_ADDR\n";
    std::cout << "---------------------------------\n";
    return 0;
  }
  std::cout << "--------------------------\n";
  std::cout << "SOCKETCONTROL DEMO STARTED\n";
  std::cout << "--------------------------\n";
  std::auto_ptr< CCdlSocket > device;
  std::auto_ptr< CCplSerialCRC > protocol;
  try
  {
    int port = 5566;
    device.reset(new CCdlSocket(argv[2], port));
    std::cout << "-------------------------------------------\n";
    std::cout << "success:  port " << port << " open\n";
    std::cout << "-------------------------------------------\n";
    protocol.reset(new CCplSerialCRC());
    std::cout << "-------------------------------------------\n";
    std::cout << "success: protocol class instantiated\n";
    std::cout << "-------------------------------------------\n";
    protocol->init(device.get());  // fails if no response from Katana
    std::cout << "-------------------------------------------\n";
    std::cout << "success: communication with Katana initialized\n";
    std::cout << "-------------------------------------------\n";
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
  std::vector< int > encoders(katana->getNumberOfMotors(), 0);
  TCurrentMot mot[6];
  for (int i = 0; i < 6; i++)
  {
    mot[i].running = false;
    mot[i].idx = i;
    mot[i].dir = true;
    mot[i].dir = RIGHT;
  }
  // set linear velocity to 60
  try
  {
    katana->setMaximumLinearVelocity(60);
    katana->calibrate();
    //....continue here
  }
  catch (...)
  {
    std::cout << "-------------------------------------------\n";
    std::cout << "An exception occurred while communicating with the Katana.\n";
    std::cout << "-------------------------------------------\n";
  }
  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
