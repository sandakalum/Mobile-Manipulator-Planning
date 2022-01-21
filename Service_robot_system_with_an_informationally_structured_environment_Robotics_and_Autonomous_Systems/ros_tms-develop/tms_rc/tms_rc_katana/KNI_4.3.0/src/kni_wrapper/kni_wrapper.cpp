/**************************************************************************
 * kni_wrapper.cpp -
 * Implements a wrapper for the kni library so that the C++ library
 * can be accessed by C based environments (MatLab, LabView)
 * Copyright (C) Neuronics AG
 * Philipp Keller, Tino Perucchi, 2008
**************************************************************************/
#include <iostream>
#include <memory>
#include <math.h>
#include <vector>
#include <string.h>
#include <map>

#define EXPORT_FCNS
#include "kni_wrapper/kni_wrapper.h"

//#include <pthread.h>
#include <sstream>
/////////////////////////////////////////////////////////////////////////////////////////////////
// KNI internal types
static std::auto_ptr< CLMBase > katana;
static std::auto_ptr< CCdlSocket > device;
static std::auto_ptr< CCplSerialCRC > protocol;
/////////////////////////////////////////////////////////////////////////////////////////////////
// A map containing movement vectors accessible by a string name:
std::map< std::string, std::vector< TMovement > > movements;
// A vector for storing encoders
std::vector< int > encoders;
// The number of motors, initialized in initKatana()
int numberOfMotors;
// variables to store communication data
byte packet[32];   // comm packet
byte buffer[256];  // comm readbuf
byte size = 0;     // comm readbuf size
/////////////////////////////////////////////////////////////////////////////////////////////////

DLLEXPORT int initKatana(char *configFile, char *ipaddress)
{
  try
  {
    int port = 5566;
    device.reset(new CCdlSocket(ipaddress, port));
    protocol.reset(new CCplSerialCRC());
    protocol->init(device.get());
    katana.reset(new CLMBase());
    katana->create(configFile, protocol.get());
    numberOfMotors = katana->getNumberOfMotors();
    for (int i = 0; i < numberOfMotors; i++)
    {
      encoders.push_back(0);
    }
    // MessageBox(NULL, "Katana successfully initiated!",TEXT(""),MB_OK);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int calibrate(int axis)
{
  try
  {
    katana->calibrate();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int moveMot(int axis, int enc, int speed, int accel)
{
  try
  {
    // set speed
    katana->setMotorVelocityLimit(axis - 1, speed);
    // set acceleration
    katana->setMotorAccelerationLimit(axis - 1, accel);
    // move
    katana->moveMotorToEnc(axis - 1, enc);
  }
  catch (...)
  {
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int waitForMot(int axis, int targetpos, int tolerance)
{
  try
  {
    katana->waitForMotor((short)axis - 1, targetpos, tolerance, 0, TM_ENDLESS);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int moveMotAndWait(int axis, int targetpos, int tolerance)
{
  try
  {
    katana->moveMotorToEnc(axis - 1, targetpos);
    waitForMot(axis, targetpos, tolerance);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int motorOn(int axis)
{
  try
  {
    katana->switchMotorOn(axis - 1);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int motorOff(int axis)
{
  try
  {
    katana->switchMotorOff(axis - 1);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int allMotorsOff()
{
  try
  {
    katana->switchRobotOff();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int allMotorsOn()
{
  try
  {
    katana->switchRobotOn();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setGripper(bool hasGripper)
{
  try
  {
    // cannot extract configured values from KNI atm.
    int openEncoders = 30770;
    int closeEncoders = 12240;

    katana->setGripperParameters(hasGripper, openEncoders, closeEncoders);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int closeGripper()
{
  try
  {
    katana->closeGripper();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int openGripper()
{
  try
  {
    katana->openGripper();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int moveToPosEnc(int enc1, int enc2, int enc3, int enc4, int enc5, int enc6, int velocity, int acceleration,
                           int tolerance, bool _wait)
{
  std::cout << "moving to: " << enc1 << ", " << enc2 << ", " << enc3 << ", " << enc4 << ", " << enc5 << ", " << enc6
            << "\n";
  try
  {
    for (int i = 0; i < numberOfMotors; i++)
    {
      // set speed
      katana->setMotorVelocityLimit(i, velocity);
      // set acceleration
      katana->setMotorAccelerationLimit(i, acceleration);
    }
    // move
    std::vector< int > enc;
    enc.push_back(enc1);
    enc.push_back(enc2);
    enc.push_back(enc3);
    enc.push_back(enc4);
    enc.push_back(enc5);
    enc.push_back(enc6);
    katana->moveRobotToEnc(enc.begin(), enc.end(), _wait, tolerance);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int sendSplineToMotor(int axis, int targetpos, int duration, int p0, int p1, int p2, int p3)
{
  try
  {
    katana->sendSplineToMotor((unsigned short)(axis - 1), (short)targetpos, (short)duration, p0, p1, p2, p3);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int startSplineMovement(int contd, int exactflag)
{
  try
  {
    bool exact = (exactflag != 0);
    katana->startSplineMovement(exact, contd);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int flushMoveBuffers()
{
  try
  {
    katana->flushMoveBuffers();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int getPosition(struct TPos *pos)
{
  try
  {
    katana->getCoordinates(pos->X, pos->Y, pos->Z, pos->phi, pos->theta, pos->psi);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int moveToPos(struct TPos *pos, int velocity, int acceleration)
{
  try
  {
    setMaxAccel(0, acceleration);
    setMaxVelocity(0, velocity);
    katana->moveRobotTo(pos->X, pos->Y, pos->Z, pos->phi, pos->theta, pos->psi);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int moveToPosLin(struct TPos *targetPos, int velocity, int acceleration)
{
  try
  {
    setMaxAccel(0, acceleration);
    setMaxVelocity(0, velocity);
    katana->moveRobotLinearTo(targetPos->X, targetPos->Y, targetPos->Z, targetPos->phi, targetPos->theta,
                              targetPos->psi);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int pushMovementToStack(struct TMovement *movement, char *name)
{
  try
  {
    // name as string
    std::string name_str(name);
    // get according movement vector, a new entry is created and inserted
    // automatically if it does not exist and store movement in it
    movements[name_str].push_back(*movement);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int deleteMovementFromStack(char *name, int index)
{
  try
  {
    int status = ERR_SUCCESS;
    // name as string
    std::string name_str(name);
    // find movement vector
    std::map< std::string, std::vector< TMovement > >::iterator it;
    it = movements.find(name_str);
    if (it == movements.end())
    {
      // movement vector does not exist
      status = ERR_FAILED;
    }
    else
    {
      // movement vector exists
      std::vector< TMovement > movement_vector = (*it).second;
      if ((int)movement_vector.size() >= index)
      {
        // index out of range
        status = ERR_FAILED;
      }
      else
      {
        // erase movement
        movement_vector.erase(movement_vector.begin() + index);
        status = ERR_SUCCESS;
      }
    }
    if (status == ERR_FAILED)
    {
      return ERR_FAILED;
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int deleteMovementStack(char *name)
{
  try
  {
    int status = ERR_SUCCESS;
    // name as string
    std::string name_str(name);
    // find movement vector
    std::map< std::string, std::vector< TMovement > >::iterator it;
    it = movements.find(name_str);
    if (it == movements.end())
    {
      // movement vector does not exist
      status = ERR_FAILED;
    }
    else
    {
      // movement vector exists, erase it
      movements.erase(it);
      status = ERR_SUCCESS;
    }
    if (status == ERR_FAILED)
    {
      return ERR_FAILED;
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int runThroughMovementStack(char *name, int loops)
{
  try
  {
    int status = ERR_SUCCESS;
    // name as string
    std::string name_str(name);
    // find movement vector
    std::map< std::string, std::vector< TMovement > >::iterator it;
    it = movements.find(name_str);
    if (it == movements.end())
    {
      // movement vector does not exist
      status = ERR_FAILED;
    }
    else
    {
      // movement vector exists
      std::vector< TMovement > movement_vector = (*it).second;
      // execute all movements in movement vector
      int size = (int)movement_vector.size();
      if (size == 1)
      {
        status = executeMovement(&(movement_vector.at(0)));
      }
      else
      {
        bool first, last;
        struct TPos *lastpos;
        for (int j = 0; j < loops; ++j)
        {
          last = false;
          for (int i = 0; i < size; ++i)
          {
            if (i == 0)
            {
              first = true;
              lastpos = NULL;
            }
            else
            {
              first = false;
              lastpos = &((movement_vector.at(i - 1)).pos);
            }
            if (i == (size - 1))
              last = true;
            status = executeConnectedMovement(&(movement_vector.at(i)), lastpos, first, last);
            if (status == ERR_FAILED)
            {
              break;
            }
          }
        }
      }
    }
    if (status == ERR_FAILED)
    {
      return ERR_FAILED;
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int executeMovement(struct TMovement *movement)
{
  int status = ERR_SUCCESS;
  if (movement->transition == PTP)
  {
    // move PTP
    status = moveToPos(&(movement->pos), movement->velocity, movement->acceleration);
  }
  else if (movement->transition == LINEAR)
  {
    // move linear
    status = moveToPosLin(&(movement->pos), movement->velocity, movement->acceleration);
  }
  else
  {
    status = ERR_FAILED;
  }
  return status;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int executeConnectedMovement(struct TMovement *movement, struct TPos *startPos, bool first, bool last)
{
  int status = ERR_SUCCESS;
  setMaxAccel(0, movement->acceleration);
  setMaxVelocity(0, movement->velocity);
  bool wait = last;  // only wait at the last of the connected movements
  if (movement->transition == PTP)
  {
    // move PTP
    try
    {
      if (first)
      {
        // first of the connected movements, no startpos known
        katana->moveRobotTo(movement->pos.X, movement->pos.Y, movement->pos.Z, movement->pos.phi, movement->pos.theta,
                            movement->pos.psi, wait);
      }
      else
      {
        // startpos known
        katana->movP2P(startPos->X, startPos->Y, startPos->Z, startPos->phi, startPos->theta, startPos->psi,
                       movement->pos.X, movement->pos.Y, movement->pos.Z, movement->pos.phi, movement->pos.theta,
                       movement->pos.psi, true, movement->velocity, wait);
      }
    }
    catch (...)
    {
      status = ERR_FAILED;
    }
  }
  else if (movement->transition == LINEAR)
  {
    // move linear
    try
    {
      if (first)
      {
        // first of the connected movements, no startpos known
        katana->moveRobotLinearTo(movement->pos.X, movement->pos.Y, movement->pos.Z, movement->pos.phi,
                                  movement->pos.theta, movement->pos.psi, wait);
      }
      else
      {
        // startpos known
        katana->movLM2P(startPos->X, startPos->Y, startPos->Z, startPos->phi, startPos->theta, startPos->psi,
                        movement->pos.X, movement->pos.Y, movement->pos.Z, movement->pos.phi, movement->pos.theta,
                        movement->pos.psi, true, movement->velocity, wait);
      }
    }
    catch (...)
    {
      status = ERR_FAILED;
    }
  }
  else
  {
    status = ERR_FAILED;
  }
  return status;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int ModBusTCP_writeWord(int address, int value)
{
  try
  {
    byte packet[32];
    byte size;
    packet[0] = 'M';
    packet[1] = 'W';
    packet[2] = (byte)address;
    packet[3] = (byte)(value >> 8);
    packet[4] = (byte)value;
    protocol->comm(packet, buffer, &size);
    if (!buffer[0] || ((short)size != 4))
    {
      return ERR_FAILED;
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int ModBusTCP_readWord(int address, int &value)
{
  try
  {
    byte packet[32];
    byte size;
    packet[0] = 'M';
    packet[1] = 'R';
    packet[2] = (byte)address;
    protocol->comm(packet, buffer, &size);
    if (!buffer[0] || ((short)size != 4))
    {
      return ERR_FAILED;
    }
    int val = buffer[2];
    val <<= 8;
    val += buffer[1];
    value = val;
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int IO_setOutput(char output, int value)
{
  try
  {
    byte packet[32];
    byte size;
    packet[0] = 'T';
    packet[1] = 'w';
    packet[2] = (byte)value << output;
    packet[3] = 0;
    packet[4] = 0;
    protocol->comm(packet, buffer, &size);
    if (!buffer[0] || ((short)size != 2))
    {
      std::cout << " command failed!" << std::endl;
      return ERR_FAILED;
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int IO_readInput(int inputNr, int &value)
{
  try
  {
    byte packet[32];
    byte size;
    packet[0] = 'T';
    packet[1] = 'r';
    packet[2] = 0;
    packet[3] = 0;
    packet[4] = 0;
    protocol->comm(packet, buffer, &size);
    if (!buffer[0] || ((short)size != 2))
    {
      std::cout << " command failed!" << std::endl;
      return ERR_FAILED;
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  byte mask = static_cast< byte >(pow(static_cast< double >(2), inputNr - 1));
  if (buffer[1] != 0xFF)
  {
    if ((mask & (int)buffer[1]) > 0)
    {
      value = 0;
    }
    else
    {
      value = 1;
    }
    return ERR_SUCCESS;
  }
  else
    return ERR_FAILED;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int clearMoveBuffers()
{
  try
  {
    for (int i = 1; i <= getNumberOfMotors(); i++)
    {
      TMotTPS tps = {MCF_CLEAR_MOVEBUFFER, 0};
      katana->GetBase()->GetMOT()->arr[i].sendTPS(&tps);
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int getVelocity(int axis, int &value)
{
  try
  {
    katana->GetBase()->GetMOT()->arr[axis - 1].recvPVP();
    short vel = katana->GetBase()->GetMOT()->arr[axis - 1].GetPVP()->vel;
    value = (int)vel;
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int getDrive(int axis, int &value)
{
  try
  {
    katana->GetBase()->GetMOT()->arr[axis - 1].recvPVP();
    byte pwm = katana->GetBase()->GetMOT()->arr[axis - 1].GetPVP()->pwm;
    value = (int)pwm;
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int getEncoder(int axis, int &value)
{
  try
  {
    katana->getRobotEncoders(encoders.begin(), encoders.end());
    value = encoders.at(axis - 1);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int getAxisFirmwareVersion(int axis, char value[])
{
  try
  {
    katana->GetBase()->GetMOT()->arr[axis - 1].recvSFW();
    int v = (int)katana->GetBase()->GetMOT()->arr[axis - 1].GetSFW()->version;
    int sv = (int)katana->GetBase()->GetMOT()->arr[axis - 1].GetSFW()->subversion;
    int r = (int)katana->GetBase()->GetMOT()->arr[axis - 1].GetSFW()->revision;
    std::stringstream ss;
    ss << v << "." << sv << "." << r;
    const char *cstr = ss.str().c_str();
    int index = -1;
    do
    {
      index++;
      value[index] = cstr[index];
    } while (cstr[index] != '\0');
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int getVersion(char value[])
{
  try
  {
    katana->GetBase()->recvMFW();
    int v = (int)katana->GetBase()->GetMFW()->ver;
    int r = (int)katana->GetBase()->GetMFW()->rev;
    std::stringstream ss;
    ss << v << "." << r;
    const char *cstr = ss.str().c_str();
    int index = -1;
    do
    {
      index++;
      value[index] = cstr[index];
    } while (cstr[index] != '\0');
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setCollisionDetection(int axis, bool state)
{
  try
  {
    if (state == true)
    {
      katana->enableCrashLimits();
    }
    else
      katana->disableCrashLimits();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setPositionCollisionLimit(int axis, int limit)
{
  try
  {
    katana->setPositionCollisionLimit(axis - 1, limit);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setVelocityCollisionLimit(int axis, int limit)
{
  try
  {
    katana->setSpeedCollisionLimit(axis - 1, limit);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setCollisionParameters(int axis, int position, int velocity)
{
  try
  {
    setPositionCollisionLimit(axis, position);
    setVelocityCollisionLimit(axis, velocity);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setForceLimit(int axis, int limit)
{
  try
  {
    katana->setForceLimit(axis, limit);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int getForce(int axis)
{
  try
  {
    return katana->getForce(axis);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int getCurrentControllerType(int axis)
{
  try
  {
    return katana->getCurrentControllerType(axis);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int unblock()
{
  try
  {
    katana->unBlock();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setControllerParameters(int axis, int ki, int kspeed, int kpos)
{
  try
  {
    katana->GetBase()->GetMOT()->arr[axis - 1].setControllerParameters(kspeed, kpos, ki);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setMaxVelocity(int axis, int vel)
{
  try
  {
    if (axis == 0)
    {
      for (int i = 0; i <= getNumberOfMotors() - 1; i++)
      {
        katana->setMotorVelocityLimit(i, vel);
      }
    }
    else
    {
      katana->setMotorVelocityLimit(axis - 1, vel);
    }
    katana->setMaximumLinearVelocity((double)vel);
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int setMaxAccel(int axis, int acceleration)
{
  try
  {
    if (axis == 0)
    {
      for (int i = 0; i <= getNumberOfMotors() - 1; i++)
      {
        katana->setMotorAccelerationLimit(i, acceleration);
      }
    }
    else
    {
      katana->setMotorAccelerationLimit(axis - 1, acceleration);
    }
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int getNumberOfMotors()
{
  try
  {
    return katana->getNumberOfMotors();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DLLEXPORT int ping(int axis)
{
  try
  {
    // throws ParameterReadingException if unsuccessful
    katana->GetBase()->recvECH();
  }
  catch (Exception &e)
  {
    std::cout << "ERROR: " << e.message() << std::endl;
#ifdef WIN32
    MessageBox(NULL, TEXT(e.message().c_str()), TEXT("KNI ERROR"), MB_OK | MB_ICONWARNING | MB_SYSTEMMODAL);
#endif
    return ERR_FAILED;
  }
  return ERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
