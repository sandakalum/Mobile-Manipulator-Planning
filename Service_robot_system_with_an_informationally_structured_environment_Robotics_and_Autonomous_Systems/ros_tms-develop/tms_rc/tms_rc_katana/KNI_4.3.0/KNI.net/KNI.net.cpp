// This is the main DLL file.

#include "stdafx.h"

#include "KNI.net.h"

#include <vcclr.h>
#include <string>
#include <exception>

namespace KNInet
{
bool To_string(System::String ^ source, std::string &target)
{
  int len = ((source->Length + 1) * 2);
  char *ch = new char[len];
  bool result;
  {
    pin_ptr< const wchar_t > wch = PtrToStringChars(source);
    result = wcstombs(ch, wch, len) != -1;
  }
  target = ch;
  delete ch;
  return result;
}
////////////////////////////////////////////////////////////////////////////////////////
Katana::~Katana()
{
  if (katana)
    delete katana;
  if (proto)
    delete proto;
  if (comm)
    delete comm;
  if (socket)
    delete socket;
}
////////////////////////////////////////////////////////////////////////////////////////
Katana::Katana(System::String ^ ipAddress, System::String ^ configurationFile) : katana(0), socket(0), proto(0)
{
  std::string ip, portno, configFile;
  To_string(ipAddress, ip);
  // To_string(port, portno);
  To_string(configurationFile, configFile);
  try
  {
    socket = new CCdlSocket(const_cast< char * >(ip.c_str()), atoi("5566"));
    proto = new CCplSerialCRC();
    proto->init(socket);

    katana = new CLMBase();
    katana->create(configFile.c_str(), proto);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
// Katana::Katana(System::String ^connectionString, System::String ^configurationFile) :
//	katana(0), comm(0), proto(0) {
//	std::string connString, configFile;
//	To_string(connectionString, connString);
//	To_string(configurationFile, configFile);

//	try {
//		TCdlCOMDesc ccd = {atoi(connString.c_str()), 57600, 8, 'N', 1, 300, 0};
//		comm = new CCdlCOM(ccd);

//		proto = new CCplSerialCRC();
//		proto->init(comm);

//		katana = new CLMBase();
//		katana->create(configFile.c_str(), proto);
//	} catch(std::exception &e) {
//		throw gcnew System::Exception(gcnew System::String(e.what()));
//	}
//}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::calibrate(void)
{
  try
  {
    katana->calibrate();
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
array< int > ^ Katana::getRobotEncoders(bool refreshEncoders)
{
  std::vector< int > encodersVec(katana->getNumberOfMotors(), 0);
  array< int > ^ encoders = gcnew array< int >(encodersVec.size());
  try
  {
    katana->getRobotEncoders(encodersVec.begin(), encodersVec.end(), refreshEncoders);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
  for (unsigned int i = 0; i < encodersVec.size(); ++i)
  {
    encoders[i] = encodersVec[i];
  }
  return encoders;
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::moveRobotToEnc(array< int > ^ encoders, bool waitUntilReached, int waitTimeout)
{
  std::vector< int > encoderVector(katana->getNumberOfMotors(), 0);
  for (int i = 0; i < katana->getNumberOfMotors(); ++i)
  {
    encoderVector[i] = encoders[i];
  }
  try
  {
    katana->moveRobotToEnc(encoderVector, waitUntilReached, waitTimeout);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::moveMotorToEnc(int motor, int encoder, bool waitUntilReached, int waitTimeout)
{
  try
  {
    katana->moveMotorToEnc(motor, encoder, waitUntilReached, waitTimeout);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
array< double > ^ Katana::getCoordinates(bool refreshEncoders)
{
  std::vector< double > coordinateVector(6, 0);
  try
  {
    katana->getCoordinates(coordinateVector[0], coordinateVector[1], coordinateVector[2], coordinateVector[3],
                           coordinateVector[4], coordinateVector[5], refreshEncoders);
  }
  catch (Exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }

  array< double > ^ coordinates = gcnew array< double >(6);
  for (unsigned int i = 0; i < 6; ++i)
  {
    coordinates[i] = coordinateVector[i];
  }
  return coordinates;
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::moveRobotTo(array< double > ^ coordinates, bool waitUntilReached, int waitTimeout)
{
  std::vector< double > coordinateVector(6, 0);
  for (unsigned int i = 0; i < 6; ++i)
  {
    coordinateVector[i] = coordinates[i];
  }
  try
  {
    katana->moveRobotTo(coordinateVector, waitUntilReached, waitTimeout);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::moveRobotLinearTo(array< double > ^ coordinates, bool waitUntilReached, int waitTimeout)
{
  std::vector< double > coordinateVector(6, 0);
  for (unsigned int i = 0; i < 6; ++i)
  {
    coordinateVector[i] = coordinates[i];
  }
  try
  {
    katana->moveRobotLinearTo(coordinateVector, waitUntilReached, waitTimeout);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::setMaximumLinearVelocity(double maximumVelocity)
{
  katana->setMaximumLinearVelocity(maximumVelocity);
}
////////////////////////////////////////////////////////////////////////////////////////
double Katana::getMaximumLinearVelocity()
{
  return katana->getMaximumLinearVelocity();
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::setActivatePositionController(bool activate)
{
  katana->setActivatePositionController(activate);
}
////////////////////////////////////////////////////////////////////////////////////////
bool Katana::getActivatePositionController()
{
  return katana->getActivatePositionController();
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::enableCollisionLimits()
{
  try
  {
    katana->enableCrashLimits();
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::disableCollisionLimits()
{
  try
  {
    katana->disableCrashLimits();
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::unBlock()
{
  try
  {
    katana->unBlock();
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
void Katana::setCollisionLimit(int number, int limit)
{
  try
  {
    katana->setCrashLimit(number, limit);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
int Katana::getNumberOfMotors()
{
  return katana->getNumberOfMotors();
}
////////////////////////////////////////////////////////////////////////////////////////
int Katana::getMotorEncoders(int number, bool refreshEncoders)
{
  try
  {
    return katana->getMotorEncoders(number, refreshEncoders);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
int Katana::getMotorVelocityLimit(int number)
{
  return katana->getMotorVelocityLimit(number);
}
////////////////////////////////////////////////////////////////////////////////////////
int Katana::getMotorAccelerationLimit(int number)
{
  return katana->getMotorAccelerationLimit(number);
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::setMotorVelocityLimit(int number, int velocity)
{
  katana->setMotorVelocityLimit(number, velocity);
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::setMotorAccelerationLimit(int number, int acceleration)
{
  katana->setMotorAccelerationLimit(number, acceleration);
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::openGripper(bool waitUntilReached, int waitTimeout)
{
  try
  {
    katana->openGripper(waitUntilReached, waitTimeout);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::closeGripper(bool waitUntilReached, int waitTimeout)
{
  try
  {
    katana->closeGripper(waitUntilReached, waitTimeout);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::freezeRobot()
{
  try
  {
    katana->freezeRobot();
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::freezeMotor(int number)
{
  try
  {
    katana->freezeMotor(number);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::switchRobotOn()
{
  try
  {
    katana->switchRobotOn();
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::switchRobotOff()
{
  try
  {
    katana->switchRobotOff();
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::switchMotorOn(int number)
{
  try
  {
    katana->switchMotorOn(number);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void Katana::switchMotorOff(int number)
{
  try
  {
    katana->switchMotorOff(number);
  }
  catch (std::exception &e)
  {
    throw gcnew System::Exception(gcnew System::String(e.what()));
  }
}
////////////////////////////////////////////////////////////////////////////////////////
}
