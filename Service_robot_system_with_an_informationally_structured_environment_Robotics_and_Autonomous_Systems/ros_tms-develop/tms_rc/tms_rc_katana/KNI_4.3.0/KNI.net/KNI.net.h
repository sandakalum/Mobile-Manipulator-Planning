// KNI.net.h

#pragma once

#include "kniBase.h"
#include <memory>
namespace KNInet
{
public
ref class Katana
{
private:
  CLMBase* katana;
  CCdlCOM* comm;
  CCdlSocket* socket;
  CCplSerialCRC* proto;

public:
  // Katana(System::String ^connectionString, System::String ^configurationFile);
  Katana(System::String ^ ipAddress, System::String ^ configurationFile);
  ~Katana();

  void calibrate(void);

  array< int > ^ getRobotEncoders(bool refreshEncoders);
  void moveRobotToEnc(array< int > ^ encoders, bool waitUntilReached, int waitTimeout);
  void moveMotorToEnc(int motor, int encoder, bool waitUntilReached, int waitTimeout);
  array< double > ^ getCoordinates(bool refreshEncoders);
  void moveRobotTo(array< double > ^ coordinates, bool waitUntilReached, int waitTimeout);
  void moveRobotLinearTo(array< double > ^ coordinates, bool waitUntilReached, int waitTimeout);

  void setMaximumLinearVelocity(double maximumVelocity);
  double getMaximumLinearVelocity();

  void setActivatePositionController(bool activate);
  bool getActivatePositionController();

  void enableCollisionLimits();
  void disableCollisionLimits();
  void unBlock();
  void setCollisionLimit(int number, int limit);

  int getNumberOfMotors();
  int getMotorEncoders(int number, bool refreshEncoders);

  int getMotorVelocityLimit(int number);
  int getMotorAccelerationLimit(int number);
  void setMotorVelocityLimit(int number, int velocity);
  void setMotorAccelerationLimit(int number, int acceleration);

  void openGripper(bool waitUntilReached, int waitTimeout);
  void closeGripper(bool waitUntilReached, int waitTimeout);

  void freezeRobot();
  void freezeMotor(int number);
  void switchRobotOn();
  void switchRobotOff();
  void switchMotorOn(int number);
  void switchMotorOff(int number);
};
}
