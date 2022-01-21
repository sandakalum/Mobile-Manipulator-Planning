///-----------------------------------------------------------------------------
/// @FileName corba_cmd_tag.cpp
/// @Date 2013.05.27
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include "corba_client.h"

#define BOOL(b) (b ? "Success" : "Failure")

//------------------------------------------------------------------------------
bool Client::TagInit(void)
{
  if (!bInitialize)
  {
    printf("Initialization has not been finished.\n");
    return 0;
  }

  bool ret[5] = {false};

  // ret  = true	: Success
  // 		= false : Failure
  ret[0] = CommandObj_TagF->intialize();
  ret[1] = CommandObj_TagB->intialize();
  ret[2] = CommandObj_TagL->intialize();
  ret[3] = CommandObj_TagR->intialize();

  ret[4] = ret[0] && ret[1] && ret[2] && ret[3];
  printf("TagInit Result: %s, %s, %s, %s\n", BOOL(ret[0]), BOOL(ret[1]), BOOL(ret[2]), BOOL(ret[3]));

  return ret[4];
}

//------------------------------------------------------------------------------
bool Client::TagSetConfig(int conNo, int confVal)
{
  if (!bInitialize)
  {
    printf("Initialization has not been finished.\n");
    return 0;
  }

  bool ret[5] = {false};

  corbaD1 = (CORBA::Double)conNo;
  corbaD2 = (CORBA::Double)confVal;

  // ret  = true	: Success
  // 		= false : Failure
  ret[0] = CommandObj_TagF->setConfig(corbaD1, corbaD2);
  ret[1] = CommandObj_TagB->setConfig(corbaD1, corbaD2);
  ret[2] = CommandObj_TagL->setConfig(corbaD1, corbaD2);
  ret[3] = CommandObj_TagR->setConfig(corbaD1, corbaD2);

  ret[4] = ret[0] && ret[1] && ret[2] && ret[3];

  printf("TagSetConfig(%f, %f) Result: %s, %s, %s, %s\n", corbaD1, corbaD2, BOOL(ret[0]), BOOL(ret[1]), BOOL(ret[2]),
         BOOL(ret[3]));

  return ret[4];
}

//------------------------------------------------------------------------------
bool Client::TagSetPower(int power)
{
  if (!bInitialize)
  {
    printf("Initialization has not been finished.\n");
    return 0;
  }

  bool ret[5] = {false};

  corbaD1 = (CORBA::Double)power;

  // ret  = true	: Success
  // 		= false : Failure
  ret[0] = CommandObj_TagF->setPower(corbaD1);
  ret[1] = CommandObj_TagB->setPower(corbaD1);
  ret[2] = CommandObj_TagL->setPower(corbaD1);
  ret[3] = CommandObj_TagR->setPower(corbaD1);

  ret[4] = ret[0] && ret[1] && ret[2] && ret[3];

  printf("TagSetPower(%f) Result: %s, %s, %s, %s\n", corbaD1, BOOL(ret[0]), BOOL(ret[1]), BOOL(ret[2]), BOOL(ret[3]));

  return ret[0];
}

//------------------------------------------------------------------------------
bool Client::TagInventory(unsigned char *size, unsigned char (*data)[128])
{
  if (!bInitialize)
  {
    printf("Initialization has not been finished.\n");
    return -1;
  }

  TagData_var tagout;
  tagout = new TagData;
  CORBA::Octet *buf;

  bool ret[5] = {false};

  //--------------------------------------------------------------------------
  ret[0] = CommandObj_TagF->inventory(tagout);
  buf = tagout->get_buffer();
  size[0] = (unsigned int)tagout->length();

  for (int i = 0; i < (int)tagout->length(); i++)
  {
    data[0][i] = buf[i];
  }

  //--------------------------------------------------------------------------
  ret[1] = CommandObj_TagB->inventory(tagout);
  buf = tagout->get_buffer();
  size[1] = (unsigned int)tagout->length();

  for (int i = 0; i < (int)tagout->length(); i++)
  {
    data[1][i] = buf[i];
  }

  //--------------------------------------------------------------------------
  ret[2] = CommandObj_TagL->inventory(tagout);
  buf = tagout->get_buffer();
  size[2] = (unsigned int)tagout->length();

  for (int i = 0; i < (int)tagout->length(); i++)
  {
    data[2][i] = buf[i];
  }

  //--------------------------------------------------------------------------
  ret[3] = CommandObj_TagR->inventory(tagout);
  buf = tagout->get_buffer();
  size[3] = (unsigned int)tagout->length();

  for (int i = 0; i < (int)tagout->length(); i++)
  {
    data[3][i] = buf[i];
  }

  ret[4] = ret[0] && ret[1] && ret[2] && ret[3];
  printf("TagInit Result: %s, %s, %s, %s\n", BOOL(ret[0]), BOOL(ret[1]), BOOL(ret[2]), BOOL(ret[3]));

  return ret[4];
}
//------------------------------------------------------------------------------
