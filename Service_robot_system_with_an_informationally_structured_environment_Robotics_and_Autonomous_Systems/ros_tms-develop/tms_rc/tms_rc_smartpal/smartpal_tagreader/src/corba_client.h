//------------------------------------------------------------------------------

#ifndef __CORBA_CLIENT_H__
#define __CORBA_CLIENT_H__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;
#include <vector>
#include "corba/tag.h"

class Client
{
public:
  Client();
  virtual ~Client();
  bool Initialize();
  bool Shutdown();

  bool TagInit(void);
  bool TagSetConfig(int, int);
  bool TagSetPower(int);
  bool TagInventory(unsigned char *, unsigned char (*)[128]);

protected:
  CORBA::ORB_var orb;

  Tag::ReaderWriter_var CommandObj_TagF;
  Tag::ReaderWriter_var CommandObj_TagB;
  Tag::ReaderWriter_var CommandObj_TagL;
  Tag::ReaderWriter_var CommandObj_TagR;

  CORBA::Short corbaS1, corbaS2;
  CORBA::Double corbaD1, corbaD2, corbaD3, corbaD4, corbaD5, corbaD6;
  CORBA::String_var corbaStr1;
  CORBA::Boolean corbaB1;
  CORBA::Long corbaL1;
  CORBA::ULong corbaUL1;

  char orb_ip[24];
  char orb_port[8];
  char context_name[64];

private:
  bool bInitialize;
};
#endif  // __CORBA_CLIENT_H__

//------------------------------------------------------------------------------
