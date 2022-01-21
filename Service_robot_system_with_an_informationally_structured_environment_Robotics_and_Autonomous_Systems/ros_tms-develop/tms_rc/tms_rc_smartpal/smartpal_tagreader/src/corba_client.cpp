///-----------------------------------------------------------------------------
/// @FileName corba_client.cpp
/// @Date 2013.05.27
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
#include <stdlib.h>
#include <iostream>
using namespace std;
#include <vector>
#include "corba_client.h"

//------------------------------------------------------------------------------
// "Client" class constructor
//------------------------------------------------------------------------------
Client::Client()
  : bInitialize(false)
  , orb(NULL)
  , CommandObj_TagF(NULL)
  , CommandObj_TagB(NULL)
  , CommandObj_TagL(NULL)
  , CommandObj_TagR(NULL)
{
  corbaS1 = 0;
  corbaS2 = 0;
  corbaD1 = 0.;
  corbaD2 = 0.;
  corbaD3 = 0.;
  corbaD4 = 0.;
  corbaD5 = 0.;
  corbaD6 = 0.;
  corbaB1 = false;
  corbaL1 = 0;
  corbaUL1 = 0;
}
//------------------------------------------------------------------------------
// "Client" class destructor
//------------------------------------------------------------------------------
Client::~Client()
{
}

//------------------------------------------------------------------------------
// Function "Initialize" prepares CORBA interface connection,
// which includes getting naming service reference and resolving servant object(s)
//------------------------------------------------------------------------------
bool Client::Initialize()
{
  int argc = 2;
  char* argv[2];

  argv[0] = new char[sizeof("-ORBInitRef")];
  argv[1] = new char[128];

  strcpy(orb_ip, "192.168.4.221");
  strcpy(orb_port, "5005");
  strcpy(context_name, "root");

  snprintf(argv[0], sizeof("-ORBInitRef"), "-ORBInitRef");
  snprintf(argv[1], 128, "NameService=corbaloc::%s:%s/NameService", orb_ip, orb_port);

  cout << "ORB_init parameter: " << argv[1] << endl;

  // initialize ORB
  try
  {
    orb = CORBA::ORB_init(argc, argv);

    delete argv[0];
    delete argv[1];

    // Getting reference of naming service
    CORBA::Object_var ns;

    try
    {
      ns = orb->resolve_initial_references("NameService");
    }
    catch (const CORBA::ORB::InvalidName&)
    {
      cerr << argv[0] << ": can't resolve `NameService'" << endl;
      return false;
    }

    if (CORBA::is_nil(ns))
    {
      cerr << argv[0] << ": `NameService' is a nil object reference" << endl;
      return false;
    }

    // Getting root naming context
    CosNaming::NamingContext_var rootnc = CosNaming::NamingContext::_narrow(ns);

    if (CORBA::is_nil(rootnc))
    {
      cerr << argv[0] << ": `NameService' is not a NamingContext object reference" << endl;
      return false;
    }

    CosNaming::Name ncName;
    CORBA::Object_var obj;
    ncName.length(1);

    try
    {
      ncName[0].id = CORBA::string_dup("tagReaderFrontVehicle");
      obj = rootnc->resolve(ncName);
      CommandObj_TagF = Tag::ReaderWriter::_narrow(obj);

      ncName[0].id = CORBA::string_dup("tagReaderBottom2Vcl");
      obj = rootnc->resolve(ncName);
      CommandObj_TagB = Tag::ReaderWriter::_narrow(obj);

      ncName[0].id = CORBA::string_dup("tagReaderLeftHand");
      obj = rootnc->resolve(ncName);
      CommandObj_TagL = Tag::ReaderWriter::_narrow(obj);

      ncName[0].id = CORBA::string_dup("tagReaderRightHand");
      obj = rootnc->resolve(ncName);
      CommandObj_TagR = Tag::ReaderWriter::_narrow(obj);
    }
    catch (...)
    {
      cerr << "Object(Tagreader ...) not found" << endl;
      return false;
    }
  }
  catch (...)
  {
    cerr << "Object not found" << endl;
    return false;
  }

  bInitialize = true;
  return true;
}

//------------------------------------------------------------------------------
// Function "Shutdown" destroys ORB object
//------------------------------------------------------------------------------
bool Client::Shutdown()
{
  // destroy ORB
  try
  {
    if (!CORBA::is_nil(orb))
    {
      orb->destroy();
    }
  }
  catch (...)
  {
    cerr << "Shutdown error" << endl;
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
