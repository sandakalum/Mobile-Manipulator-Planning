///-----------------------------------------------------------------------------
/// @FileName sp5_client.cpp
/// @Date 2013.06.02 / 2013.07.03
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
#include "sp5_client.h"

//------------------------------------------------------------------------------
// "Client" class constructor
//------------------------------------------------------------------------------
Client::Client()
  : bInitialize(false)
  , orb(NULL)
  , CommandObj_Vehicle(NULL)
  , CommandObj_ArmR(NULL)
  , CommandObj_ArmL(NULL)
  , CommandObj_GripperR(NULL)
  , CommandObj_GripperL(NULL)
  , CommandObj_Lumbar(NULL)
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

  strcpy(orb_ip, "192.168.4.211");
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

    //----------------------------------------------------------------------
    // resolve [Vcl.command],[MnpR],[MnpL],[GripperR],[GripperL] object from root context
    //----------------------------------------------------------------------
    // Wireless LAN			 192.168.4.200
    // Control PC            192.168.4.201
    // Vcl      			 192.168.4.202
    // MnpR					 192.168.4.203
    // MnpL					 192.168.4.204
    // GripperR				(192.168.4.203)
    // GripperL				(192.168.4.204)
    // Lumbar				 192.168.4.205
    //----------------------------------------------------------------------
    CosNaming::Name ncName;
    CORBA::Object_var obj;
    ncName.length(1);

    try
    {
      ncName[0].id = CORBA::string_dup("Mobility");
      ncName[0].kind = CORBA::string_dup("Unit");
      obj = rootnc->resolve(ncName);
      CommandObj_Vehicle = YeRTUnitMobility::mobility::_narrow(obj);
    }
    catch (...)
    {
      cerr << "Mobility.Unit not found" << endl;
      return false;
    }

    try
    {
      ncName[0].id = CORBA::string_dup("MnpR");
      ncName[0].kind = CORBA::string_dup("");
      obj = rootnc->resolve(ncName);
      CommandObj_ArmR = ArmUnit::_narrow(obj);
    }
    catch (...)
    {
      cerr << "MnpR not found" << endl;
      return false;
    }

    try
    {
      ncName[0].id = CORBA::string_dup("MnpL");
      ncName[0].kind = CORBA::string_dup("");
      obj = rootnc->resolve(ncName);
      CommandObj_ArmL = ArmUnit::_narrow(obj);
    }
    catch (...)
    {
      cerr << "MnpL not found" << endl;
      return false;
    }

    try
    {
      ncName[0].id = CORBA::string_dup("GripperR");
      ncName[0].kind = CORBA::string_dup("");
      obj = rootnc->resolve(ncName);
      CommandObj_GripperR = GripperUnit::_narrow(obj);
    }
    catch (...)
    {
      cerr << "GripperR not found" << endl;
      return false;
    }

    try
    {
      ncName[0].id = CORBA::string_dup("GripperL");
      ncName[0].kind = CORBA::string_dup("");
      obj = rootnc->resolve(ncName);
      CommandObj_GripperL = GripperUnit::_narrow(obj);
    }
    catch (...)
    {
      cerr << "GripperL not found" << endl;
      return false;
    }

    try
    {
      ncName[0].id = CORBA::string_dup("Lumbar");
      ncName[0].kind = CORBA::string_dup("");
      obj = rootnc->resolve(ncName);
      CommandObj_Lumbar = LumbarUnit::_narrow(obj);
    }
    catch (...)
    {
      cerr << "Lumbar not found" << endl;
      return false;
    }
  }
  catch (...)
  {
    cerr << "Object(Mobility.Unit, MnpL, MnpR ...) not found" << endl;
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
