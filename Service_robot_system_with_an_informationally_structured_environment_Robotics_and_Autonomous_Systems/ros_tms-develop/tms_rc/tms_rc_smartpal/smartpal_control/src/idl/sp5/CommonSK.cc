// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "Common.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



void
Alarm::operator>>= (cdrStream &_n) const
{
  code >>= _n;
  type >>= _n;
  _n.marshalString(description,0);

}

void
Alarm::operator<<= (cdrStream &_n)
{
  (::CORBA::ULong&)code <<= _n;
  (AlarmType&)type <<= _n;
  description = _n.unmarshalString(0);

}

void
AxisLimit::operator>>= (cdrStream &_n) const
{
  id >>= _n;
  upper >>= _n;
  lower >>= _n;

}

void
AxisLimit::operator<<= (cdrStream &_n)
{
  (::CORBA::ULong&)id <<= _n;
  (::CORBA::Double&)upper <<= _n;
  (::CORBA::Double&)lower <<= _n;

}

