//
// C++ Implementation: kmlSctBase
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "KNI/kmlSctBase.h"

bool CSctBase::init(CKatBase* _own, const TSctDesc _sctDesc, CCplBase* _protocol)
{
  gnl.own = _own;
  gnl.SID = _sctDesc.ctrlID;
  gnl.res = _sctDesc.sens_res;
  dat.cnt = _sctDesc.sens_count;
  dat.arr = new short[dat.cnt];
  protocol = _protocol;
  return true;
}

void CSctBase::recvDAT()
{
  int i;          // iterator
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size

  // switch between 8/12 bit resolution
  p[0] = 'E';
  p[1] = gnl.SID;

  protocol->comm(p, buf, &sz);
  if (!buf[1])
    throw ParameterReadingException("DAT");
  for (i = 0; i < dat.cnt; i++)
  {
    dat.arr[i] = (short)buf[i + 2];
  }
}
