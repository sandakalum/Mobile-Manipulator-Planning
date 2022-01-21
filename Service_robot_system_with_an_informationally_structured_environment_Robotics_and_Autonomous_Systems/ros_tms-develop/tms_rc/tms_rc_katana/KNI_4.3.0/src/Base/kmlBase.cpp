/*
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005 Neuronics AG
 *   Check out the AUTHORS file for detailed contact information.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "KNI/kmlBase.h"

#include "common/Timer.h"

#include <vector>
////////////////////////////////////////////////////////////////
bool CKatBase::init(const TKatGNL _gnl, const TKatMOT _mot, const TKatSCT _sct, const TKatEFF _eff, CCplBase* _protocol)
{
  // init vars
  gnl = _gnl;
  mot = _mot;
  sct = _sct;
  eff = _eff;

  protocol = _protocol;

  // init motors
  mot.arr = new CMotBase[mot.cnt];
  for (int m = 0; m < mot.cnt; m++)
  {
    if (!mot.arr[m].init(this, mot.desc[m], protocol))
    {
      delete[] mot.arr;
      return false;
    }
  }

  // init sensor contollers
  sct.arr = new CSctBase[sct.cnt];
  for (int s = 0; s < sct.cnt; s++)
  {
    if (!sct.arr[s].init(this, sct.desc[s], protocol))
    {
      delete[] sct.arr;
      return false;
    }
  }
  return true;
}
////////////////////////////////////////////////////////////////
void CKatBase::recvMFW()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'B';

  protocol->comm(p, buf, &sz);

  mfw.ver = buf[1];
  mfw.rev = buf[2];
}
////////////////////////////////////////////////////////////////
void CKatBase::recvIDS()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'Y';

  protocol->comm(p, buf, &sz);

  memcpy(ids.strID, buf + 1, sz - 1);
  ids.strID[sz - 3] = 0;
}
////////////////////////////////////////////////////////////////
void CKatBase::recvCTB()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'X';

  protocol->comm(p, buf, &sz);

  memcpy(ctb.cmdtbl, buf + 1, sz - 1);
  ctb.cmdtbl[sz - 1] = 0;
}
////////////////////////////////////////////////////////////////
//'G'et every 'M'otor's 'S'tatus flag
void CKatBase::recvGMS()
{
  int i;
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'N';
  p[1] = 1;
  p[2] = 0;

  protocol->comm(p, buf, &sz);

  for (i = 0; i < mot.cnt; i++)
  {
    mot.arr[i].pvp.msf = (TMotStsFlg)buf[i + 1];
  }
}
////////////////////////////////////////////////////////////////
void CKatBase::waitFor(TMotStsFlg status, int waitTimeout, bool gripper)
{
  KNI::Timer t(waitTimeout);
  t.Start();
  int nOfMot = GetMOT()->cnt;
  if (gripper)
    nOfMot--;
  // win32 compiler compatibility
  bool reached[10];
  for (int i = 0; i < nOfMot; ++i)
  {
    reached[i] = false;
  }
  bool reachedall;
  while (true)
  {
    if (t.Elapsed())
      throw MotorTimeoutException();
    recvGMS();
    reachedall = true;
    for (int i = 0; i < nOfMot; ++i)
    {
      if (mot.arr[i].pvp.msf == 40)
        throw MotorCrashException();
      if ((reached[i] == false) && (mot.arr[i].pvp.msf == status))
      {
        reached[i] = true;
      }
      if (reached[i] == false)
        reachedall = false;
    }
    if (reachedall)
      return;
    KNI::sleep(1000);
  }
}
////////////////////////////////////////////////////////////////
void CKatBase::recvECH()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'Z';

  protocol->comm(p, buf, &sz);

  ech.echo = buf[0];
  if (ech.echo != 'z')
  {
    throw ParameterReadingException("ECH");
  }
}
////////////////////////////////////////////////////////////////
void CKatBase::getMasterFirmware(short* fw, short* rev)
{
  *fw = mMasterVersion;
  *rev = mMasterRevision;
}
////////////////////////////////////////////////////////////////
void CKatBase::enableCrashLimits()
{
  // adjust second byte of packet according to katana model
  short version, revision;
  int motor = 1;  // master ON with KatHD300
  getMasterFirmware(&version, &revision);
  if (checkKatanaType(400))
  {
    motor = 0;  // switch all motors with KatHD400
  }
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'A';
  p[1] = motor;
  p[2] = 1;
  protocol->comm(p, buf, &sz);
}
////////////////////////////////////////////////////////////////
void CKatBase::disableCrashLimits()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'A';
  p[1] = 0;
  p[2] = 0;
  protocol->comm(p, buf, &sz);
}
////////////////////////////////////////////////////////////////
// deprecated, use speed & position
void CKatBase::setCrashLimit(long idx, int limit)
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'S';
  p[1] = 5;  // subcommand 5 "Set Crashlimit"
  p[2] = (char)(limit >> 8);
  p[3] = (char)(limit);
  p[4] = 0;
  protocol->comm(p, buf, &sz);
}
////////////////////////////////////////////////////////////////
void CKatBase::setPositionCollisionLimit(long idx, int limit)
{
  mot.arr[idx].setPositionCollisionLimit(limit);
}
////////////////////////////////////////////////////////////////
void CKatBase::setSpeedCollisionLimit(long idx, int limit)
{
  mot.arr[idx].setSpeedCollisionLimit(limit);
}
////////////////////////////////////////////////////////////////
void CKatBase::unBlock()
{
  for (int i = 0; i < mot.cnt; ++i)
  {
    mot.arr[i].resetBlocked();
  }
}
////////////////////////////////////////////////////////////////
void CKatBase::startSplineMovement(int exactflag, int moreflag)
{
  // Start Linear movement
  std::vector< byte > sendBuf(3), recvBuf(2, 0);
  byte readBytes;
  sendBuf[0] = 'G' + 128;
  sendBuf[1] = (byte)moreflag;
  sendBuf[2] = (byte)exactflag;
  protocol->comm(&sendBuf.front(), &recvBuf.front(), &readBytes);
}
////////////////////////////////////////////////////////////////
void CKatBase::setAndStartPolyMovement(std::vector< short > polynomial, int exactflag, int moreflag)
{
  // set and start poly movement on all motors
  std::vector< byte > sendBuf(75), recvBuf(3, 0);
  byte readBytes;
  sendBuf[0] = 'H';
  for (int i = 0; i < (int)polynomial.size(); ++i)
  {
    sendBuf[2 * i + 1] = static_cast< byte >(polynomial[i] >> 8);
    sendBuf[2 * i + 2] = static_cast< byte >(polynomial[i]);
  }
  sendBuf[73] = (byte)moreflag;
  sendBuf[74] = (byte)exactflag;
  protocol->comm(&sendBuf.front(), &recvBuf.front(), &readBytes);
}
////////////////////////////////////////////////////////////////
void CKatBase::recvMPS()
{
  byte p[32];     // packet
  byte buf[256];  // readbuf
  byte sz = 0;    // readbuf size
  p[0] = 'N';
  p[1] = 3;
  p[2] = 0;
  protocol->comm(p, buf, &sz);
  for (int i = 0; i < mot.cnt; i++)
  {
    mot.arr[i].pvp.pos = (((short)buf[2 * i + 1]) << 8) | buf[2 * i + 2];
  }
}
////////////////////////////////////////////////////////////////
int CKatBase::checkKatanaType(int type)
{
  if (protocol != NULL)
  {
    recvMFW();
    if ((type == 400) || (type == 450))
    {
      if (mfw.ver > K400_OLD_PROTOCOL_THRESHOLD)
      {
        return -1;
      }
    }
    else if (type == 300)
    {
      if (mfw.ver < K400_OLD_PROTOCOL_THRESHOLD)
      {
        return -1;
      }
    }
  }

  return 1;
}
////////////////////////////////////////////////////////////////
int CKatBase::readDigitalIO()
{
  byte p[32];
  byte buf[256];
  byte sz = 0;
  p[0] = 'T';
  p[1] = 'r';
  p[2] = 0;
  p[3] = 0;
  p[4] = 0;
  protocol->comm(p, buf, &sz);
  return buf[1];
}
////////////////////////////////////////////////////////////////
int CKatBase::flushMoveBuffers()
{
  byte p[32];
  byte buf[256];
  byte sz = 0;
  p[0] = 'C';
  p[1] = 0;
  p[2] = 32;
  p[3] = 0;
  p[4] = 0;
  protocol->comm(p, buf, &sz);
  return 1;
}
////////////////////////////////////////////////////////////////
