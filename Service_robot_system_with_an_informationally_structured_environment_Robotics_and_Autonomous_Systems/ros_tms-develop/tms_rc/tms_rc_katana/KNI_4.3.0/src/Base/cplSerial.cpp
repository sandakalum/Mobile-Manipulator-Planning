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

#include "KNI/cplSerial.h"
#include "KNI/CRC.h"
#include <assert.h>
#include <iostream>
//<tfromm date="22.05.2009">
#include <cstring>
//</tfromm>

#include <stdio.h>

bool CCplSerialCRC::load_tbl()
{
  bool status = true;

  // set table to zero
  memset(cmd, 0, sizeof(cmd));

  // set a tentative command table for the katana
  cmd[(int)'B'].send_sz = 1;
  cmd[(int)'B'].read_sz = 3;
  cmd[(int)'X'].send_sz = 1;
  cmd[(int)'X'].read_sz = 181;
  cmd[(int)'Y'].send_sz = 1;
  cmd[(int)'Y'].read_sz = 84;
  cmd[(int)'Z'].send_sz = 1;
  cmd[(int)'Z'].read_sz = 1;
  cmd[(int)'C'].send_sz = 5;
  cmd[(int)'C'].read_sz = 3;
  cmd[(int)'D'].send_sz = 2;
  cmd[(int)'D'].read_sz = 8;
  cmd[(int)'E'].send_sz = 2;
  cmd[(int)'E'].read_sz = 18;
  cmd[(int)'V'].send_sz = 3;
  cmd[(int)'V'].read_sz = 13;
  cmd[(int)'N'].send_sz = 3;
  cmd[(int)'N'].read_sz = 13;
  cmd[(int)'G'].send_sz = 14;
  cmd[(int)'G'].read_sz = 2;
  cmd[(int)'G' + 128].send_sz = 3;
  cmd[(int)'G' + 128].read_sz = 2;
  cmd[(int)'H'].send_sz = 75;
  cmd[(int)'H'].read_sz = 3;
  cmd[(int)'A'].send_sz = 3;
  cmd[(int)'A'].read_sz = 2;
  cmd[(int)'S'].send_sz = 6;
  cmd[(int)'S'].read_sz = 6;
  cmd[(int)'I'].send_sz = 2;
  cmd[(int)'I'].read_sz = 3;
  cmd[(int)'M'].send_sz = 5;
  cmd[(int)'M'].read_sz = 4;
  cmd[(int)'T'].send_sz = 5;
  cmd[(int)'T'].read_sz = 2;

  return status;
}

void CCplSerialCRC::defineProtocol(byte _kataddr)
{
  hdr.size = 3;
  hdr.data[0] = 1;         // convention
  hdr.data[1] = _kataddr;  // complete header
}

bool CCplSerialCRC::init(CCdlBase* _device, byte _kataddr)
{
  device = _device;
  defineProtocol(_kataddr);
  return load_tbl();
}

void CCplSerialCRC::comm(const byte* pack, byte* buf, byte* size)
{
  // This method enssamble the packet with the header, data, and CRC.
  // Sends it and receives the answer.
  memset(send_buf, 0, 256);                       // override old values
  hdr.data[hdr.size - 1] = cmd[pack[0]].send_sz;  // complete header
  memcpy(send_buf, hdr.data, hdr.size);
  memcpy(send_buf + hdr.size, pack, hdr.data[hdr.size - 1]);

  short crc = CRC_CHECKSUM((uint8*)pack, hdr.data[hdr.size - 1]);
  byte bufsz = hdr.size + hdr.data[hdr.size - 1];
  send_buf[bufsz++] = (byte)(crc >> 8);    // hi-byte
  send_buf[bufsz++] = (byte)(crc & 0xFF);  // lo-byte

  memset(read_buf, 0, 256);  // read through device
  byte read_sz = cmd[pack[0]].read_sz + 2;

  short tries_recv = 0;
  while (true)
  {
    try
    {
      // uncomment to get debug output:
      // if (pack[0] != 'N') {
      //	// N = Get all axis's positions at once
      //	printf("KNI >>> %c", pack[0]);
      //	for (size_t i = 1; i < cmd[pack[0]].send_sz; i++)
      //	printf(" %d", pack[i]);
      //	printf("\n");
      // }
      send(send_buf, bufsz, NUMBER_OF_RETRIES_SEND);  // pass exceptions from this since already retried
    }
    catch (...)
    {
      throw;
    }
    // For comm DEBUG only: std::cout << "send OK, try to receive...\n";
    try
    {
      recv(read_buf, read_sz, size);

      // uncomment to get debug output:
      // if (read_buf[0] != 'n')
      // {
      // 	printf("KNI <<< %c", read_buf[0]);
      // 	for (size_t i = 1; i < *size; i++)
      // 	printf(" %d", read_buf[i]);
      // 	printf("\n");
      // }
      memcpy(buf, read_buf, *size);  // copy read_buf to _buf
    }
    catch (ReadNotCompleteException&)
    {
      if (tries_recv < NUMBER_OF_RETRIES_RECV)
      {
        ++tries_recv;
        continue;
      }
      throw;
    }
    catch (WrongCRCException&)
    {
      if (tries_recv < NUMBER_OF_RETRIES_RECV)
      {
        ++tries_recv;
        continue;
      }
      else
      {
        throw;
      }
    }
    catch (FirmwareException&)
    {
      throw;
    }
    catch (Exception&)
    {
      throw;
      // FIXME why? --MRE
      // 			if(errorFlag == true){
      // 			}
    }
    catch (...)
    {
      throw;
    }
    break;  // write succeeded
  }
}

void CCplSerialCRC::getMasterFirmware(short* fw, short* rev)
{
  *fw = mMasterVersion;
  *rev = mMasterRevision;
}

void CCplSerialCRC::send(byte* send_buf, byte bufsz, short retries)
{
  short r = 0;
  while (true)
  {
    try
    {
      device->send(send_buf, bufsz);  // send to device
    }
    catch (WriteNotCompleteException&)
    {
      if (r < retries)
      {
        ++r;
        continue;
      }
      else
      {
        throw;
      }
    }
    catch (Exception&)
    {
      throw;  // throw other exceptions immediately
    }
    break;
  }
}

void CCplSerialCRC::recv(byte* read_buf, byte read_sz, byte* size)
{
  // Receives the packet and checks the CRC.
  *size = device->recv(read_buf, read_sz);  // receives from device

  bool getErrorMessage = false;
  // check for error from Katana:
  if (read_buf[0] == KATANA_ERROR_FLAG)
  {
    std::cout << "Error flag received:\n";
    assert(*size == 3);
    getErrorMessage = true;
    read_sz = *size;  // FIXME: should not return the now invalid buffer?
  }
  else
  {
    if (*size != read_sz)
    {
      throw ReadNotCompleteException("?");  // FIXME: should get _ipAddr for nicer error message
    }
  }

  *size -= 2;
  byte bhi = read_buf[read_sz - 2];
  byte blo = read_buf[read_sz - 1];

  short crc = CRC_CHECKSUM((uint8*)read_buf, *size);
  byte hi = (byte)(crc >> 8);
  byte lo = (byte)(crc & 0xFF);

  if ((hi != bhi) || (lo != blo))
  {
    std::cout << "warning: crc error, throwing exception" << std::endl;
    throw WrongCRCException();
  }

  if (getErrorMessage)
  {
    byte buf[57];
    buf[0] = 0;  // ignored
    buf[1] = 0;  // ignored
    buf[2] = 0;  // ignored
    buf[3] = KATANA_ERROR_FLAG + 1;
    try
    {
      send(buf, 4, 1);
      byte size_errormsg = 57;
      recv(buf, 57, &size_errormsg);
    }
    catch (...)
    {
      std::cout << "Error while requesting error message!\n";
    }

    if (buf[0] != KATANA_ERROR_FLAG + 1)
    {
      std::cout << "bad response to error request\n";
    }
    byte lastCommand = buf[1];
    byte errorCode = buf[2];
    byte device = buf[3];
    std::string errorString((char*)buf + 4);
    if (device != 0)
    {
      errorString += " (axis ";
      errorString += '0' + device;
      errorString += ")";
    }
    // std::cout << "\"" << errorString << "\"\n";
    throw FirmwareException(errorString, static_cast< signed char >(errorCode), device, lastCommand);
  }
}
