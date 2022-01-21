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

/****************************************************************************/
#ifndef _CRC_H
#define _CRC_H
/****************************************************************************/

#define uint8 unsigned char    //!< unsigned 8 bit
#define uint16 unsigned short  //!< unsigned 16 bit

/*--------------------------------------------------------------------------*/

/*	\brief	Calculates a 16 bit checksum value of give data with
 *			given size.
 */
uint16 CRC_CHECKSUM(uint8 *data, uint8 size_of_BYTE);

/****************************************************************************/
#endif  //_CRC_H
/****************************************************************************/
