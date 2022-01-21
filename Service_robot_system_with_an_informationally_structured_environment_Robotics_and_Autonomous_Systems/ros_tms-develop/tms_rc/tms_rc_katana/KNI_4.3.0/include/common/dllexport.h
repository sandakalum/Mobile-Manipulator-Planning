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

#ifndef _DLLEXPORT_H_
#define _DLLEXPORT_H_

#undef DLLDIR
#undef DLLDIR_IK

#define DLLDIR
#define DLLDIR_IK
#define DLLDIR_LM

#ifdef DLLDIR_EXPORT  // export DLL information
#undef DLLDIR
//#	undef DLLDIR_IK
#define DLLDIR __declspec(dllexport)
//#	define DLLDIR_IK __declspec(dllexport)
#endif

#ifdef DLLDIR_IMPORT  // import DLL information
#undef DLLDIR
#undef DLLDIR_IK
#undef DLLDIR_LM
#define DLLDIR __declspec(dllimport)
#define DLLDIR_IK __declspec(dllimport)
#define DLLDIR_LM __declspec(dllimport)
#endif

#ifdef DLLDIR_INVKIN_EXPORT
#undef DLLDIR
#undef DLLDIR_IK
#define DLLDIR
//#	define DLLDIR  __declspec(dllimport)
#define DLLDIR_IK __declspec(dllexport)
#endif

#ifdef DLLDIR_LM_EXPORT
#undef DLLDIR
#undef DLLDIR_IK
#undef DLLDIR_LM
#define DLLDIR
#define DLLDIR_IK
//#	define DLLDIR  __declspec(dllimport)
//#	define DLLDIR_IK  __declspec(dllimport)
#define DLLDIR_LM __declspec(dllexport)
#endif

#ifdef WIN32
#pragma warning(disable : 4251)
#pragma warning(disable : 4275)
#endif

#endif
