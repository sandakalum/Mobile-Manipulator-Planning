/***************************************************************************
 *   Copyright (C) 2006 by Tiziano Mueller   *
 *   tiziano.mueller@neuronics.ch   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef KNIKATANAKINEMATICSDECISIONALGORITHMS_H
#define KNIKATANAKINEMATICSDECISIONALGORITHMS_H

#include "common/dllexport.h"
#include <vector>

namespace KNI
{
struct DLLDIR_IK KinematicsDefaultEncMinAlgorithm
{
  typedef std::vector< int > encoders;
  typedef encoders::const_iterator c_iter;
  typedef std::vector< encoders >::const_iterator t_iter;

  t_iter operator()(t_iter targetEnc_begin, t_iter targetEnc_end, c_iter currentEnc_begin, c_iter currentEnc_end);
};
}

#endif
