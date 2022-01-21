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

#include "KNI_InvKin/KatanaKinematicsDecisionAlgorithms.h"
#include "common/MathHelperFunctions.h"

namespace KNI
{
KinematicsDefaultEncMinAlgorithm::t_iter KinematicsDefaultEncMinAlgorithm::
operator()(t_iter targetEnc_begin, t_iter targetEnc_end, c_iter currentEnc_begin, c_iter currentEnc_end)
{
  // assert on distance(tE_beg, tE_enc) != distance(cE_beg, cE_enc)
  // assert(::std::distance(targetEnc_begin, targetEnc_end) == ::std::distance(currentEnc_begin, currentEnc_end) &&
  // "Numbers of target encoders and current encoders provided don't match" );
  using namespace KNI_MHF;
  double dist(0), sum(0), mindist = 1000000;
  t_iter index = targetEnc_end;

  for (t_iter target = targetEnc_begin; target != targetEnc_end; ++target)
  {
    sum = 0;

    c_iter t = (*target).begin();
    c_iter c = currentEnc_begin;
    while (t != (*target).end() && c != currentEnc_end)
    {
      sum += pow2< double >(*t - *c);
      ++t;
      ++c;
    }
    dist = sqrt(sum);
    if (dist < mindist)
    {
      index = target;
      mindist = dist;
    }
  }

  return index;
}
}
