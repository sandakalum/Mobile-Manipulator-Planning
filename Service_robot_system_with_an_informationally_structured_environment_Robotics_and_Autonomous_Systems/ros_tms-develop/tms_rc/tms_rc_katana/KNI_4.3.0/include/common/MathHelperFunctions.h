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

#define M_PI 3.14159265358979323846

//<tfromm date="22.05.2009">
#include <cstdlib>
//</tfromm>
#include <cmath>
#include <vector>
#include <functional>
#include <cassert>

#ifndef KNI_MATH_HELPER_FUNCTIONS
#define KNI_MATH_HELPER_FUNCTIONS

namespace KNI_MHF
{
//*****************************************************************

template < typename _T >
inline short sign(_T x)
{
  return ((x < 0) ? -1 : 1);
}

//*****************************************************************

///
/// function-object which calculates sinus for n-elements of a container if used together
/// with a STL algorithm
template < typename _T >
struct unary_precalc_sin : public std::unary_function< _T, _T >
{
  _T operator()(_T& x)
  {
    return sin(x);
  }
};

///
/// \sa unary_precalc_sin
///
template < typename _T >
struct unary_precalc_cos : public std::unary_function< _T, _T >
{
  _T operator()(_T x)
  {
    return cos(x);
  }
};

//*****************************************************************
template < typename _T >
inline _T atan1(_T in1, _T in2)
{
  if (in1 == 0.0 && in2 != 0.0)
    // FIXME: Verify -/+
    return M_PI - sign(in2) * M_PI / 2;
  // return M_PI+sign(in2)*M_PI/2;

  if (in1 == 0.0 && in2 == 0.0)
    return 0.0;

  if (in1 < 0.0)
    return atan(in2 / in1) + M_PI;

  if ((in1 > 0.0) && (in2 < 0.0))
    return atan(in2 / in1) + 2.0 * M_PI;

  return atan(in2 / in1);
}

//*****************************************************************
template < typename _T >
inline _T acotan(const _T in)
{
  if (in == 0.0)
    return M_PI / 2;
  else
    return atan(1 / in);
}

//*************************************************
template < typename _T >
inline _T atan0(const _T in1, const _T in2)
{
  if (in1 == 0.0)
    return M_PI / 2;
  return atan(in2 / in1);
}

//*************************************************
template < typename _T >
inline _T pow2(const _T in)
{
  return pow(in, 2);
}

///
/// conversion from radian to degree
///
template < typename _T >
inline _T rad2deg(const _T a)
{
  return a * (180.0 / M_PI);
}

///
/// a function-object version of rad2deg
///
template < typename _T >
struct unary_rad2deg : public std::unary_function< _T, _T >
{
  _T operator()(const _T a)
  {
    return rad2deg(a);
  }
};

///
/// conversion from degree to radian
///
template < typename _T >
inline _T deg2rad(const _T a)
{
  return a * (M_PI / 180.0);
}

///
/// a function-object version of rad2deg
///
template < typename _T >
struct unary_deg2rad : public std::unary_function< _T, _T >
{
  _T operator()(const _T a)
  {
    deg2rad(a);
  }
};

//*************************************************
template < typename _T >
_T inline anglereduce(const _T a)
{
  return a - floor(a / (2 * M_PI)) * 2 * M_PI;
}
//*************************************************

///
/// converts absolute angles in radian to encoders.
///
template < typename _angleT, typename _encT >
inline _encT rad2enc(_angleT const& angle, _angleT const& angleOffset, _encT const& epc, _encT const& encOffset,
                     _encT const& rotDir)
{
  // converting all parameters to _angleT (usually =double)
  _angleT _epc = epc, _rotDir = rotDir, _angleOffset = angleOffset, _encOffset = encOffset;
#ifdef WIN32
  double temp = _encOffset + (_angleOffset - angle) * _epc * _rotDir / (2 * M_PI);
  return static_cast< _encT >((temp >= 0) ? floor(temp + 0.5) : floor(temp - 0.5));
#else
  return static_cast< _encT >(round(_encOffset + (_angleOffset - angle) * _epc * _rotDir / (2 * M_PI)));
#endif
}

///
/// converts encoders to absolute angles in radian
///
template < typename _angleT, typename _encT >
inline _angleT enc2rad(_encT const& enc, _angleT const& angleOffset, _encT const& epc, _encT const& encOffset,
                       _encT const& rotDir)
{
  // converting all parameters to _angleT (usually = double)
  _angleT _epc = epc, _rotDir = rotDir, _angleOffset = angleOffset, _encOffset = encOffset, _enc = enc;
  return _angleOffset - (_enc - _encOffset) * 2.0 * M_PI / (_epc * _rotDir);
}

///
/// Find the first equal angle.
/// You have to pass a cos and a sin Value
inline double findFirstEqualAngle(double cosValue, double sinValue, double tolerance)
{
  double v1[2], v2[2];

  v1[0] = acos(cosValue);
  v1[1] = -v1[0];
  v2[0] = asin(sinValue);
  v2[1] = M_PI - v2[0];

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      if (std::abs(anglereduce(v1[i]) - anglereduce(v2[j])) < tolerance)
        return v1[i];
    }
  }
  assert(!"precondition for findFirstEqualAngle failed -> no equal angles found");
  return 0;
}
}

#endif
