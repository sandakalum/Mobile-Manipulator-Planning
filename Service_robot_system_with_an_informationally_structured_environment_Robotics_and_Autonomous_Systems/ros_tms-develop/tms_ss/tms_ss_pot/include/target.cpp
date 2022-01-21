#ifdef WIN32
#pragma warning(disable : 4996)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
//#include "opencv2\opencv.hpp"
#include "opencv/cv.h"

#include "define.h"
#include "target.h"

CTarget::CTarget()
{
  id = 0;
  px = 0;
  py = 0;
  cnt = 0;
}

CTarget::~CTarget()
{
}

void CTarget::SetPosi(double x, double y)
{
  px = x;
  py = y;
}
