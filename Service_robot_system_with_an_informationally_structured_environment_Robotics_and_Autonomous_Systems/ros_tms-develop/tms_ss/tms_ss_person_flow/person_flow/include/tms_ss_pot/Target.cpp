
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "opencv/cv.h"

#include "define.h"
#include "Target.h"

// CTarget

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
