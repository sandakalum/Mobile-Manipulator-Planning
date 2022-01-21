#ifndef REFLEXXES_DATA
#define REFLEXXES_DATA

#include <Reflexxes/TypeII/ReflexxesAPI.h>
#include <Reflexxes/TypeII/RMLPositionFlags.h>
#include <Reflexxes/TypeII/RMLPositionInputParameters.h>
#include <Reflexxes/TypeII/RMLPositionOutputParameters.h>

struct ReflexxesData {
  ReflexxesAPI *rml;
  RMLPositionInputParameters *inputParameters;
  RMLPositionOutputParameters *outputParameters;
  RMLPositionFlags flags;
  unsigned int NUMBER_OF_DOFS;
  int resultValue;

  const ReflexxesData clone() {return *this;}
};

#endif
