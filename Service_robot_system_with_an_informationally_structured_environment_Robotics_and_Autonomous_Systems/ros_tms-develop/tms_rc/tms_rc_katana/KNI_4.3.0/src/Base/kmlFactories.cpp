
#include "KNI/kmlFactories.h"

namespace KNI
{
kmlFactory::kmlFactory() : _configfile()
{
}

void kmlFactory::_readEntry(char* dest, int destsz, const char* section, const char* subsection, const char* entry)
{
  char line[256];
  short pos = 0;
  short idx = 0;

  _configfile.seekg(0);  // goto the begin

  if (!_configfile.good())
    throw ConfigFileStateException();

  do
  {  // search section
    memset(line, 0, sizeof(line));
    _configfile.getline(line, sizeof(line));
    strtok(line, "\r");  // strip off the CR
  } while (strcmp(line, section));

  if (_configfile.eof())
    throw ConfigFileSectionNotFoundException(section);

  do
  {  // search subsection
    memset(line, 0, sizeof(line));
    _configfile.getline(line, sizeof(line));
    strtok(line, "\r");  // strip off the CR
  } while (strcmp(line, subsection));
  if (_configfile.eof())
    throw ConfigFileSubsectionNotFoundException(subsection);

  do
  {  // search entry
    memset(line, 0, sizeof(line));
    _configfile.getline(line, sizeof(line));
    strtok(line, "\r");  // strip off the CR
  } while (strncmp(line, entry, strlen(entry)));
  if (_configfile.eof())
    throw ConfigFileEntryNotFoundException(entry);

  // parse input line the detect entry value

  while (line[pos++] != '=')
  {
    if (pos == 256)
      throw ConfigFileSyntaxErrorException(line);
  }
  while (line[pos++] != '"')
  {
    if (pos == 256)
      throw ConfigFileSyntaxErrorException(line);
  }

  memset(dest, 0, destsz);
  while (line[pos] != '"')
  {
    dest[idx++] = line[pos++];
    if (pos == 256)
      throw ConfigFileSyntaxErrorException(line);
  }
  while (line[pos++] != ';')
  {
    if (pos == 256)
      throw ConfigFileSyntaxErrorException(line);
  }
}

TKatGNL kmlFactory::getGNL()
{
  char input[256];
  TKatGNL gnl;

  _readEntry(input, sizeof(input), "[KATANA]", "[GENERAL]", "addr");
  gnl.adr = atoi(input);

  _readEntry(input, sizeof(input), "[KATANA]", "[GENERAL]", "modelName");
  sprintf(gnl.modelName, "%s", input);

  return gnl;
}

TKatMOT kmlFactory::getMOT()
{
  char input[256];
  TKatMOT mot;
  _readEntry(input, sizeof(input), "[KATANA]", "[GENERAL]", "motcnt");
  mot.cnt = atoi(input);
  mot.arr = NULL;
  mot.desc = getMotDesc(mot.cnt);
  return mot;
}

TKatSCT kmlFactory::getSCT()
{
  char input[256];
  TKatSCT katsct;
  _readEntry(input, sizeof(input), "[KATANA]", "[GENERAL]", "sctcnt");
  katsct.cnt = atoi(input);
  katsct.arr = NULL;
  katsct.desc = getSctDesc(katsct.cnt);
  return katsct;
}

int kmlFactory::getType()
{
  char input[256];
  _readEntry(input, sizeof(input), "[KATANA]", "[GENERAL]", "type");
  return atoi(input);
}

int kmlFactory::getKinematics()
{
  char input[256];
  _readEntry(input, sizeof(input), "[KATANA]", "[GENERAL]", "kinematics");
  if (!strcmp("Analytical", input))
    return 0;
  if (!strcmp("RobAnaGuess", input))
    return 1;
  // default: RobAnaGuess
  return 1;
}

TKatEFF kmlFactory::getEFF()
{
  char input[256];
  TKatEFF eff;
  _readEntry(input, sizeof(input), "[ENDEFFECTOR]", "[GENERAL]", "segment1");
  eff.arr_segment[0] = atof(input);
  _readEntry(input, sizeof(input), "[ENDEFFECTOR]", "[GENERAL]", "segment2");
  eff.arr_segment[1] = atof(input);
  _readEntry(input, sizeof(input), "[ENDEFFECTOR]", "[GENERAL]", "segment3");
  eff.arr_segment[2] = atof(input);
  _readEntry(input, sizeof(input), "[ENDEFFECTOR]", "[GENERAL]", "segment4");
  eff.arr_segment[3] = atof(input);
  return eff;
}

TMotDesc* kmlFactory::getMotDesc(short count)
{
  char input[256];
  char section[256];

  TMotDesc* motdesc = new TMotDesc[count];
  for (int i = 0; i < count; ++i)
  {
    memset(section, 0, sizeof(section));
    sprintf(section, "[MOT[%d]]", i);
    _readEntry(input, sizeof(input), section, "[GENERAL]", "slvID");
    motdesc[i].slvID = atoi(input);
  }
  return motdesc;
}

TSctDesc* kmlFactory::getSctDesc(short count)
{
  char input[256];
  char section[256];

  TSctDesc* sctdesc = new TSctDesc[count];

  for (int i = 0; i < count; ++i)
  {
    memset(section, 0, sizeof(section));
    sprintf(section, "[SCT[%d]]", i);
    _readEntry(input, sizeof(input), section, "[GENERAL]", "ctrlID");
    sctdesc[i].ctrlID = atoi(input);
    _readEntry(input, sizeof(input), section, "[GENERAL]", "sens_res");
    sctdesc[i].sens_res = atoi(input);
    _readEntry(input, sizeof(input), section, "[GENERAL]", "sens_count");
    sctdesc[i].sens_count = atoi(input);
  }
  return sctdesc;
}

TMotCLB kmlFactory::getMotCLB(short number)
{
  char input[256];
  char section[256];

  TMotCLB clb;

  memset(section, 0, sizeof(section));
  sprintf(section, "[MOT[%d]]", number);

  _readEntry(input, sizeof(input), section, "[CALIBRATION]", "enable");
  clb.enable = strcmp("TRUE", input) ? false : true;

  _readEntry(input, sizeof(input), section, "[CALIBRATION]", "order");
  clb.order = atoi(input);

  _readEntry(input, sizeof(input), section, "[CALIBRATION]", "dir");
  clb.dir = strcmp("DIR_POSITIVE", input) ? DIR_NEGATIVE : DIR_POSITIVE;

  //_readEntry(input,sizeof(input),section,"[CALIBRATION]","diff");
  // clbarr[i].diff = atoi(input);

  _readEntry(input, sizeof(input), section, "[CALIBRATION]", "mcf");
  if (!strcmp("MCF_OFF", input))
    clb.mcf = MCF_OFF;
  if (!strcmp("MCF_ON", input))
    clb.mcf = MCF_ON;
  if (!strcmp("MCF_FREEZE", input))
    clb.mcf = MCF_FREEZE;

  //_readEntry(input,sizeof(input),section,"[CALIBRATION]","timeout");
  // clbarr[i].timeout = strcmp("TM_ENDLESS", input) ? atoi(input) : TM_ENDLESS;

  //_readEntry(input,sizeof(input),section,"[CALIBRATION]","enc_tolerance");
  // clbarr[i].enc_tolerance = atoi(input);

  _readEntry(input, sizeof(input), section, "[CALIBRATION]", "encoderPositionAfter");
  clb.encoderPositionAfter = atoi(input);

  return clb;
}

TMotSCP kmlFactory::getMotSCP(short number)
{
  TMotSCP scp;
  char input[256];
  char section[256];

  memset(section, 0, sizeof(section));
  sprintf(section, "[MOT[%d]]", number);

  _readEntry(input, sizeof(input), section, "[STATIC]", "maxppwm");
  scp.maxppwm = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "maxnpwm");
  scp.maxnpwm = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kP");
  scp.kP = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kI");
  scp.kI = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kD");
  scp.kD = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kARW");
  scp.kARW = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kP_speed");
  scp.kP_speed = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kI_speed");
  scp.kI_speed = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kD_speed");
  scp.kD_speed = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "maxppwm_nmp");
  scp.maxppwm_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "maxnpwm_nmp");
  scp.maxnpwm_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kspeed_nmp");
  scp.kspeed_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kpos_nmp");
  scp.kpos_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "kI_nmp");
  scp.kI_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "crash_limit_nmp");
  scp.crash_limit_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[STATIC]", "crash_limit_lin_nmp");
  scp.crash_limit_lin_nmp = atoi(input);

  return scp;
}

TMotDYL kmlFactory::getMotDYL(short number)
{
  TMotDYL dyl;
  char input[256];
  char section[256];

  memset(section, 0, sizeof(section));
  sprintf(section, "[MOT[%d]]", number);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxaccel");
  dyl.maxaccel = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxdecel");
  dyl.maxdecel = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "minpos");
  dyl.minpos = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxpspeed");
  dyl.maxpspeed = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxnspeed");
  dyl.maxnspeed = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxcurr");
  dyl.maxcurr = atoi(input);

  dyl.actcurr = 0;

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxaccel_nmp");
  dyl.maxaccel_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxpspeed_nmp");
  dyl.maxpspeed_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxnspeed_nmp");
  dyl.maxnspeed_nmp = atoi(input);

  _readEntry(input, sizeof(input), section, "[DYNAMIC]", "maxcurr_nmp");
  dyl.maxcurr_nmp = atoi(input);

  return dyl;
}

TMotInit kmlFactory::getMotInit(short number)
{
  char input[256];
  char section[256];
  TMotInit init;
  memset(section, 0, sizeof(section));
  sprintf(section, "[MOT[%d]]", number);

  _readEntry(input, sizeof(input), section, "[INIT]", "encodersPerCycle");
  init.encodersPerCycle = atoi(input);

  _readEntry(input, sizeof(input), section, "[INIT]", "encoderOffset");
  init.encoderOffset = atoi(input);

  _readEntry(input, sizeof(input), section, "[INIT]", "rotationDirection");
  init.rotationDirection = strcmp("DIR_POSITIVE", input) ? -1 : 1;

  _readEntry(input, sizeof(input), section, "[INIT]", "angleOffset");
  init.angleOffset = atof(input);

  _readEntry(input, sizeof(input), section, "[INIT]", "angleRange");
  init.angleRange = atof(input);

  return init;
}

void kmlFactory::getGripperParameters(bool& isPresent, int& openEncoders, int& closeEncoders)
{
  char input[256];

  _readEntry(input, sizeof(input), "[KATANA]", "[GRIPPER]", "isPresent");
  isPresent = strcmp("YES", input) ? false : true;

  _readEntry(input, sizeof(input), "[KATANA]", "[GRIPPER]", "openEncoders");
  openEncoders = atoi(input);

  _readEntry(input, sizeof(input), "[KATANA]", "[GRIPPER]", "closeEncoders");
  closeEncoders = atoi(input);
}
}
