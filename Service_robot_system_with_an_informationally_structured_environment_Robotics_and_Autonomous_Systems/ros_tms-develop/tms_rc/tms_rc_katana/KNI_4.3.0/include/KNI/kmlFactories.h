
#ifndef KMLFACTORIES_H
#define KMLFACTORIES_H

#include "common/exception.h"

#include "KNI/kmlBase.h"
#include "KNI/kmlMotBase.h"
#include "KNI/kmlSctBase.h"

//<tfromm date="22.05.2009">
#include <cstdlib>
//</tfromm>
#include <string>
#include <fstream>

///
/// @addtogroup exceptions
/// @{
///

///
/// The state of the configuration file wasn't "good"
/// \note error_number=-41
class ConfigFileStateException : public Exception
{
public:
  ConfigFileStateException() throw() : Exception("ConfigFile is not open or other failure", -41)
  {
  }
};

///
/// The requested section could not be found
/// \note error_number=-42
class ConfigFileSectionNotFoundException : public Exception
{
public:
  ConfigFileSectionNotFoundException(const std::string& attribute) throw()
    : Exception("Could not find section '" + attribute + "' in configfile", -42)
  {
  }
};

///
/// The requested subsection could not be found
/// \note error_number=-43
class ConfigFileSubsectionNotFoundException : public Exception
{
public:
  ConfigFileSubsectionNotFoundException(const std::string& attribute) throw()
    : Exception("Could not find subsection '" + attribute + "' in configfile", -43)
  {
  }
};

///
/// The requested entry could not be found
/// \note error_number=-44
class ConfigFileEntryNotFoundException : public Exception
{
public:
  ConfigFileEntryNotFoundException(const std::string& attribute) throw()
    : Exception("Could not find entry '" + attribute + "' in configfile", -44)
  {
  }
};

///
/// There was a syntax error in the configuration file
/// \note error_number=-45
class ConfigFileSyntaxErrorException : public Exception
{
public:
  ConfigFileSyntaxErrorException(const std::string& line) throw()
    : Exception("Syntax error in this line: '" + line + "'", -45)
  {
  }
};

///
/// @}
///

namespace KNI
{
///
/// This class is for internal use only
/// It may change at any time
/// It shields the configuration file parsing.
class DLLDIR kmlFactory
{
private:
  std::ifstream _configfile;
  void _readEntry(char* dest, int destsz, const char* section, const char* subsection, const char* entry);

public:
  kmlFactory();

  bool openFile(const char* filepath)
  {
    _configfile.open(filepath);
    return _configfile.fail() ? false : true;
  }
#ifdef _UNICODE
  bool openFile(const wchar_t* filepath)
  {
    _configfile.open(filepath);
    return _configfile.fail() ? false : true;
  }
#endif

  TKatGNL getGNL();
  TKatMOT getMOT();
  TKatSCT getSCT();
  TKatEFF getEFF();
  TMotDesc* getMotDesc(short count);
  TSctDesc* getSctDesc(short count);

  TMotCLB getMotCLB(short number);
  TMotSCP getMotSCP(short number);
  TMotDYL getMotDYL(short number);

  //!returns the Katana type
  //!@return 300 for Katana300, 400 for Katana400, 450 for Katana450
  int getType();
  //!returns the Kinematics to use
  //!@return 0 for Analytical, 1 for RobAnaGuess
  int getKinematics();

  TMotInit getMotInit(short number);

  void getGripperParameters(bool& isPresent, int& openEncoders, int& closeEncoders);
};
}

#endif
