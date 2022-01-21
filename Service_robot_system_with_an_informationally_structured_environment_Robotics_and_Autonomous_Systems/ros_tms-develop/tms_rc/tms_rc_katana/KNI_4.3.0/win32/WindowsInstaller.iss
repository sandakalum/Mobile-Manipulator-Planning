#define KNIVERSION "4.3"; define constant
#define KNIRELEASE "0"; define constant
[Setup]
ShowLanguageDialog=auto
VersionInfoVersion={#KNIVERSION}
VersionInfoCompany=Neuronics AG
VersionInfoDescription=Katana Native Interface
VersionInfoCopyright=Switzerland 2009, Neuronics AG
AppCopyright=Neuronics AG
AppName=KatanaNativeInterface
AppVerName=Katana Native Interface v{#KNIVERSION}.{#KNIRELEASE}
LicenseFile=..\LICENSE.txt
PrivilegesRequired=admin
DefaultDirName={pf}\Neuronics AG\KatanaNativeInterface
AppID={{6096C108-7512-489E-AA7C-6379BBF22E14}
AppPublisher=Neuronics AG
AppPublisherURL=http://www.neuronics.ch
AppSupportURL=http://www.neuronics.ch
AppUpdatesURL=http://www.neuronics.ch
DisableProgramGroupPage=false
OutputDir=.
OutputBaseFilename=KatanaNativeInterface-v{#KNIVERSION}.{#KNIRELEASE}
DefaultGroupName=Katana Native Interface
[Files]
Source: ..\lib\win32\*; DestDir: {app}\lib\win32; Flags: overwritereadonly; Components: Libraries
Source: ..\configfiles400\*; DestDir: {app}\configfiles400; Flags: confirmoverwrite; Components: Libraries
Source: ..\configfiles450\*; DestDir: {app}\configfiles450; Flags: confirmoverwrite; Components: Libraries
Source: ..\doc\*; DestDir: {app}\doc; Flags: recursesubdirs; Components: Documentation; Excludes: source
Source: ..\include\*; DestDir: {app}\include; Flags: recursesubdirs; Excludes: *~; Components: Libraries
Source: ..\py\*; DestDir: {app}\py; Components: PythonBindings
Source: ..\demo\*; DestDir: {app}\demo; Flags: recursesubdirs; Components: Demos; Excludes: Makefile*, Release, Debug, *.pdb, *.user, *.exp, *.orig, *.idb, *.manifest, *.htm
Source: ..\KNI.net\*; DestDir: {app}\KNI.net; Components: KNInet
Source: ..\src\*; DestDir: {app}\src; Flags: recursesubdirs; Components: Sourcecode; Excludes: *~
Source: ..\lib\kinematics\*; DestDir: {app}\lib\kinematics; Flags: recursesubdirs; Components: Sourcecode; Excludes: *~
Source: ..\drivers\*; DestDir: {app}\src; Flags: recursesubdirs; Components: Sourcecode
Source: ..\*.txt; DestDir: {app}; Components: Libraries
Source: ..\win32\*; DestDir: {app}\win32; Components: Libraries; Excludes: *.iss

[Components]
Name: Libraries; Description: The KNI libraries (DLL and LIB); Flags: fixed; Types: custom compact full
Name: Demos; Description: Demo Applications; Types: full
Name: KNInet; Description: KNI .net binding; Types: full
Name: Documentation; Description: Documentation (Contains the KNI Manual and the API Reference); Types: full
Name: PythonBindings; Description: Bindings for Python
Name: Sourcecode; Description: The KNI source code; Types: full

[Code]

[Icons]
Name: {group}\KNI Manual; Filename: {app}\doc\en\233551-Katana_450-Quickstart_Guide-Programming_with_KNI-v1.0.0.pdf; Comment: Katana Native Interface Manual; Components: Documentation
Name: {group}\API Reference; Filename: {app}\doc\neuronics-doc\233559-Katana_450-API_Documentation-Katana_Native_Interface-v0.0.4.pdf; Comment: KNI API Reference; Components: Documentation
Name: {group}\Demo Applications; Filename: {app}\demo; Components: Demos
Name: {group}\KNI Visual C++ Solution; Filename: {app}\win32\KatanaNativeInterface.sln; Components: Demos
Name: {group}\Uninstall; Filename: {uninstallexe}
