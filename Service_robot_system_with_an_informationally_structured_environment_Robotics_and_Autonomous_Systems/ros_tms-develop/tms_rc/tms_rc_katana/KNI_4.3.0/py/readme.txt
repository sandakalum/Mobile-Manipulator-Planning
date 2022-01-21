----------------------------------------------
Katana Native Interface
Python wrapper
Neuronics 2008/PKE
----------------------------------------------

In this directory, you can automatically generate a Python interface for the KNI wrapper library.
In order to build the interface, you need to install SWIG.

'make' builds the interface, provided the KNI library has been built beforehand.
'make rebuild' builds all anew, but you need to have SWIG installed for a successful build. The SWIG generated .cxx file has been included in this source tree in order to reduce the required dependencies for building KNI.

In order to use the interface, you may just open a Python shell and start with:

	>>> import KNI
	>>> KNI.initKatana("../configfiles450/katana6M90T.cfg", "192.168.1.1")
	>>> KNI.calibrate(0)
	Katana400 calibration started
	>>> 


You may also use the demo skript in ./demo and extend it accordingly.

The API is generally the same as the one from the kni_wrapper, so you may consult the kni_wrapper.h file or the respective doxygen documentation.
There exists a difference occuring in fife function calls: 'By reference' arguments - to store the asked value in it - in the kni_wrapper interface are missing in the python interface and the return value is used to return the asked value. For example the function 'int  getEncoder(int axis, int &value)' in the kni_wrapper returns the encoder value in the value variable and an error code as return value. In python the function 'getEncoder(axis)' returns the encoder value as return value and ignores errors.
Following functions do have this difference: getDrive, getEncoder, getVelocity, IO_readInput and ModBusTCP_readWord.


Building for Windows:
----------------------------------------------
Precompiled binaries of the KNI Python wrapper come with the KNI sources.
If you want to rebuid them, you will need to have the following tools installed:

- Python 2.5
- nmake
- swig
- Windows Platform SDK at http://www.microsoft.com/downloads

A makefile and a batch skript for rebuilding can be found in the ./win32 directory.





