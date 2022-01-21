Katana Native Interface
-----------------------

Katana Native Interface is a C++ library for programmers who would like to
write their own programs, but don't want to implement the protocol and
device stuff katana is using.

Please also read INSTALL.txt and documentation in "doc/*" before starting.

- doc:
------

	Contains the KNI Manual / API with a complete class and method list,
	dependency graphs, etc.
	Type 'make doc' to build the API documentation. in html and pdf.

- lib:
------

	All the static and dynamic libraries are placed here (win32/linux).


- demo:
-------

	Contains a few sample demo programs, which show how the library
	should be used to develop applications - it's the best place to learn
	quickly how to use the libraries.

	- control:		demonstrates most of the high-level-functions available
				in the KNI.
	- commands:		similar to the control demo, but uses low level firmware commands directly
	- kni_wrapper:		a demo of how to use the 'C' based KNI wrapper interface
	- kni_labview:		a demontration .vi for an integration into LabView
	- kni_matlab:		a demontration .m for an integration into Matlab


Please have a look at the "KNI Manual" and the CLASS-Documentation in the 'doc/html'-folder.


Neuronics AG
Software Development

Last Update: 2009-01-30
