/*#===========================================================================
# KNI Python interface Swig file
# copyright Neuronics Switzerland 2005-2008
# Authors: see AUTHORS file in the KNI root directory 
#===========================================================================*/

%module(package="KNI") KNI


%{
#include "../include/kni_wrapper/kni_wrapper.h"
%}

%ignore getDrive(int, int&);
%rename(getDrive) getDrive_new(int);
%ignore getEncoder(int, int&);
%rename(getEncoder) getEncoder_new(int);
%ignore getVelocity(int, int&);
%rename(getVelocity) getVelocity_new(int);
%ignore IO_readInput(int, int&);
%rename(IO_readInput) IO_readInput_new(int);
%ignore ModBusTCP_readWord(int, int&);
%rename(ModBusTCP_readWord) ModBusTCP_readWord_new(int);

%include ../include/kni_wrapper/kni_wrapper.h

%inline %{
/* wrapper for reference arguments */
int getDrive_new(int axis) {
	int drive;
	getDrive(axis, drive);
	return drive;
}
int getEncoder_new(int axis) {
	int encoder;
	getEncoder(axis, encoder);
	return encoder;
}
int getVelocity_new(int axis) {
	int velocity;
	getVelocity(axis, velocity);
	return velocity;
}
int IO_readInput_new(int inputNr) {
	int input;
	IO_readInput(inputNr, input);
	return input;
}
int ModBusTCP_readWord_new(int address) {
	int word;
	ModBusTCP_readWord(address, word);
	return word;
}
%}


