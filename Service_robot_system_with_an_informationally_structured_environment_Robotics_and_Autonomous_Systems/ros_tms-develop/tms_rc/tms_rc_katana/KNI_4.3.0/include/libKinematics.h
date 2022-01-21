/* --------------------------------------------------------------------------------
 #
 #	libKinematics.h
 #	Project : Kinematics
 #	author : jhaller
 #	24.04.2008
 #
 # --------------------------------------------------------------------------------*/

/*
* Kinematics library.
*
* General usage: kin_setType or (kin_setMDH and than kin_setImmob if needed,
* kin_setEPC, kin_setEncOff, kin_setRotDir, kin_setAngOff and kin_setAngRan),
* then kin_init, then use the conversion and kinematics functions.
*/

#ifndef WIN32
#define _declspec(dllexport) /**/
#endif                       // ifndef WIN32

extern "C" {

///// STRUCTURES //////////////////////////////////////////////////////

//! Maximum degree of freedom
const int MaxDof = 10;

// This structure holds an int array including its length.
typedef struct IntVector
{
  int length;
  int data[MaxDof];
} IntVector;
// This structure holds a float array including its length.
typedef struct FloatVector
{
  int length;
  float data[MaxDof];
} FloatVector;

///// SETTERS /////////////////////////////////////////////////////////

/*! \brief This sets the robot type
*
* Setting robot type includes setting mDH parameters, immobile flag, degree of
* freedom (dof), degree of mobility (dom) and angles.
*
* type		dof	dom
* 6M90A_F	6	6
* 6M90A_G	6	5
* 6M180		5	5
* 6M90B_F	6	6
* 6M90B_G	6	5
*
* @param type	0: 6M90A_F, 1: 6M90A_G, 2: 6M180, 3: 6M90B_F, 4: 6M90B_G
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setType(int type);

/*! \brief This sets the modified Denavit-Hartenberg parameters
*
* Transformation from previous to current link: move a along (previous) x-axis,
* rotate alpha about (previous) x-axis, move d along (current) z-axis, rotate
* theta about (current) z-axis. Setting the mDH parameters includes setting dof,
* adjusting immobile flag (all free) and dom (= dof).
*
* The length of the vectors need to be the same.
*
* @param theta	angle about z-axis
* @param d		distance along x-axis
* @param a		distance previous to current along previous x-axis
* @param alpha	angle about previous x-axis
* @param typeNr	0: 6M90A_F, 1: 6M90A_G, 2: 6M180, 3: 6M90B_F, 4: 6M90B_G, -1: other
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setMDH(FloatVector* theta, FloatVector* d, FloatVector* a, FloatVector* alpha,
                                    int typeNr = -1);

/*! \brief This sets the link length parameters (part of mDH-parameters)
*
* Setting link length parameters requires the type to be set to 0 (6M90A_F),
* 1 (6M90A_G), 2 (6M180), 3 (6M90B_F) or 4 (6M90B_G). This is done using the
* setType or setModifiedDH function that set the mDH parameters so they can be
* adjusted. The length of the vector has to be four.
*
* @param links	length of the links in m
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setLinkLen(FloatVector* links);

/*! \brief This sets the immobile flag of the last joint
*
* Flag for the last joint if it is free (0) or locked (1). Setting the
* immobile flag includes adjusting dom. Type or mDH parameters have to be set.
*
* @param immobile	last joint immobile flag: 0 for free, 1 for locked
*						(immobile)
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setImmob(int immobile);

/*! \brief This sets the encoders per cycle
*
* Number of encoders in a full cycle of each joint. Length of epc needs to be
* equal to DOM set with Type or mDH parameters.
*
* @param epc	encoders pec cycle
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setEPC(IntVector* epc);

/*! \brief This sets the encoder offsets
*
* The encoder values at the calibration stops. Length of encOffset needs to be
* equal to DOM set with Type or mDH parameters.
*
* @param encOffset	encoder values at calibration stops
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setEncOff(IntVector* encOffset);

/*! \brief This sets the rotation direction
*
* The rotation direction of the joints: +1 if encoders and angles (mDH
* convention) grow in same direction, -1 if encoders and angles grow in opposit
* direction. Length of rotDir needs to be equal to DOM set with Type or mDH
* parameters.
*
* @param rotDir	rotation direction of the joints
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setRotDir(IntVector* rotDir);

/*! \brief This sets the angle offsets
*
* The angles at the calibration stops in mDH convention! Length of angleOffset
* needs to be equal to DOM set with Type or mDH parameters.
*
* @param angleOffset	angle values at calibration stops in mDH
* @return				0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setAngOff(FloatVector* angleOffset);

/*! \brief This sets the angle range
*
* The angle range from angle offset to angle stop in mDH convention (negative if
* angleOffset > angleStop)! Length of angleRange needs to be equal to DOM set
* with Type or mDH parameters.
*
* @param angleRange	angle range from angle offset to angle stop
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setAngRan(FloatVector* angleRange);

/*! \brief This sets the tcp offset
*
* Offset from the flange to the effective tcp.
*
* @param tcpOffset	(x, y, z, psi) offset in m and rad respectively where psi
*						is rotation about x-axis of tool coordinate system
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_setTcpOff(FloatVector* tcpOffset);

///// GETTERS /////////////////////////////////////////////////////////

/*! \brief Get the robot type
*
* Get the robot type
*
* @return	0 for 6M90A_F, 1 for 6M90A_G, 2 for 6M180, 3 for 6M90B_F,
*			4 for 6M90B_G, -1 for other or not set yet
*/
_declspec(dllexport) int kin_getType();

/*! \brief Get the maximum degree of freedom
*
* Get the maximum degree of freedom (length of array in the vectors)
*
* @return	Maximum degree of freedom
*/
_declspec(dllexport) int kin_getMaxDOF();

/*! \brief Get the degree of freedom
*
* Get the degree of freedom
*
* @return	Degree of freedom or -1 if not set yet
*/
_declspec(dllexport) int kin_getDOF();

/*! \brief Get the degree of mobility
*
* Get the degree of mobility
*
* @return	Degree of mobility or -1 if not set yet
*/
_declspec(dllexport) int kin_getDOM();

/*! \brief Get the modified Denavit-Hartenberg parameters
*
* @param theta	vector to write in angle about z-axis
* @param d		vector to write in distance along x-axis
* @param a		vector to write in distance previous to current along previous
*					x-axis
* @param alpha	vector to write in angle about previous x-axis
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getMDH(FloatVector* theta, FloatVector* d, FloatVector* a, FloatVector* alpha);

/*! \brief Get the immobile flag of the last joint
*
* @return			0 if mobile, 1 if immobile, < 0 if failed
*/
_declspec(dllexport) int kin_getImmob();

/*! \brief Get the encoders per cycle
*
* @param epc	vector to write in encoders pec cycle
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getEPC(IntVector* epc);

/*! \brief Get the encoder offsets
*
* @param encOffset	vector to write in encoder values at calibration stops
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getEncOff(IntVector* encOffset);

/*! \brief Get the rotation direction
*
* @param rotDir	vector to write in rotation direction of the joints
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getRotDir(IntVector* rotDir);

/*! \brief Get the angle offsets
*
* @param angleOffset	vector to write in angle values at calibration stops in
*							mDH
* @return				0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getAngOff(FloatVector* angleOffset);

/*! \brief Get the angle range
*
* @param angleRange		vector to write in angle range from angle offset to angle
*							stop
* @return				0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getAngRan(FloatVector* angleRange);

/*! \brief Get the tcp offset
*
* @param tcpOffset	vector to write in (x, y, z, psi) offset
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getTcpOff(FloatVector* tcpOffset);

/*! \brief Get the version number of the library
*
* @param version	vector to write in version (major, minor, revision)
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_getVersion(IntVector* version);

///// INITIALIZATION //////////////////////////////////////////////////

/*! \brief This initializes the kinematic
*
* Initialize the kinematic. Parameters mDH, dof, angle offset and angle range
* (SetType or SetMDH and SetAngleOffset and SetAngleRange) have to be set
* prior to initialization of the kinematic!
*
* @return	0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_init();

///// CLEANUP /////////////////////////////////////////////////////////

/*! \brief This cleans the kinematics library
*
* Free memory allocated in setType() / setMDH() before closing the library
*
* @return	0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_clean();

///// CONVERSION AND KINEMATICS ///////////////////////////////////////

/*! \brief This converts angles from Katana4D to mDH convention
*
* Length of angleK4D needs to be equal to DOM set with Type or mDH parameters.
*
* @param angleK4D	angle in K4D convention
* @param angleMDH	angle in mDH convention
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_K4D2mDHAng(FloatVector* angleK4D, FloatVector* angleMDH);

/*! \brief This converts angles from mDH to Katana4D convention
*
* Length of angleMDH needs to be equal to DOM set with Type or mDH parameters.
*
* @param angleMDH	angle in mDH convention
* @param angleK4D	angle in K4D convention
* @return			0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_mDH2K4DAng(FloatVector* angleMDH, FloatVector* angleK4D);

/*! \brief This converts encoders to angles in mDH convention
*
* Length of enc needs to be equal to DOM set with Type or mDH parameters.
*
* @param enc	encoders
* @param angle	angles in mDH convention
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_enc2rad(IntVector* enc, FloatVector* angle);

/*! \brief This converts angles in mDH convention to encoders
*
* Length of angle needs to be equal to DOM set with Type or mDH parameters.
*
* @param angle	angle in mDH convention
* @param enc	encoders
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_rad2enc(FloatVector* angle, IntVector* enc);

/*! \brief This calculates the direct kinematics
*
* Length of angle needs to be equal to DOM set with Type or mDH parameters.
*
* @param angle	angles in mDH convention
* @param pose	pose of the TCP [x, y, z, phi, theta, psi]
* @return		0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_DK(FloatVector* angle, FloatVector* pose);

/*! \brief This calculates the inverse kinematics
*
* Length of prev needs to be equal to DOM set with Type or mDH parameters.
*
* @param pose			pose of the TCP [x, y, z, phi, theta, psi]
* @param prev			previous angles (starting point)
* @param angle			angles in mDH convention
* @param maxBisection	maximum number of bisections done, if no solution found
* @return				0 if successful, < 0 if failed
*/
_declspec(dllexport) int kin_IK(FloatVector* pose, FloatVector* prev, FloatVector* angle, int maxBisection = 0);

}  // extern c
