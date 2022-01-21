#include <UnitTest++.h>

#include <iostream>

#include "Logging.h"

#include "Model.h"
#include "Contacts.h"
#include "Dynamics.h"
#include "Kinematics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-13;

Model *model = NULL;

unsigned int hip_id,
		 upper_leg_right_id,
		 lower_leg_right_id,
		 foot_right_id,
		 upper_leg_left_id,
		 lower_leg_left_id,
		 foot_left_id;

Body hip_body,
		 upper_leg_right_body,
		 lower_leg_right_body,
		 foot_right_body,
		 upper_leg_left_body,
		 lower_leg_left_body,
		 foot_left_body;

Joint joint_rot_z, joint_rot_y, joint_rot_x;
Joint joint_trans_z, joint_trans_y, joint_trans_x;

VectorNd Q;
VectorNd QDot;
VectorNd QDDot;
VectorNd Tau;

ConstraintSet constraint_set_right;
ConstraintSet constraint_set_left;
ConstraintSet constraint_set_left_flat;
ConstraintSet constraint_set_both;

enum ParamNames {
	ParamSteplength = 0,
	ParamNameLast
};

enum PosNames {
	PosHipPosX,
	PosHipPosY,
	PosHipRotZ,
	PosRightThighRotZ,
	PosRightShankRotZ,
	PosRightAnkleRotZ,
	PosLeftThighRotZ,
	PosLeftShankRotZ,
	PosLeftAnkleRotZ,
	PosNameLast
};

enum StateNames {
	StateHipPosX,
	StateHipPosY,
	StateHipRotZ,
	StateRightThighRotZ,
	StateRightShankRotZ,
	StateRightAnkleRotZ,
	StateLeftThighRotZ,
	StateLeftShankRotZ,
	StateLeftAnkleRotZ,
	StateHipVelX,
	StateHipVelY,
	StateHipRotVelZ,
	StateRightThighRotVelZ,
	StateRightShankRotVelZ,
	StateRightAnkleRotVelZ,
	StateLeftThighRotVelZ,
	StateLeftShankRotVelZ,
	StateLeftAnkleRotVelZ,
	StateNameLast
};

enum ControlNames {
	ControlRightThighRotZ,
	ControlRightKneeRotZ,
	ControlRightAnkleRotZ,
	ControlLeftThighRotZ,
	ControlLeftKneeRotZ,
	ControlLeftAnkleRotZ,
	ControlNameLast
};

enum SegmentLengthsNames {
	SegmentLengthsHip = 0,
	SegmentLengthsThigh,
	SegmentLengthsShank,
	SegmentLengthsFootHeight,
	SegmentLengthsFoot,
	SegmentLengthsNameLast
};

const double ModelMass = 73.;
const double ModelHeight = 1.741;

// absolute lengths!
double segment_lengths[SegmentLengthsNameLast] = {
	0.4346,
	0.4222,
	0.4340,
	0.0317,
	0.2581
};

enum JointLocations {
	JointLocationHip = 0,
	JointLocationKnee,
	JointLocationAnkle,
	JointLocationLast
};

Vector3d joint_location[JointLocationLast] = {
	Vector3d (0., 0., 0.),
	Vector3d (0., - 0.2425 * ModelHeight, 0.),
	Vector3d (0., - 0.2529 * ModelHeight, 0.)
};

enum SegmentMassNames {
	SegmentMassHip,
	SegmentMassThigh,
	SegmentMassShank,
	SegmentMassFoot,
	SegmentMassLast
};

double segment_mass[SegmentMassLast] = {
	0.4346 * ModelMass,
	0.1416 * ModelMass,
	0.0433 * ModelMass,
	0.0137 * ModelMass
};

enum COMNames {
	COMHip,
	COMThigh,
	COMShank,
	COMFoot,
	COMNameLast
};

Vector3d com_position[COMNameLast] = {
	Vector3d (0., 0.3469 * ModelHeight, 0.),
	Vector3d (0., 0.2425 * ModelHeight, 0.),
	Vector3d (0., 0.2529 * ModelHeight, 0.),
	Vector3d (0.0182 * ModelHeight, 0., 0.)
};

enum RGyrationNames {
	RGyrationHip,
	RGyrationThigh,
	RGyrationShank,
	RGyrationFoot,
	RGyrationLast
};

Vector3d rgyration[RGyrationLast] = {
	Vector3d (0.1981, 0.1021, 0.1848),
	Vector3d (0.1389, 0.0629, 0.1389),
	Vector3d (0.1123, 0.0454, 0.1096),
	Vector3d (0.0081, 0.0039, 0.0078)
};

Vector3d heel_point (0., 0., 0.);
Vector3d medial_point (0., 0., 0.);

void init_model () {
	assert (model);

	model->Init();

	model->gravity = Vector3d (0., -9.81, 0.);

	joint_rot_z = Joint (JointTypeRevolute, Vector3d (0., 0., 1.));
	joint_rot_y = Joint (JointTypeRevolute, Vector3d (0., 1., 0.));
	joint_rot_x = Joint (JointTypeRevolute, Vector3d (1., 0., 0.));

	joint_trans_z = Joint (JointTypePrismatic, Vector3d (0., 0., 1.));
	joint_trans_y = Joint (JointTypePrismatic, Vector3d (0., 1., 0.));
	joint_trans_x = Joint (JointTypePrismatic, Vector3d (1., 0., 0.));

	Body null_body (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));

	// hip
	hip_body = Body (segment_mass[SegmentMassHip], com_position[COMHip], rgyration[RGyrationHip]);

	// lateral right
	upper_leg_right_body = Body (segment_mass[SegmentMassThigh], com_position[COMThigh], rgyration[RGyrationThigh]);
	lower_leg_right_body = Body (segment_mass[SegmentMassShank], com_position[COMShank], rgyration[RGyrationShank]);
	foot_right_body = Body (segment_mass[SegmentMassFoot], com_position[COMFoot], rgyration[RGyrationFoot]);

	// lateral left
	upper_leg_left_body = Body (segment_mass[SegmentMassThigh], com_position[COMThigh], rgyration[RGyrationThigh]);
	lower_leg_left_body = Body (segment_mass[SegmentMassShank], com_position[COMShank], rgyration[RGyrationShank]);
	foot_left_body = Body (segment_mass[SegmentMassFoot], com_position[COMFoot], rgyration[RGyrationFoot]);

	// temporary value to store most recent body id
	unsigned int temp_id;

	// add hip to the model (planar, 3 DOF)
	temp_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_trans_x, null_body);
 	temp_id = model->AddBody (temp_id, Xtrans (Vector3d (0., 0., 0.)), joint_trans_y, null_body);
	hip_id = model->AddBody (temp_id, Xtrans (Vector3d (0., 0., 0.)), joint_rot_z, hip_body);

	//
	// right leg
	//
	
	// add right upper leg
	temp_id = model->AddBody (hip_id, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z, upper_leg_right_body);
	upper_leg_right_id = temp_id;

	// add the right lower leg (only one DOF)
	temp_id = model->AddBody (temp_id, Xtrans (joint_location[JointLocationKnee]), joint_rot_z, lower_leg_right_body);
	lower_leg_right_id = temp_id;

	// add the right foot (1 DOF)
	temp_id = model->AddBody (temp_id, Xtrans (joint_location[JointLocationAnkle]), joint_rot_z, foot_right_body);
	foot_right_id = temp_id;

	//
	// left leg
	//

	// add left upper leg
	temp_id = model->AddBody (hip_id, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z, upper_leg_left_body);
	upper_leg_left_id = temp_id;

	// add the left lower leg (only one DOF)
	temp_id = model->AddBody (temp_id, Xtrans (joint_location[JointLocationKnee]), joint_rot_z, lower_leg_left_body);
	lower_leg_left_id = temp_id;

	// add the left foot (1 DOF)
	temp_id = model->AddBody (temp_id, Xtrans (joint_location[JointLocationAnkle]), joint_rot_z, foot_left_body);
	foot_left_id = temp_id;

//	cerr << "--- model created (" << model->dof_count << " DOF) ---" << endl;

	// contact data

	// the contact points for heel and toe
	heel_point.set (-0.05, -0.0317, 0.);
	medial_point.set  (-0.05, -0.0317 + segment_lengths[SegmentLengthsFoot], 0.);

	constraint_set_right.AddConstraint(foot_right_id, heel_point, Vector3d (1., 0., 0.), "right_heel_x");
	constraint_set_right.AddConstraint(foot_right_id, heel_point, Vector3d (0., 1., 0.), "right_heel_y");

	constraint_set_left.AddConstraint(foot_left_id, heel_point, Vector3d (1., 0., 0.), "left_heel_x");
	constraint_set_left.AddConstraint(foot_left_id, heel_point, Vector3d (0., 1., 0.), "left_heel_y");

	constraint_set_both.AddConstraint(foot_right_id, heel_point, Vector3d (1., 0., 0.), "right_heel_x");
	constraint_set_both.AddConstraint(foot_right_id, heel_point, Vector3d (0., 1., 0.), "right_heel_y");
	constraint_set_both.AddConstraint(foot_right_id, heel_point, Vector3d (0., 0., 1.), "right_heel_z");

	constraint_set_both.AddConstraint(foot_left_id, heel_point, Vector3d (1., 0., 0.), "left_heel_x");
	constraint_set_both.AddConstraint(foot_left_id, heel_point, Vector3d (0., 1., 0.), "left_heel_y");
	constraint_set_both.AddConstraint(foot_left_id, heel_point, Vector3d (0., 0., 1.), "left_heel_z");

	constraint_set_right.Bind (*model);
	constraint_set_left.Bind (*model);
	constraint_set_both.Bind (*model);
}

template <typename T>
void copy_values (T *dest, const T *src, size_t count) {
	memcpy (dest, src, count * sizeof (T));
}

TEST ( TestForwardDynamicsContactsLagrangianFootmodel ) {
	model = new Model;

	init_model();

	Q.resize(model->dof_count);
	QDot.resize(model->dof_count);
	QDDot.resize(model->dof_count);
	Tau.resize(model->dof_count);

	Q[0] = -0.2;
	Q[1] = 0.9;
	Q[2] = 0;
	Q[3] = -0.15;
	Q[4] = -0.15;
	Q[5] = 0.1;
	Q[6] = 0.15;
	Q[7] = -0.15;
	Q[8] = 0;

	QDot.setZero();

	Tau[0] = 0;
	Tau[1] = 0;
	Tau[2] = 0;
	Tau[3] = 1;
	Tau[4] = 1;
	Tau[5] = 1;
	Tau[6] = 1;
	Tau[7] = 1;
	Tau[8] = 1;

	Vector3d contact_accel_left;
	Vector3d contact_vel_left;
	Vector3d contact_force = Vector3d::Zero();

	VectorNd QDDot_aba (QDDot);
	VectorNd QDDot_lag (QDDot);
	ForwardDynamics (*model, Q, QDot, Tau, QDDot_aba);
	ForwardDynamicsLagrangian (*model, Q, QDot, Tau, QDDot_lag);

//	cout << "QDDot_aba = " << QDDot_aba.transpose() << endl;
//	cout << "QDDot_lag = " << QDDot_lag.transpose() << endl;

	unsigned int body_id = constraint_set_left.body[0];
	Vector3d contact_point = constraint_set_left.point[0];

	MatrixNd G (3, Q.size());
	CalcPointJacobian (*model, Q, body_id, contact_point, G, true);

//	cout << G << endl;

	ClearLogOutput();

	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_left, QDDot);

//	cout << "C0: " << contact_data_left[0].body_id << ", " << contact_data_left[0].point.transpose() << endl;
//	cout << "C1: " << contact_data_left[1].body_id << ", " << contact_data_left[1].point.transpose() << endl;
//	cout << "td: " << foot_left_id << ", " << heel_point.transpose() << endl;

	contact_force[0] = constraint_set_left.constraint_force[0];
	contact_force[1] = constraint_set_left.constraint_force[1];

	CHECK_EQUAL (body_id, foot_left_id);
	CHECK_EQUAL (contact_point, heel_point);

//	cout << LogOutput.str() << endl;
	contact_accel_left = CalcPointAcceleration (*model, Q, QDot, QDDot, foot_left_id, heel_point);
	contact_vel_left = CalcPointVelocity (*model, Q, QDot, foot_left_id, heel_point);
//	cout << contact_force << endl;
//	cout << contact_accel_left << endl;

	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.).data(), contact_accel_left.data(), 3, TEST_PREC);

	delete model;
}
