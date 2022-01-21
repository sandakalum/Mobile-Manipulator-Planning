#include "rbdl.h"

struct FixedBase3DoF {
	FixedBase3DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;
		model->Init();

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *      Z
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint(
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);

		body_b_id = model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_c_id = model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

		point_position = Vector3d::Zero (3);
		point_acceleration = Vector3d::Zero (3);

		ref_body_id = 0;

		ClearLogOutput();
	}
	~FixedBase3DoF () {
		delete model;
	}
	
	RigidBodyDynamics::Model *model;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	RigidBodyDynamics::Body body_a, body_b, body_c;
	RigidBodyDynamics::Joint joint_a, joint_b, joint_c;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;

	RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};

struct FixedBase6DoF {
	FixedBase6DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;
		model->Init();

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 3 DoF (rot.) joint at base
		 * 3 DoF (rot.) joint child origin
		 *
		 *          X Contact point (ref child)
		 *          |
		 *    Base  |
		 *   / body |
		 *  O-------*
		 *           \
		 *             Child body
		 */

		// base body (3 DoF)
		base_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		base_rot_z_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_z, base_rot_z);

		base_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		base_rot_y_id = model->AppendBody (Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_y, base_rot_y);

		base_rot_x = Body (
				1.,
				Vector3d (0.5, 0., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_base_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		base_rot_x_id = model->AddBody (base_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_x, base_rot_x);

		// child body (3 DoF)
		child_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		child_rot_z_id = model->AddBody (base_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_rot_z, child_rot_z);

		child_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		child_rot_y_id = model->AddBody (child_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_y, child_rot_y);

		child_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		child_rot_x_id = model->AddBody (child_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_x, child_rot_x);

		Q = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		Tau = VectorNd::Constant (model->mBodies.size() - 1, 0.);

		contact_body_id = child_rot_x_id;
		contact_point = Vector3d  (0.5, 0.5, 0.);
		contact_normal = Vector3d  (0., 1., 0.);

		ClearLogOutput();
	}
	
	~FixedBase6DoF () {
		delete model;
	}
	RigidBodyDynamics::Model *model;

	unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id,
		base_body_id;

	RigidBodyDynamics::Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x;

	RigidBodyDynamics::Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;

	unsigned int contact_body_id;
	RigidBodyDynamics::Math::Vector3d contact_point;
	RigidBodyDynamics::Math::Vector3d contact_normal;
	RigidBodyDynamics::ConstraintSet constraint_set;
};

struct FloatingBase12DoF {
	FloatingBase12DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;
		model->Init();

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 3 DoF (rot.) joint at base
		 * 3 DoF (rot.) joint child origin
		 *
		 *          X Contact point (ref child)
		 *          |
		 *    Base  |
		 *   / body |
		 *  O-------*
		 *           \
		 *             Child body
		 */

		base_rot_x = Body (
				1.,
				Vector3d (0.5, 0., 0.),
				Vector3d (1., 1., 1.)
				);
		base_rot_x_id = model->SetFloatingBaseBody(base_rot_x);

		// child body 1 (3 DoF)
		child_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		child_rot_z_id = model->AddBody (base_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_rot_z, child_rot_z);

		child_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		child_rot_y_id = model->AddBody (child_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_y, child_rot_y);

		child_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		child_rot_x_id = model->AddBody (child_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_x, child_rot_x);

		// child body (3 DoF)
		child_2_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_2_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		child_2_rot_z_id = model->AddBody (child_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_2_rot_z, child_2_rot_z);

		child_2_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_2_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		child_2_rot_y_id = model->AddBody (child_2_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_2_rot_y, child_2_rot_y);

		child_2_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_2_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		child_2_rot_x_id = model->AddBody (child_2_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_2_rot_x, child_2_rot_x);

		Q = VectorNd::Constant (model->dof_count, 0.);
		QDot = VectorNd::Constant (model->dof_count, 0.);
		QDDot = VectorNd::Constant (model->dof_count, 0.);
		Tau = VectorNd::Constant (model->dof_count, 0.);

		ClearLogOutput();
	}
	
	~FloatingBase12DoF () {
		delete model;
	}
	RigidBodyDynamics::Model *model;

	unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id,
		child_2_rot_z_id, child_2_rot_y_id,child_2_rot_x_id,
		base_body_id;

	RigidBodyDynamics::Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x,
		child_2_rot_z, child_2_rot_y, child_2_rot_x;

	RigidBodyDynamics::Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x,
		joint_child_2_rot_z, joint_child_2_rot_y, joint_child_2_rot_x;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;
};

struct SimpleFixture {
	SimpleFixture () {
		ClearLogOutput();
		model = new RigidBodyDynamics::Model;
		model->Init();
		model->gravity = RigidBodyDynamics::Math::Vector3d (0., -9.81, 0.);
	}
	~SimpleFixture () {
		delete model;
	}
	void ResizeVectors () {
		Q = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
		QDot = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
		QDDot = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
		Tau = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
	}

	RigidBodyDynamics::Model *model;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;
};

struct FixedJoint2DoF {
	FixedJoint2DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;
		model->Init();

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *      Z
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint(
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint (JointTypeFixed);

		body_b_id = model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_c_id = model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);

		point_position = Vector3d::Zero (3);
		point_acceleration = Vector3d::Zero (3);

		ref_body_id = 0;

		ClearLogOutput();
	}
	~FixedJoint2DoF () {
		delete model;
	}
	
	RigidBodyDynamics::Model *model;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	RigidBodyDynamics::Body body_a, body_b, body_c;
	RigidBodyDynamics::Joint joint_a, joint_b, joint_c;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;

	RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};

/** \brief Fixture that contains two models of which one has one joint fixed.
 */
struct FixedAndMovableJoint {
	FixedAndMovableJoint () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model_movable = new Model;
		model_movable->Init();

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *      Z
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint(
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_a_id = model_movable->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);

		body_b_id = model_movable->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_c_id = model_movable->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		QDot = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		QDDot = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		Tau = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		C_movable = VectorNd::Zero ((size_t) model_movable->dof_count);
		H_movable = MatrixNd::Zero ((size_t) model_movable->dof_count, (size_t) model_movable->dof_count);

		// Assemble the fixed joint model
		model_fixed = new Model;
		model_fixed->Init();

		body_a_fixed_id = model_fixed->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
		Joint joint_fixed (JointTypeFixed);
		body_b_fixed_id = model_fixed->AddBody(body_a_fixed_id, Xtrans(Vector3d(1., 0., 0.)), joint_fixed, body_b);
		body_c_fixed_id = model_fixed->AddBody(body_b_fixed_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		QDot_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		QDDot_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		Tau_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		C_fixed = VectorNd::Zero ((size_t) model_fixed->dof_count);
		H_fixed = MatrixNd::Zero ((size_t) model_fixed->dof_count, (size_t) model_fixed->dof_count);

		point_position = Vector3d::Zero (3);
		point_acceleration = Vector3d::Zero (3);

		ref_body_id = 0;

		ClearLogOutput();
	}

	~FixedAndMovableJoint () {
		delete model_movable;
		delete model_fixed;
	}
	RigidBodyDynamics::Math::VectorNd CreateDofVectorFromReducedVector (const RigidBodyDynamics::Math::VectorNd &q_fixed) {
		assert (q_fixed.size() == model_fixed->dof_count);

		RigidBodyDynamics::Math::VectorNd q_movable (model_movable->dof_count);

		q_movable[0] = q_fixed[0];
		q_movable[1] = 0.;
		q_movable[2] = q_fixed[1];

		return q_movable;
	}

	RigidBodyDynamics::Math::MatrixNd CreateReducedInertiaMatrix(const RigidBodyDynamics::Math::MatrixNd &H_movable) {
		assert (H_movable.rows() == model_movable->dof_count);
		assert (H_movable.cols() == model_movable->dof_count);
		RigidBodyDynamics::Math::MatrixNd H (model_fixed->dof_count, model_fixed->dof_count);

		H (0,0) = H_movable(0,0); H (0,1) = H_movable(0,2);
		H (1,0) = H_movable(2,0); H (1,1) = H_movable(2,2);

		return H;
	}
	
	RigidBodyDynamics::Model *model_fixed;
	RigidBodyDynamics::Model *model_movable;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	unsigned int body_a_fixed_id, body_b_fixed_id, body_c_fixed_id;

	RigidBodyDynamics::Body body_a, body_b, body_c;
	RigidBodyDynamics::Joint joint_a, joint_b, joint_c;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;
	RigidBodyDynamics::Math::VectorNd C_movable;
	RigidBodyDynamics::Math::MatrixNd H_movable;
	
	RigidBodyDynamics::Math::VectorNd Q_fixed;
	RigidBodyDynamics::Math::VectorNd QDot_fixed;
	RigidBodyDynamics::Math::VectorNd QDDot_fixed;
	RigidBodyDynamics::Math::VectorNd Tau_fixed;
	RigidBodyDynamics::Math::VectorNd C_fixed;
	RigidBodyDynamics::Math::MatrixNd H_fixed;

	RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};


