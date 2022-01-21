#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <tms_msg_ss/Beacon.h>
#include "kalman-Ndof.hpp"

using namespace std;
using namespace Eigen;

#define N 4

ros::Subscriber beacon_sub;
ros::Publisher marker_pub;

typedef struct {
	Vector3d loc;
	double r;
} beacon;

vector<beacon> rasp_pis;
beacon rasp_pi[N];
Kalman *kalman;

#define SD_DIST 0.1 // Error of beacon [m]
#define sqr(var) ((var)*(var))
#define dist2(var1, var2) (sqr(var1[0]-var2[0])+sqr(var1[1]-var2[1])+sqr(var1[2]-var2[2])) // square of distance

#define REPEAT_LIMIT 1000

//#define LOGGING
// #define DEBUG 1

Vector3d calc_position(vector<beacon> beacons, Vector3d initpos)
{
	Vector3d pos(0, 0, 0);
	int n = beacons.size();

	MatrixXd A(n, 3);
	VectorXd L(n);
	MatrixXd SigmaL(n, n);
	MatrixXd K(n, n);
	MatrixXd Sigma(n, n);
	VectorXd Error(3);
	double gain = 0.01;//0.1;

	pos = initpos;
	int repeat = 0;

	while (1) {
		repeat++;
		if(repeat>REPEAT_LIMIT)
			break;
		A.setZero();
		L.setZero();
		SigmaL.setZero();
		Sigma.setZero();

		K.setZero();
		Error.setZero();

		for (int i = 0; i<n; i++) {
			double distance = dist2(beacons[i].loc, pos);

			A(i, 0) = pos[0] - beacons[i].loc[0];
			A(i, 1) = pos[1] - beacons[i].loc[1];
			A(i, 2) = pos[2] - beacons[i].loc[2];

			L(i) = (sqr(beacons[i].r) - dist2(beacons[i].loc, pos)) / 2.0;

			K(i, i) = beacons[i].r;
			Sigma(i, i) = sqr(SD_DIST);
		}

		SigmaL = K * Sigma * K.transpose();

		Matrix3d Covariance;
		Covariance = (A.transpose() * SigmaL.inverse() * A).inverse();
		Error = Covariance * A.transpose() * SigmaL.inverse() * L;

		#ifdef LOGGING
		std::ofstream outf("test.txt");
		static int testCount = 1;
		outf << "[" << testCount++ << "]" << std::endl << std::endl;
		outf << "Covariance = " << std::endl << Covariance << std::endl << std::endl;
		outf << "det(S-1) = " << std::endl << (SigmaL.inverse()).determinant() << std::endl << std::endl;
		outf << "det(S) = " << std::endl << SigmaL.determinant() << std::endl << std::endl;
		outf << "det(Covariance) = " << std::endl << Covariance.determinant() << std::endl << std::endl;
		outf << "A.transpose() = " << std::endl << A.transpose() << std::endl << std::endl;
		outf << "SigmaL = " << std::endl << SigmaL << std::endl << std::endl;
		outf << "Error = " << std::endl << Error << std::endl << std::endl;
		outf << "SigmaL -1 = " << std::endl << SigmaL.inverse() << std::endl << std::endl;
		outf << "L = " << std::endl << L << std::endl << std::endl;

		outf << "C * at = " << std::endl << Covariance * A.transpose() << std::endl << std::endl;
		outf << "C * at * SigmaL-1 = " << std::endl << Covariance * A.transpose() * SigmaL.inverse() << std::endl << std::endl;
		outf << "C * at * SigmaL-1 * L = " << std::endl << Covariance * A.transpose() * SigmaL.inverse() * L << std::endl << std::endl;
		outf.flush();

		outf.close();
		#endif

		if (sqrt(sqr(Error(0)) + sqr(Error(1)) + sqr(Error(2))) > 1000.0) {
			return Vector3d(0,0,0); // �v�Z�����U
		}

		// reNew parent position
		pos[0] += gain * Error(0);
		pos[1] += gain * Error(1);
		pos[2] += gain * Error(2);

#if DEBUG
		cout << "Error " << sqrt(sqr(Error(0)) + sqr(Error(1)) + sqr(Error(2))) << endl;
		cout << "Pos " << std::endl << pos << std::endl << std::endl;
		cout << "Covariance = " << std::endl << Covariance << std::endl << std::endl;
#endif

		// do repeat until (dx,dy,dz) is under than 1 mm
		if (sqrt(sqr(Error(0)) + sqr(Error(1)) + sqr(Error(2))) < 0.001) {
			break;
		}
	}

	return pos;

}

void drawRaspi(beacon pi,int id,float r,float g,float b,float a)
{
	uint32_t shape = visualization_msgs::Marker::SPHERE;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/world_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "beacon";
	marker.id = id;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = pi.loc[0];
	marker.pose.position.y = pi.loc[1];
	marker.pose.position.z = pi.loc[2];

	marker.scale.x = marker.scale.y = marker.scale.z = pi.r*2;

	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
}

void drawPos(Vector3d pos,int id,float r,float g,float b,float a)
{
	uint32_t shape = visualization_msgs::Marker::SPHERE;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/world_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "beacon";
	marker.id = id;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = pos[0];
	marker.pose.position.y = pos[1];
	marker.pose.position.z = pos[2];//0.8

	marker.scale.x = marker.scale.y = 0.2;
	marker.scale.z = 0.2;//1.6

	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
}

ros::Time last_time;
ros::Time now;
Vector3d pos(8,3,0.8);
Vector3d last_pos(8,3,0.8);
Vector3d kf_pos(8,3,0.8);
void Callback(const tms_msg_ss::Beacon &msg)
{
	if(msg.pi_id<N)
		rasp_pi[msg.pi_id].r = msg.distance;

	ROS_INFO("%f %f %f %f",rasp_pi[0].r,rasp_pi[1].r,rasp_pi[2].r,rasp_pi[3].r);

  drawRaspi(rasp_pi[0],0,1.0,0.0,0.0,0.2);
	drawRaspi(rasp_pi[1],1,0.0,1.0,0.0,0.2);
	drawRaspi(rasp_pi[2],2,0.0,1.0,1.0,0.2);
	drawRaspi(rasp_pi[3],3,1.0,1.0,0.0,0.2);

	rasp_pis.clear();
	rasp_pis.push_back(rasp_pi[0]);
	rasp_pis.push_back(rasp_pi[1]);
	rasp_pis.push_back(rasp_pi[2]);
	rasp_pis.push_back(rasp_pi[3]);

	last_pos = pos;
	ROS_INFO("calc");
	pos = calc_position(rasp_pis, pos);
	ROS_INFO("calcend");
	ROS_INFO("pos:(%f,%f,%f)",pos[0],pos[1],pos[2]);

	if(pos[0]==0&&pos[1]==0&&pos[2]==0){
		return;
	}

	last_time = now;
	now = ros::Time::now();
	double delta_t = (now.sec+now.nsec*0.000000001)-(last_time.sec+last_time.nsec*0.000000001);
	ROS_INFO("callback:%f",delta_t);

	drawPos(pos,10,1.0,1.0,1.0,1.0);

	double velX = (pos[0]-last_pos[0])/delta_t;
	double velY = (pos[1]-last_pos[1])/delta_t;
	double velZ = (pos[2]-last_pos[2])/delta_t;

	static int count = 0;

	if(kalman == NULL)
		return;

	if(count==0)
	{
		count++;
		return;
	}
	else if(count==1)
	{
		kalman->init(0.1,1.0,0.1);//(0.1,0.1,1.0);
		double C[3][6];
		memset(C,0,sizeof(C));
		C[0][0]=C[1][1]=C[2][2]=1;
		kalman->setC((double *)&C);
		kalman->setX(0,pos[0]);
		kalman->setX(1,pos[1]);
		kalman->setX(2,pos[2]);
		kalman->setX(3,velX);
		kalman->setX(4,velY);
		kalman->setX(5,velZ);

		count++;
	}

	double A[3][6];
	memset(A,0,sizeof(A));
	A[0][0]=A[1][1]=A[2][2]=1;
	A[0][3]=A[1][4]=A[2][5]=delta_t;

	double obs[6]={pos[0],pos[1],pos[2],velX,velY,velZ};
	double input[3]={0,0,0};

	kalman->update(obs,input);

	kf_pos[0]=kalman->getX(0);
	kf_pos[1]=kalman->getX(1);
	kf_pos[2]=kalman->getX(2);


	ROS_INFO("kf_pos:(%f,%f,%f)",kf_pos[0],kf_pos[1],kf_pos[2]);

	drawPos(kf_pos,11,1.0,0.0,0.0,1.0);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tms_ss_beacon");
	ros::NodeHandle nh;

	kalman = new Kalman(6,6,3);

	now = ros::Time::now();

	beacon_sub = nh.subscribe("/tms_ss_beacon",1,&Callback);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",1);

	ROS_INFO("READY");

	rasp_pi[0].loc << 7.66, 5.62, 0.82;
	rasp_pi[1].loc << 10.12, 3.90, 0.58;
	rasp_pi[2].loc << 8.74, 0.31, 0.92;
	rasp_pi[3].loc << 5.65, 1.66, 0.73;

	rasp_pi[0].r = 3;
	rasp_pi[1].r = 1.5;
	rasp_pi[2].r = 2;
	rasp_pi[3].r = 4;

	ros::spin();

	return 0;
}
