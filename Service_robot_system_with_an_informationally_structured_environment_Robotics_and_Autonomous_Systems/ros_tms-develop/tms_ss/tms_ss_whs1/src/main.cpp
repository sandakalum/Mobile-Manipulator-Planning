#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>

#define PORT 65001
#define MAX_COUNT 100
#define DB_WRITE 10
#define PEAK 700
#define NOT_PEAK 500
#define MIN_INTERVAL 300
#define PI 3.14159265

using namespace std;

class tms_ss_whs1
{
private:
	ros::NodeHandle nh;
	ros::Publisher db_pub;
	int sock;
public:
	float temp=0;
	int rate=0,p_rate=0;
	int msec=0,p_msec=0;
	int hakei[MAX_COUNT] = {0};
	int count=0;
	double roll,pitch;
	int db_count=0;
	int last_peak_time=-1;
	void spin()
	{
		while(ros::ok()){
			int rcvmsg[3];
			int n = recv(sock,rcvmsg,sizeof(rcvmsg),0);
			if(n<1){
				continue;
			}
			p_msec=msec;
			msec = (rcvmsg[0]>>16)&0xffff;
			hakei[count] = (rcvmsg[0]&0xffff);
			temp = ((rcvmsg[1]>>16)&0xffff)*0.01;
			float acc_x = (((short)(rcvmsg[1]&0xffff)<<16)>>16)*0.01;
			float acc_y = (rcvmsg[2]>>16)*0.01;
			float acc_z = (((short)(rcvmsg[2]&0xffff)<<16)>>16)*0.01;

			if(msec==p_msec) msec+=8;
			else if(msec==(p_msec-8)) msec+=16;
			else if(msec==(p_msec-16)) msec+=24;
			else if(msec==(p_msec-24)) msec+=32;

			double G = sqrt(acc_x*acc_x+acc_y*acc_y+acc_z*acc_z);
			if(acc_y != 0){
				roll = asin(-acc_x/G);
				pitch = atan(acc_z/acc_y);
			}

			ROS_INFO("msec:%d hakei:%d rate:%d",msec,hakei[count],rate);

			int interval = msec-last_peak_time;
			if(interval<-MIN_INTERVAL) interval+=60000;

			if(hakei[count]>PEAK&&interval>MIN_INTERVAL){
				if(last_peak_time==-1)
				last_peak_time = msec;
				else{
					p_rate = rate;
					rate = (int)(1000.0 / (double)interval * 60.0);
					if(rate<30) rate = 0;
					else if(rate>200) rate = p_rate;
					last_peak_time = msec;
				}
			}

			count++;
			db_count++;

			if(db_count>=DB_WRITE){
				db_count=0;
				db_write(1);
			}

			if(count>=MAX_COUNT){
				count=0;
			}
			ros::spinOnce();
		}
		db_write(0);
		close(sock);
	}
	void db_write(int state)
	{
		char data[500];
		char buf[8];
		const char *c1 = "{";
		strcpy(data,c1);
		strcat(data,"\"temp\":");
		sprintf(buf,"%.2f",temp);
		strcat(data,buf);
		strcat(data,", \"rate\":");
		sprintf(buf,"%d",rate);
		strcat(data,buf);
		strcat(data,", \"wave\":[");
		for(int i=0;i<MAX_COUNT;i++){
			int j = ((count+i)>=MAX_COUNT) ? (count+i-MAX_COUNT) : count+i;
			sprintf(buf,"%d",hakei[j]);
			strcat(data,buf);
			if(i!=MAX_COUNT-1) strcat(data,",");
		}
		strcat(data,"]");
		strcat(data,"}");

		ros::Time now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
		tms_msg_db::TmsdbStamped db_msg;

		std::string frame_id("/world");
		db_msg.header.frame_id = frame_id;
		db_msg.header.stamp = now;
		db_msg.tmsdb.clear();
		tms_msg_db::Tmsdb tmpData;

		tmpData.time    = boost::posix_time::to_iso_extended_string(now.toBoost());
		tmpData.name    = "whs1_mybeat";
		tmpData.id      = 3021;
		tmpData.place   = 5001;
		tmpData.sensor  = 3021;
		tmpData.state   = state;
		tmpData.rr			= roll;
		tmpData.rp 			= pitch;
		tmpData.ry			= 0;

		tmpData.note=data;
		db_msg.tmsdb.push_back(tmpData);
		db_pub.publish(db_msg);
	}
	tms_ss_whs1()
	{
		db_pub=nh.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data", 1000);
		sock = socket(AF_INET,SOCK_DGRAM,0);
		struct sockaddr_in s_address;
		s_address.sin_family=AF_INET;
		s_address.sin_addr.s_addr=INADDR_ANY;
		s_address.sin_port=htons(PORT);
		const int on = 1;
		setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &on, sizeof(on));
		bind(sock,(struct sockaddr *)&s_address,sizeof(s_address));
		int val=1;
		ioctl(sock,FIONBIO,&val);
		ROS_INFO("tms_ss_whs1 ready...");
	}
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"tms_ss_whs1");
	tms_ss_whs1 whs1;
	whs1.spin();
	return 0;
}
