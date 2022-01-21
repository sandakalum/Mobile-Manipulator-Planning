#include <stdio.h>
#include "noiseReduction.h"
#include "waveFilter.h"
#include "vitalmonitor.h"
#include "OpenNI.h"
#include "OniSampleUtilities.h"
#include <time.h>
#include <math.h>

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "stdint.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace openni;



float getDepthAverage(DepthPixel* pDepth,int start_x,int start_y, int height, int width, int W) {
	int pixel_num = height * width;
	int Z = start_y * W + start_x;
	float Sum = 0;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			
			Sum += pDepth[Z + j];
		}
		Z = Z + W;
	}
	Sum = Sum / pixel_num;
	return Sum;
}

int main(int argc,char **argv){

	ros::init(argc, argv, "vitalmonitor");
	ros::NodeHandle nh;

	ros::Time start = ros::Time::now(); 

	

	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	openni::VideoStream depth, color;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}


	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}

	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}


	ros::Publisher heartrate_pub = nh.advertise<std_msgs::Float64>("vitalmonitor/heartbeatrate", 50);
	
	ros::Publisher heartbeat_pub = nh.advertise<std_msgs::Float64>("vitalmonitor/heartbeat", 50);
	ros::Publisher breath_pub = nh.advertise<std_msgs::Float64>("vitalmonitor/breath", 50);

	ros::Publisher breathrate_pub = nh.advertise<std_msgs::Float64>("vitalmonitor/brathrate", 50);

	std_msgs::Float64 heartbeatwave, breathwave,heartbeatrate, breathrate;

	VideoFrameRef frame;

	NoiseReduction* Breath_filter = new NoiseReduction(200);
	NoiseReduction* dfilter = new NoiseReduction(50);
	NoiseReduction* heartbeat_filter = new NoiseReduction(10);

	BreathRateFilter* BreathCount = new BreathRateFilter();
	HeartRateFilter* HeartRateCount = new HeartRateFilter();

	float DepthRectangle;
	int loop_count = 0;
	ros::Rate loop_rate(30);


	while (ros::ok()) {
		int changedStreamDummy;
		VideoStream* pStream = &depth;
		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != STATUS_OK)
		{
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}

		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			printf("Read failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}

		if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
		{
			printf("Unexpected frame format\n");
			continue;
		}

		DepthPixel* pDepth = (DepthPixel*)frame.getData();

		DepthRectangle = getDepthAverage(pDepth,160,120,240,320, frame.getWidth());

		Breath_filter->Estimation(DepthRectangle);
		dfilter->Estimation(DepthRectangle);
		heartbeat_filter->Estimation(DepthRectangle);

		float bFout = Breath_filter->C;
		float dFout = dfilter->C;
		float hFout = heartbeat_filter->C;

		ros::Time time_b =  ros::Time::now(); 

		double time = (time_b - start).toSec();
	
		float BreathData = 0, BreathRate = 0;
		float HeartbeatData = 0, Heartbeatrate = 0;
		loop_count++;

		if (loop_count > 500) {
			if(abs(dFout - bFout)> threshold_breath){
				BreathData = BreathCount->BreathFilter((float)(dFout - bFout), (float)time);
			}
			else{
				BreathData = BreathCount->BreathFilter(0.0f, (float)time);
			}
			BreathRate = BreathCount->getBreathRate(BreathData);

			HeartbeatData = HeartRateCount->GetFilter((float)(hFout - dFout) , (float)time);
			Heartbeatrate = HeartRateCount->GetHeartBeatRate(HeartbeatData);
		}

		printf("%f 	, %lf,%f, %f, %f	,  %f\n"	,(float)time,DepthRectangle,hFout - dFout,HeartRateCount->rectOut.Output , HeartbeatData, Heartbeatrate);

		breathrate.data = BreathRate;
		heartbeatrate.data = Heartbeatrate;

		breathwave.data = dFout - bFout;
		heartbeatwave.data = hFout - dFout;
		breath_pub.publish(breathwave);
		breathrate_pub.publish(breathrate);
		heartbeat_pub.publish(heartbeatwave);
		heartrate_pub.publish(heartbeatrate);

		ros::spinOnce();

		loop_rate.sleep();
	}
	openni::OpenNI::shutdown();
	return 0;


}
