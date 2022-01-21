#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <behaviortree_ros/AddTwoInts.h>
#include <behaviortree_ros/FibonacciAction.h>
#include <map>
#include <iterator>
#include <vector>
#include <string>
#include <chrono>
#include <ctime>
#include <cmath>
#include <boost/make_shared.hpp>
#include <bits/stdc++.h>

// Srvs & msgs
#include <kuka_msgs/ArmPose.h>
#include <kuka_msgs/Place.h>
#include <kuka_msgs/ObjectPose.h>
#include <kuka_msgs/BestWorkstation.h>
#include <kuka_msgs/PoseExtraction.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <ros/package.h>

using namespace BT;

const std::string GLOBAL_FRAME = "base_footprint";

std::string object_ = "leaf_cell";
int bin = 1;
float detection_threshold = 0.016;
std::map<int, geometry_msgs::Point> bins_loc;

int number_of_poses = 1;
std::vector<double> pose_vector_param;
std::map<int, geometry_msgs::PoseStamped> candidate_poses_dictionary;
geometry_msgs::PoseStamped temp_pose;

tf2_ros::Buffer tfBuffer;

class Timer
{
public:
    void start()
    {
        m_StartTime = std::chrono::system_clock::now();
        m_bRunning = true;
    }
    
    void stop()
    {
        m_EndTime = std::chrono::system_clock::now();
        m_bRunning = false;
    }
    
    double elapsedMilliseconds()
    {
        std::chrono::time_point<std::chrono::system_clock> endTime;
        
        if(m_bRunning)
        {
            endTime = std::chrono::system_clock::now();
        }
        else
        {
            endTime = m_EndTime;
        }
        
        return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - m_StartTime).count();
    }
    
    double elapsedSeconds()
    {
        return elapsedMilliseconds() / 1000.0;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> m_StartTime;
    std::chrono::time_point<std::chrono::system_clock> m_EndTime;
    bool                                               m_bRunning = false;
};



class PrintValue : public BT::SyncActionNode
{
public:
	PrintValue(const std::string &name, const BT::NodeConfiguration &config)
		: BT::SyncActionNode(name, config) {}

	BT::NodeStatus tick() override
	{
		std::string value;
		if (getInput("message", value))
		{
			std::cout << "PrintValue: " << value << std::endl;
			return NodeStatus::SUCCESS;
		}
		else
		{
			std::cout << "PrintValue FAILED " << std::endl;
			return NodeStatus::FAILURE;
		}
	}

	static BT::PortsList providedPorts()
	{
		return {BT::InputPort<std::string>("message")};
	}
};

void getxyzTransform(tf2::Vector3 &tf2_vec, std::string from, std::string to)
{
	geometry_msgs::TransformStamped world2base_footprint;
	while (ros::ok())
	{
		try
		{
			world2base_footprint = tfBuffer.lookupTransform(from, to, ros::Time(0));
			break;
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
	}
	tf2::Transform tf2_world2base_footprint;
	
	tf2::fromMsg(world2base_footprint.transform, tf2_world2base_footprint);
	tf2_vec = tf2_world2base_footprint * tf2_vec;
}

void transform2Map(tf2::Vector3 &tf2_vec)
{
	geometry_msgs::TransformStamped world2base_footprint;
	while (ros::ok())
	{
		try
		{
			world2base_footprint = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
			break;
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
	}
	tf2::Transform tf2_world2base_footprint;
	;
	tf2::fromMsg(world2base_footprint.transform, tf2_world2base_footprint);
	tf2_vec = tf2_world2base_footprint * tf2_vec;
}

class ObjectReachable : public RosServiceNode<kuka_msgs::ObjectPose>
{

public:
	ObjectReachable(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf) : RosServiceNode<kuka_msgs::ObjectPose>(handle, node_name, conf) {}

	static PortsList providedPorts()
	{
		return {
			InputPort<geometry_msgs::PoseStamped>("pose")};
	}

	void sendRequest(RequestType &request) override
	{
		geometry_msgs::PoseStamped temp_pose;
		getInput<geometry_msgs::PoseStamped>("pose", temp_pose);
		tf2::Vector3 temp_vec;
		temp_vec.setValue(temp_pose.pose.position.x, temp_pose.pose.position.y, temp_pose.pose.position.z);
		// transform2Map(temp_vec);
		// temp_pose.header.frame_id = "base_footprint";
		request.obj_pose.pose.position.x = temp_vec.getX();
		request.obj_pose.pose.position.y = temp_vec.getY();
		request.obj_pose.pose.position.z = temp_vec.getZ();
		ROS_INFO("Object Reachable: sending request");
	}

	NodeStatus onResponse(const ResponseType &res) override
	{
		ROS_INFO("Object Reachable: response received");
		if (res.status)
		{
			ROS_INFO("Object Reachable!");
			return NodeStatus::SUCCESS;
		}
		else
		{
			ROS_INFO("Object Not Reachable!");
			return NodeStatus::FAILURE;
		}
	}

	virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
	{
		ROS_ERROR("Object Reachable object request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}
};

class BestPose : public RosServiceNode<kuka_msgs::BestWorkstation>
{

private:

	ros::NodeHandle nh_;
	boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> mobile_base_pose;
	int home_ = 3;
	int view_point_;

public:
	BestPose(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf) : nh_(handle), 
						RosServiceNode<kuka_msgs::BestWorkstation>(handle, node_name, conf) {}

	static PortsList providedPorts()
	{
		return {
			InputPort<geometry_msgs::PoseStamped>("target_pose"),
			InputPort<int>("home"),
			InputPort<int>("view_point"),
			OutputPort<int>("selected_pose")};
	}

	void sendRequest(RequestType &request) override
	{
		getInput<int>("view_point", view_point_);
		// If the robot is comming back from placing the object we select the neareast location 
		// given the currrent mobile pose, otherwise we send the detected object pose
		if (view_point_)
		{
			//mobile_base_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", nh_);
			request.view_point = view_point_;
		}
		else
		{
			geometry_msgs::PoseStamped temp_pose;
			getInput<geometry_msgs::PoseStamped>("target_pose", temp_pose);
			tf2::Vector3 temp_vec;
			temp_vec.setValue(temp_pose.pose.position.x, temp_pose.pose.position.y - 0.15, temp_pose.pose.position.z);

			// We transform to map frame given thaht the best poses are in this frame  and not in the base_footprint frame
			transform2Map(temp_vec);
			temp_pose.header.frame_id = "map";
			request.target_pose.pose.position.x = temp_vec.getX();
			request.target_pose.pose.position.y = temp_vec.getY();
			request.target_pose.pose.position.z = temp_vec.getZ();
		}

		ROS_INFO("Best Pose service: sending request");
	}

	NodeStatus onResponse(const ResponseType &rep) override
	{
		ROS_INFO("Best Pose service: response received");
		setOutput<int>("selected_pose", rep.workstation_id);

		return NodeStatus::SUCCESS;
	}

	virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
	{
		ROS_ERROR("Best Pose request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}
};

class NavigateTo : public RosActionNode<move_base_msgs::MoveBaseAction>
{

public:
	NavigateTo(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf) : RosActionNode<move_base_msgs::MoveBaseAction>(handle, name, conf) {}

	static PortsList providedPorts()
	{
		return {
			InputPort<int>("ws"), InputPort<std::string>("base_pose")};
	}
	//base_pose
	bool sendGoal(GoalType &goal) override
	{
		int workstation;
		if (getInput<int>("ws", workstation))
		{
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose = candidate_poses_dictionary[workstation].pose;
		}
		else
		{
			return false;
		}

		ROS_INFO("NavigateTo: Sending request");
		return true;
	}

	NodeStatus onResult(const ResultType &res) override
	{
		ROS_INFO("NavgitateTo: the base moved succesfully ");
		return NodeStatus::SUCCESS;
	}

	virtual NodeStatus onFailedRequest(FailureCause failure) override
	{
		ROS_ERROR("NavigateTo request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}

	void halt() override
	{
		if (status() == NodeStatus::RUNNING)
		{
			ROS_WARN("NavigateTO halted");
			BaseClass::halt();
		}
	}

private:
};

class LookAtTable : public RosServiceNode<kuka_msgs::ArmPose>
{

public:
	LookAtTable(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf) : RosServiceNode<kuka_msgs::ArmPose>(handle, node_name, conf) {}

	static PortsList providedPorts()
	{
		return {
			InputPort<std::string>("pose"),
			OutputPort<std::string>("status")};
	}

	void sendRequest(RequestType &request) override
	{
		getInput("pose", request.pose_id);
		ROS_INFO("LookAtTable: sending request");
	}

	NodeStatus onResponse(const ResponseType &rep) override
	{
		ROS_INFO("LookAtTable: response received");
		if (rep.status == 1)
		{
			setOutput<std::string>("status", "Pose SUCESS");
			return NodeStatus::SUCCESS;
		}
		else
		{
			ROS_ERROR("Look At Table not accomplished :( ");
			setOutput<std::string>("status", "Pose Failed");
			return NodeStatus::FAILURE;
		}
	}

	virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
	{
		ROS_ERROR("Look at table request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}

private:
	int expected_result_;
};

class DetectObj : public RosServiceNode<kuka_msgs::PoseExtraction>
{
private:
	std::string object_name;
	// ros::NodeHandle node_handle;
	// tf2_ros::Buffer tfBuffer;
	float x, y, z;
	// geometry_msgs::TransformStamped world2base_footprint;

public:
	DetectObj(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf) : RosServiceNode<kuka_msgs::PoseExtraction>(handle, node_name, conf)
	{
		// node_handle = handle;
		// tf2_ros::TransformListener tfListener(tfBuffer);
	}
	static PortsList providedPorts()
	{
		return {
			InputPort<std::string>("obj_name"),
			OutputPort<geometry_msgs::PoseStamped>("pose")};
	}

	void sendRequest(RequestType &request) override
	{
		// Here the name of the object should match the name that is given in Gazebo
		//if debuggin or match the string key name form the detection service
		getInput("obj_name", object_name);
		request.object = object_name;
		ROS_INFO_STREAM("Detect Object(" << request.object << "): sending request");
	}

	NodeStatus onResponse(const ResponseType &res) override
	{
		ROS_INFO("DetectObj: response received");
		//HERE: Depending where the pose come from gazebo or objec_detection, the object should be converted in map fram or not
		// Ussually when an object is detected, should be transofrmed to a map frame in order to be consistent with the frame of the candidate mobile poses
		// when the pose of the object comes from Gazebo, it already comes in map frame so no transformation needed.
		if (res.extractionSuccesfull)
		{
			geometry_msgs::PoseStamped object_pose;
			object_pose.header.frame_id = GLOBAL_FRAME;
			object_pose.pose = res.poseStamped.pose;


			if (object_name == "bracket"){
				object_pose.pose.position.z  = 0.8155;
			}
			// Increasing Z position due to imperfections of the object_tracking. 

			x = object_pose.pose.position.x;
			y = object_pose.pose.position.y;
		
			z = object_pose.pose.position.z;
			ROS_INFO("%s detected at x: %f, y: %f, z %f", object_.c_str(),
					 object_pose.pose.position.x,
					 object_pose.pose.position.y,
					 object_pose.pose.position.z);
			
			object_pose.pose.position.z += detection_threshold;

			// The pose is in world coordinates, need to be transofrmed to base_footprint coordinates
			// tf2::Vector3 object_position(x, y, z);
			// ROS_INFO("Before Z: %f", z);
			// getxyzTransform(object_position);

			// object_pose.pose.position.x = object_position.getX();
			// object_pose.pose.position.y = object_position.getY();
			// object_pose.pose.position.z = object_position.getZ();

			// IMPORTANT line of the BT
			setOutput<geometry_msgs::PoseStamped>("pose", object_pose);

			return NodeStatus::SUCCESS;
		}
		else
		{

			return NodeStatus::FAILURE;
		}
	}

	virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
	{
		ROS_ERROR("Detect object request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}
};

class Grasp : public RosServiceNode<kuka_msgs::ObjectPose>
{

public:
	Grasp(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf) : RosServiceNode<kuka_msgs::ObjectPose>(handle, node_name, conf) {}
	static PortsList providedPorts()
	{
		return {
			InputPort<geometry_msgs::PoseStamped>("object_pose")};
	}

	void sendRequest(RequestType &request) override
	{
		getInput<geometry_msgs::PoseStamped>("object_pose", request.obj_pose);
		ROS_INFO("Grasp object : sending request");
	}

	NodeStatus onResponse(const ResponseType &res) override
	{
		ROS_INFO("Grasp object: response received");
		if (res.status)
		{

			return NodeStatus::SUCCESS;
		}
		else
		{

			return NodeStatus::FAILURE;
		}
	}

	virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
	{
		ROS_ERROR("Grasp object request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}
};

class Place : public RosServiceNode<kuka_msgs::Place>
{

private:
	float x, y, z;
	// geometry_msgs::TransformStamped world2base_footprint;

public:
	Place(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf) : RosServiceNode<kuka_msgs::Place>(handle, node_name, conf)
	{
	}
	static PortsList providedPorts()
	{
		return {
			InputPort<int>("bin_number")};
	}

	void sendRequest(RequestType &request) override
	{
		int bin_number_;
		getInput<int>("bin_number", bin_number_);

		// tf2::Vector3 tf2_bin_position;
		// tf2::fromMsg(bins_loc[bin], tf2_bin_position);

		// // The pose is in world coordinates, need to be transofrmed to base_footprint coordinates
		// getxyzTransform(tf2_bin_position);

		request.place_pose.header.frame_id = "base_footprint";
		request.place_pose.pose.position.x = bins_loc[bin_number_].x;
		request.place_pose.pose.position.y = bins_loc[bin_number_].y;
		request.place_pose.pose.position.z = bins_loc[bin_number_].z;

		request.bin = 0;

		ROS_INFO("Place object in bin %i: sending request", bin);
	}

	NodeStatus onResponse(const ResponseType &res) override
	{
		ROS_INFO("Place object: response received");
		if (res.status)
		{
			ROS_INFO("Object placed!");
			return NodeStatus::SUCCESS;
		}
		else
		{
			ROS_INFO("Server filed to place the object");
			return NodeStatus::FAILURE;
		}
	}

	virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
	{
		ROS_ERROR("Grasp object request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "bt_collector");
	ros::NodeHandle nh;

	tf2_ros::TransformListener tfListener(tfBuffer);

	BehaviorTreeFactory factory;

	std::vector<std::string> collection_order_names;
	collection_order_names.resize(4);
	std::map<std::string, float> thresh_map;

	thresh_map.insert(std::pair<std::string, float>("battery_module", 0.016));
	thresh_map.insert(std::pair<std::string, float>("plate_a", 0.016));
	thresh_map.insert(std::pair<std::string, float>("plate_b", 0.0));
	thresh_map.insert(std::pair<std::string, float>("bracket", 0.016));

	if (argc > 1)
	{
		for (int i = 0; i < argc; i++)
		{
			if (std::string(argv[i]) == "-1")
				collection_order_names[0] = std::string(argv[i + 1]);
			else if (std::string(argv[i]) == "-2")
				collection_order_names[1] = std::atof(argv[i + 1]);
			else if (std::string(argv[i]) == "-3")
				collection_order_names[2] = std::string(argv[i + 1]);
			else if (std::string(argv[i]) == "-4")
				collection_order_names[3] = std::string(argv[i + 1]);
			else if (std::string(argv[i]) == "-help")
			{
				std::cout << "Order of collection: -1 [object_name] -2 [object_name] ..." << std::endl;
				return 0;
			}
		}
	}else 
	{
		ROS_WARN("Not enough arguments, aborting ...!");
		return 0;
	}

	float distance_tolerance_1 = 0.04;
	float distance_tolerance_2 = -0.03;


	if (ros::param::get("number_of_poses", number_of_poses))
	{
		for (int i = 1; i <= number_of_poses; i++)
		{
			if (ros::param::get("possible_poses/pose_" + std::to_string(i), pose_vector_param))
			{
				// Extracting candidate poses and adding them to a dictionary
				if (i != 32 && i != 75 && i != 77 && i != 78 && i != 79 && i != 80)
				{
					if (pose_vector_param[0] < 0.4)
					{
						temp_pose.pose.position.x = pose_vector_param[0] - distance_tolerance_1;
					}
					else
					{
						temp_pose.pose.position.x = pose_vector_param[0] + distance_tolerance_2;
					}
				}
				else
				{
					temp_pose.pose.position.x = pose_vector_param[0];
				}

				temp_pose.pose.position.y = pose_vector_param[1];
				temp_pose.pose.position.z = pose_vector_param[2];
				temp_pose.pose.orientation.x = pose_vector_param[3];
				temp_pose.pose.orientation.y = pose_vector_param[4];
				temp_pose.pose.orientation.z = pose_vector_param[5];
				temp_pose.pose.orientation.w = pose_vector_param[6];
				candidate_poses_dictionary.insert(std::pair<int, geometry_msgs::PoseStamped>(i, temp_pose));
			}
			else
			{
				ROS_ERROR("Failed to get workstation %i parameters from server.", i);
			}
		}
		ROS_INFO("All candidate poses captured!");
	}
	else
	{
		ROS_ERROR("Failed to get 'number_of_poses' param from server.");
	}

	// Setting the pose to place the object on a bin respective to "base_footprint" frame
	geometry_msgs::Point p;
	p.x = 0.87;
	p.y = 0.0;
	p.z = 0.83;
	bins_loc.insert(std::pair<int, geometry_msgs::Point>(1, p));
	bins_loc.insert(std::pair<int, geometry_msgs::Point>(2, p));
	bins_loc.insert(std::pair<int, geometry_msgs::Point>(3, p));
	bins_loc.insert(std::pair<int, geometry_msgs::Point>(4, p));

	// ROS Conditions
	RegisterRosService<ObjectReachable>(factory, "ObjectReachable", nh);

	// ROS Actions
	RegisterRosService<BestPose>(factory, "BestPose", nh);
	RegisterRosService<LookAtTable>(factory, "LookAtTable", nh);
	RegisterRosAction<NavigateTo>(factory, "NavigateTo", nh);
	RegisterRosService<DetectObj>(factory, "DetectObj", nh);
	RegisterRosService<Grasp>(factory, "Grasp", nh);
	RegisterRosService<Place>(factory, "Place", nh);

	BT::Tree kuka_tree;
	std::string pkg_path = ros::package::getPath("bt_kuka");

	// Running all the sub-behaviours for every object
	for (int j = 0; j < 4; j++)
	{
		if (!pkg_path.empty())
		{
			ROS_INFO("Looking  BT model at: %s", pkg_path.c_str());
			kuka_tree = factory.createTreeFromFile(pkg_path + "/trees/" + collection_order_names[j] + "_tree.xml");
			// kuka_tree = factory.createTreeFromFile(pkg_path + "/trees/def_2.xml");
			// auto tree = factory.createTreeFromFile("/home/hector/Documents/faraday/bt/definitive/def_2.xml");
		}
		else
		{
			ROS_WARN("Can not find pkg path Aborting...");
			continue;
		}

		detection_threshold = thresh_map.at(collection_order_names[j]);
		NodeStatus status = NodeStatus::IDLE;
		Timer timer;
		timer.start();

		while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
		{
			ros::spinOnce();
			status = kuka_tree.tickRoot();
			std::cout << status << std::endl;
			ros::Duration sleep_time(0.01);
			sleep_time.sleep();
		}

		timer.stop();
		std::cout << collection_order_names[j] << ": Bin-Picking BT last: " << std::endl;
		std::cout << "Seconds: " << timer.elapsedSeconds() << std::endl;
		std::cout << "Minutes " << timer.elapsedSeconds() / 60 << std::endl;
		std::cout << "\n\n";
	}

	return 0;
}
