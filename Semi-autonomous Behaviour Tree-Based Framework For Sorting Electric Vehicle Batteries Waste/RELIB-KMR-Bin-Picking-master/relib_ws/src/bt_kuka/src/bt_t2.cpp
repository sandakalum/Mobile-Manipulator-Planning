#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <behaviortree_ros/AddTwoInts.h>
#include <behaviortree_ros/FibonacciAction.h>
#include <map>
#include <iterator>
#include <vector>
#include <string>
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

using namespace BT;

const std::string GLOBAL_FRAME = "base_footprint";

std::string object_ = "leaf_cell";
int bin = 1;

std::map<int, geometry_msgs::Point> bins_loc;

int number_of_poses = 1;
std::vector<double> pose_vector_param;
std::map<int, geometry_msgs::PoseStamped> candidate_poses_dictionary;
geometry_msgs::PoseStamped temp_pose;

tf2_ros::Buffer tfBuffer;

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

void getxyzTransform(tf2::Vector3 &tf2_vec)
{
	geometry_msgs::TransformStamped world2base_footprint;
	while (ros::ok())
	{
		try
		{
			world2base_footprint = tfBuffer.lookupTransform("base_footprint", "map", ros::Time(0));
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

	void sendRequest(RequestType& request) override
	{
		geometry_msgs::PoseStamped temp_pose;
		getInput<geometry_msgs::PoseStamped>("pose", temp_pose);
		tf2::Vector3 temp_vec;
		temp_vec.setValue(temp_pose.pose.position.x, temp_pose.pose.position.y, temp_pose.pose.position.z);
		transform2Map(temp_vec);
		temp_pose.header.frame_id = "map";
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

public:
	BestPose(ros::NodeHandle &handle, const std::string &node_name, const NodeConfiguration &conf) : RosServiceNode<kuka_msgs::BestWorkstation>(handle, node_name, conf) {}

	static PortsList providedPorts()
	{
		return {
			InputPort<geometry_msgs::PoseStamped>("target_pose"),
			OutputPort<int>("selected_pose")};
		
	}

	void sendRequest(RequestType &request) override
	{
		// request.target_pose.pose.position.x = 0.8;
		// request.target_pose.pose.position.y = -1-0;

		getInput<geometry_msgs::PoseStamped>("target_pose", request.target_pose);
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
		request.object = object_;
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
			object_pose.pose.position.z += 0.015;
			x = object_pose.pose.position.x;
			y = object_pose.pose.position.y;
			z = object_pose.pose.position.z;
			ROS_INFO("%s detected at x: %f, y: %f, z %f", object_.c_str(),
					 object_pose.pose.position.x,
					 object_pose.pose.position.y,
					 object_pose.pose.position.z);

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
		request.place_pose.pose.position.x = bins_loc[bin].x;
		request.place_pose.pose.position.y = bins_loc[bin].y;
		request.place_pose.pose.position.z = bins_loc[bin].z;

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

static const char *xml_test_1 = R"(
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <SetBlackboard name="Init_obj_type" output_key="object_type" value="leaf_cell"/>
            <SetBlackboard output_key="bin" value="1"/>
            <Fallback name="main_fallback">
                <Sequence name="bin_picking">
                    <Action ID="NavigateTo" base_pose="{base_pose}" server_name="move_base" ws="1"/>
                    <Action ID="LookAtTable" pose="eye2" service_name="arm_pose" status="{succes}"/>
                    <Action ID="DetectObj" obj_name="{object_type}" pose="{object_pose}" service_name="object_tracking"/>
                    <Action ID="Grasp" object_pose="{object_pose}" service_name="grasp"/>
                    <Sequence name="place_obj">
                        <Action ID="Place" bin_number="{bin}" service_name="place"/>
                    </Sequence>
                    <Action ID="NavigateTo" base_pose="{base_pose}" server_name="move_base" ws="27"/>
                    <Action ID="LookAtTable" pose="eye" service_name="arm_pose" status="{succes}"/>
                </Sequence>
            </Fallback>
        </Sequence>
    </BehaviorTree>
</root>
)";


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_bt_v2");
	ros::NodeHandle nh;
	BehaviorTreeFactory factory;

	tf2_ros::TransformListener tfListener(tfBuffer);

	if (argv[1] != NULL)
		object_ = std::string(argv[1]);
	if (argv[2] != NULL)
		bin = std::atoi(argv[2]);

	if (ros::param::get("number_of_poses", number_of_poses))
	{
		for (int i = 1; i <= number_of_poses; i++)
		{
			if (ros::param::get("possible_poses/pose_" + std::to_string(i), pose_vector_param))
			{
				// Extracting candidate poses and adding them to a dictionary
				temp_pose.pose.position.x = pose_vector_param[0];
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
	geometry_msgs::Point p;
	geometry_msgs::Point p2;
	p2.x = -0.1;
	p2.y =  0.0;
	p2.z =  0.8;

	p.x = -0.51;
	p.y = -2.25;
	p.z = 0.545 + 0.2;
	bins_loc.insert(std::pair<int, geometry_msgs::Point>(1, p2));
	p.x = -0.71;
	bins_loc.insert(std::pair<int, geometry_msgs::Point>(2, p));
	p.x = -0.91;
	bins_loc.insert(std::pair<int, geometry_msgs::Point>(3, p));
	p.x = -1.11;
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

	auto tree = factory.createTreeFromText(xml_test_1);

	NodeStatus status = NodeStatus::IDLE;

	while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
	{
		ros::spinOnce();
		status = tree.tickRoot();
		std::cout << status << std::endl;
		ros::Duration sleep_time(0.01);
		sleep_time.sleep();
	}

	return 0;
}
