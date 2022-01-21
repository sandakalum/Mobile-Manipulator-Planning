/*C++11*/
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/make_shared.hpp>
#include <ctime>
#include <thread>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/eigen.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/registration/icp.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/moment_of_inertia_estimation.h>

/*OpenGL*/
#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

/*ROS*/
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <erl_rnt/util.hpp>
#include <erl_rnt/status.hpp>
#include <kuka_msgs/PoseExtraction.h>
#include <kuka_msgs/FevorOneExtraction.h>
#include <kuka_msgs/FevorPoseExtraction.h>

#include "fevor/fevor.hpp"

typedef pcl::PointXYZRGB PointType;
typedef pcl::ReferenceFrame RFType;

#define M 100000
#define N 5000

class Fevor{
private:
    ros::ServiceServer recognise_server;

    std::string default_dir = "/home/hector/catkin_ws/src/erl_rnt/data/";
    std::vector<std::string> files;
    bool visualiser = false;

    const char *argv[12];
    float rotation_angle, filtering, matching_threshold;
    float false_removal_thresh;
    int bounding_box_points, min_points_cluster, max_points_cluster;
    float cluster_angle, normal_radius;
public:
    Fevor(ros::NodeHandle *nh){
        recognise_server = nh->advertiseService("/fevor2/detect", &Fevor::recognise_callback, this);
        rotation_angle = 3.0; filtering = 0.00335; matching_threshold = 0.15;
        false_removal_thresh = 0.05;
        bounding_box_points = 50000; min_points_cluster = 10; min_points_cluster = 100;
        cluster_angle = 0.1;
        normal_radius = 0.0375;
    };
    ~Fevor(){

    }
    bool recognise_callback(kuka_msgs::FevorPoseExtraction::Request& request, kuka_msgs::FevorPoseExtraction::Response& recognise_res);

};

//FIXME: given the mean of the cluster relative to the camera frame, transformate it relative to base_footprint coordinates and send result back
bool Fevor::recognise_callback(kuka_msgs::FevorPoseExtraction::Request& request, kuka_msgs::FevorPoseExtraction::Response& recognise_res){
    ROS_INFO_STREAM("Detection started");
    files.push_back("Dummy");
    files.push_back(default_dir + std::string(request.scene_name)   + ".pcd");
    files.push_back(default_dir + std::string(request.object_name)  + ".pcd");

    if (request.rotationAngle !=0 && request.filteringThreshold != 0 && request.matchingThreshold != 0 && request.numberPoints != 0){
        rotation_angle = request.rotationAngle;
        filtering = request.filteringThreshold;
        matching_threshold = request.matchingThreshold;
        false_removal_thresh = request.falseRemovalThreshold;
        bounding_box_points = request.numberPoints;
        min_points_cluster = request.minClusterPoints;
        max_points_cluster = request.maxClusterPoints;
        cluster_angle = request.clusterAngle;
        normal_radius = request.normalRadius;

    }
    visualiser = request.visualise;

    argv[0] = "Dummy";//FEVOR_pose_estimation.exe
    argv[1] = "scene_pcd";//scene
    argv[2] = "object_pcd";//object (to detect in scene)
    argv[3] = "3.0";        //rotation to object (optional ...set default to 3)
    argv[4] = "0.00335"; //filtering (NOTE:always use filter with 0.000895 value for PLATE object) and (for cover set it to 0.0001)
    argv[5] = "0.15";     //Matching threshold1 (change it when require)
    argv[6] = "0.05";     //False removal threshold
    argv[7] = "50000";      //points for setting bounding box(CUBE)
    argv[8] = "10";     //min points in cluster
    argv[9] = "100";    //max points in cluster
    argv[10] = "0.1";       //cluster angle
    argv[11] = "0.0375";    //normal radius




	time_t tstart, tend;
	tstart = time(0);

	std::vector<int> indices;
	std::vector<pcl::PointIndices> cluster_indices;
	int argg = 1;
    // int K = argv[8];
    int K = min_points_cluster;
	float r1_max1 = 0; float r1_max2 = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr befor_clustering_cloud_final1(new pcl::PointCloud<pcl::PointXYZRGB>());////output with keypoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr befor_clustering_cloud_final2(new pcl::PointCloud<pcl::PointXYZRGB>());///output with keypoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr befor_clustering_cloud_final3(new pcl::PointCloud<pcl::PointXYZRGB>());///output with keypoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr befor_clustering_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	pcl::PCDWriter writer;
	//int nbrs_cluster_cloud[100][100];////..........................................................................cloud nbrs information
	// dynamically create array of pointers of size M
	int** nbrs_cluster_cloud = new int* [500];

	// dynamically allocate memory of size N for each row
	for (int i = 0; i < 500; i++)
		nbrs_cluster_cloud[i] = new int[500];

	float*** features = new float** [500];
	for (int i = 0; i < 500; ++i) {
		features[i] = new float* [500];
		for (int j = 0; j < 500; ++j)
			features[i][j] = new float[500];
	}

	float** dis_cen_2_cen = new float* [500];

	// dynamically allocate memory of size N for each row
	for (int i = 0; i < 500; i++)
		dis_cen_2_cen[i] = new float[500];

	//float sq_dis_bwt_nbrs[100][100];
	float** sq_dis_bwt_nbrs = new float* [500];

	// dynamically allocate memory of size N for each row
	for (int i = 0; i < 500; i++)
		sq_dis_bwt_nbrs[i] = new float[500];


	//float centroid[500][3], centroid_original[500][3];//...................................//for all centroid of hull...................//for all.centroid of nearest point of centroid ............

	float** centroid = new float* [500];

	// dynamically allocate memory of size N for each row
	for (int i = 0; i < 500; i++)
		centroid[i] = new float[3];


	// float** centroid_original = new float* [500];

	// // dynamically allocate memory of size N for each row
	// for (int i = 0; i < 500; i++)
	// 	centroid_original[i] = new float[3];
	// //////////////////////////////////////


	for (argg = 1;argg < 3;argg++)
	{
		// pcl::io::loadPCDFile<pcl::PointXYZ>(argv[argg], *cloud_ptr); //////argv[1]
		// pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[argg], *befor_clustering_cloud);
        pcl::io::loadPCDFile<pcl::PointXYZ>(files[argg], *cloud_ptr); //////argv[1]
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(files[argg], *befor_clustering_cloud);
		// float size = atof(argv[4]);
		float size = filtering;

		
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud_ptr);
		sor.setLeafSize(size, size, size);
		sor.filter(*cloud_ptr);
		/*if(argg==1)
		writer.write<pcl::PointXYZ>("3_filtered0.0005.pcd", *cloud_ptr, false);*/

		pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
		sor1.setInputCloud(befor_clustering_cloud);
		sor1.setLeafSize(size, size, size);
		sor1.filter(*befor_clustering_cloud);
		
		if (argg == 2)
		{
			// float theta = atof(argv[3]); // The angle of rotation in radians
			float theta = rotation_angle; // The angle of rotation in radians

		Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
		//Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	//	transform_1.translation() << atof(argv[5]), 0.0, 0.0;
		transform_1.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitX()));
		
		
		pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, transform_1);
		pcl::transformPointCloud(*befor_clustering_cloud, *befor_clustering_cloud, transform_1);

		}


		//std::cout << "Loaded pcd file before sampling " << argv[1] << " with " << cloud_ptr->points.size() << std::endl;
		

		std::cout << "Loaded pcd file after sampling" << files[argg] << " with " << cloud_ptr->points.size() << std::endl;

		// Normal estimation////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		//Eigen::Vector4f p = cloud_normals->points[0].getNormalVector4fMap();
		//Eigen::Vector4f q = cloud_normals->points[11].getNormalVector4fMap();
	 ////   float r1 = p.dot(q);
		////std::cout << "Estimated the normals" <<r1<< std::endl;
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//float rad1 = atof(argv[5]);
		//float rad2 = atof(argv[6]);

		/*int rad1 = atoi(argv[5]);
		int rad2 = atoi(argv[6]);*/
		
		//if (argg == 1)
		//{
		//	ne.setRadiusSearch(rad1); //0.005-->0.05 /////also affect number of cluster
		//   cout << "rad is " << rad1;
		//	//ne.setKSearch(rad1);
		//}
		//else
		//{
		//	ne.setRadiusSearch(rad2);
		//	cout << "rad is " << rad2;
		//	//ne.setKSearch(rad2);
		//}
	//	ne.setKSearch(50); ///dont touch it
	//	ne.compute(*cloud_normals);
	//	std::cout << "Estimated the normals" << std::endl;

		// Creating the kdtree object for the search method of the extraction
		//pcl::KdTree<pcl::PointXYZ>::Ptr tree_ec(new pcl::KdTreeFLANN<pcl::PointXYZ>());
		//tree_ec->setInputCloud(cloud_ptr);cd

		// Extracting Euclidean clusters using cloud and its normals
	
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//const float tolerance = 0.5; // 50cm tolerance in (x, y, z) coordinate system
		//const double eps_angle = 5 * (M_PI / 180.0); // 5degree tolerance in normals
		//const unsigned int min_cluster_size = 400;
		//pcl::extractEuclideanClusters(*cloud_ptr, *cloud_normals, tolerance, tree_ec, cluster_indices, eps_angle, min_cluster_size);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud_ptr);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree_n);
		// ne.setRadiusSearch(atof(argv[11]));
		ne.setRadiusSearch(normal_radius);
		//ne.setKSearch(50);
		ne.compute(*cloud_normals);


		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ec(new pcl::search::KdTree<pcl::PointXYZ>());
		//tree_ec->setInputCloud(cloud_ptr);
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	//	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;

		
		//pcl::IndicesPtr indices1(new std::vector <int>);
		//pcl::PassThrough<pcl::PointXYZRGB> pass1;
		//pass1.setInputCloud(cloud_ptr);
		//pass1.setFilterFieldName("z");
		//pass1.setFilterLimits(0.0, 1);
		//pass1.filter(*indices1);
		//////////////////////////////////////////////////////////////////////////////////////////////////////
		reg.setMinClusterSize(K); //400
		// reg.setMaxClusterSize(atof(argv[9]));
		reg.setMaxClusterSize(max_points_cluster);
		reg.setSearchMethod(tree_ec);
		//reg.setDistanceThreshold(5);
	// reg.setNumberOfNeighbours(500);
		reg.setInputCloud(cloud_ptr);
	//	reg.setIndices (indices1);
		reg.setInputNormals(cloud_normals);
		// reg.setSmoothnessThreshold( atof(argv[10]) / 180.0 * M_PI);
		reg.setSmoothnessThreshold( cluster_angle / 180.0 * M_PI);

	    reg.setCurvatureThreshold(1);
		//reg.setPointColorThreshold(5);/////4
		//reg.setRegionColorThreshold(5); ///1
	//	reg.setMinClusterSize(10);////30
	   // reg.setMaxClusterSize(50);
		reg.extract(cluster_indices);


		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////pcl::search::KdTree<pcl::Normal>::Ptr segtree(new pcl::search::KdTree<pcl::Normal>);
		//std::vector<pcl::PointIndices> cluster_indices;
		//pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		//ec.setClusterTolerance(atof(argv[9])); // 2cm
		//ec.setMinClusterSize(100);
		//ec.setMaxClusterSize(10000);
		//ec.setSearchMethod(tree_ec);
		//ec.setInputCloud(cloud_ptr);
		//ec.extract(cluster_indices);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		std::cout << "No of clusters formed are " << cluster_indices.size() << std::endl;

		// Saving the clusters in separate pcd files ///////////////////////////
		////////////////////////SAVING WITH XYZ//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ> > cloud_cluster; //for all cluster....................................
		std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ> > cloud_hull; ///for all hull...................................................
		
		




		



		int j = 0; ////nbrs around center....................................................................................
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>); ///for temp cluster ..........................................................
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_temp(new pcl::PointCloud<pcl::PointXYZ>); ///for hull temp.................................................

			Eigen::Vector3f rotation_vector;
			//float r;
			for (const int& index : it->indices)
			{
				cloud_temp->points.push_back(cloud_ptr->points[index]);
				// r = pcl::EarClipping::crossProduct (cloud_cluster->points[index], cloud_cluster->points[index]);
				  //std::cout << "points " << cloud_cluster->points[index] << "\n";
			}
			cloud_temp->width = static_cast<uint32_t> (cloud_temp->points.size());
			cloud_temp->height = 1;
			cloud_temp->is_dense = true;

			cloud_cluster.push_back(*cloud_temp);

			//.........................................COMPUTE CONVEX HULL OF EVERY CLUSTER.............................................
			pcl::ConvexHull<pcl::PointXYZ> chull;
			//pcl::ConcaveHull<pcl::PointXYZ> chull;
			chull.setInputCloud(cloud_temp);
			//chull.setAlpha(0.1);
			chull.reconstruct(*cloud_hull_temp);
		//	std::cerr << "Convex hull has: " << cloud_hull_temp->points.size() << " data points." << std::endl;
			//if (cloud_hull_temp->points.size() < 5) //////avoiding error
				//continue;

			cloud_hull.push_back(*cloud_hull_temp);
			//................................CENTROID OF EVERY HULL......................................................
			//Eigen::Vector2f Eigen::Vector4f  centroid(cluster_indices.size(), Eigen::Vector4f(3));//////for all centroid of cluster.............................................
			Eigen::Vector4f centroid_temp; ////for temp centroid .................................................................................................................
			pcl::compute3DCentroid(*cloud_hull_temp, centroid_temp);
			//std::cerr << "\n center of Convex hull is: " << centroid_temp << " \n" << std::endl;
			for (int i = 0;i < 3;i++)
				centroid[j][i] = centroid_temp[i]; //for all centroid.................................................................................................................................
			//...................................................................................................................................................................................
			pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
			tree_->setInputCloud(cloud_temp);

			std::vector<int> nn_indices(1);
			std::vector<float> nn_dists(1);

			tree_->nearestKSearch(pcl::PointXYZ(centroid_temp[0], centroid_temp[1], centroid_temp[2]), 1, nn_indices, nn_dists);


			//cout << "\nThe closest point is:" << cloud_temp->points[nn_indices[0]].x << cloud_temp->points[nn_indices[0]].y << cloud_temp->points[nn_indices[0]].z;
			/////////////////////////k nearest neigbors of centroid of original cloud (befor_clustering_cloud)/////////////////////////////////////// 

			pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
			kdtree.setInputCloud(befor_clustering_cloud); //////original cloud
			pcl::PointXYZRGB searchPoint;

			searchPoint.x = cloud_temp->points[nn_indices[0]].x;
			searchPoint.y = cloud_temp->points[nn_indices[0]].y;
			searchPoint.z = cloud_temp->points[nn_indices[0]].z;

			// K nearest neighbor search



			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);

			/*std::cout << "K nearest neighbor search at (" << searchPoint.x
				<< " " << searchPoint.y
				<< " " << searchPoint.z
				<< ") with K=" << K << std::endl;*/



			if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
				{

					nbrs_cluster_cloud[j][i] = pointIdxNKNSearch[i];
					//nbrs_cluster_cloud[j][i][1] = befor_clustering_cloud->points[pointIdxNKNSearch[i]].y;
					//nbrs_cluster_cloud[j][i][2] = befor_clustering_cloud->points[pointIdxNKNSearch[i]].z;
					sq_dis_bwt_nbrs[j][i] = pointNKNSquaredDistance[i];
					/*std::cout << "    " << befor_clustering_cloud->points[nbrs_cluster_cloud[j][i]].x
						<< " " << befor_clustering_cloud->points[nbrs_cluster_cloud[j][i]].y
						<< " " << befor_clustering_cloud->points[nbrs_cluster_cloud[j][i]].z
						<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
						//intensity_cen = (int(befor_clustering_cloud->points[nn_indices[0]].r) + int(befor_clustering_cloud->points[nn_indices[0]].g) + int(befor_clustering_cloud->points[nn_indices[0]].b)) / 3;
						//intensity_nbrs = (int(befor_clustering_cloud->points[pointIdxNKNSearch[i]].r) + int(befor_clustering_cloud->points[pointIdxNKNSearch[i]].g) + int(befor_clustering_cloud->points[pointIdxNKNSearch[i]].b)) / 3;
						//if (abs(intensity_cen - intensity_nbrs) < 5) ////////// color_cloud is main cloud.........color_cloud->points[nn_indices[0]] is centroid intensity of cluster 1("cloud")
							//freq_count++;
				}
				//cout << "freq of centroid is" << freq_count << "\n";
			}


			j++;
		} ////loop of cluster.....................................................................................................................................................................
		/*for (int i = 0; i < cluster_indices.size(); i++)
		{
			std::stringstream ss;
			ss << "./cloud_cluster_" << i << ".pcd";
			writer.write<pcl::PointXYZ>(ss.str(), cloud_cluster[i], false);
		}*/
		///////////////////////////////////////////////////////////////////////////////////////////////
		//cloud_cluster[], cloud_hull[], centroid[][] ,nbrs_cluster_cloud[j][i][0] .....j for cluster, i for nbrs, [0] for x-axis

		///////////////////////////////FEATURE DETECTION IN EVERY CLOUD_CLUSTER[i] ////////////////////////////////////////////////////////////////////////
		int freq_count = 1; float intensity_cen = 0, intensity_nbrs = 0;
		/*float features[100][100][100];
		float dis_cen_2_cen[100][100];*/

		//float features[50][50][50];
		//float dis_cen_2_cen[50][50];

		

	


		///////////////////////////////	////////////////////////////////distnace between center to center//////////////////////////////////////////////////////////
		for (int i1 = 0; i1 < cluster_indices.size(); i1++)
		{
			for (int i2 = 0; i2 < cluster_indices.size(); i2++)
			{
				float a, b, c;
				a = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].x), 2);
				b = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].y), 2);
				c = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].z), 2);
				dis_cen_2_cen[i1][i2] = sqrt(a + b + c)+0.000001;
				
			}
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	MAX MIN of angle b/w two point
		//float r1_max = 0;
		//for (int i1 = 0; i1 < cluster_indices.size(); i1++)
		//{
		//	for (int j1 = 0; j1 < K; j1++)
		//	{
		//		for (int i2 = 0; i2 < cluster_indices.size();i2++)
		//		{
		//			if (dis_cen_2_cen[i1][i2] < 0.10) //0.01 // 100
		//			{

		//				for (int j2 = 0; j2 < K; j2++)
		//				{
		//					Eigen::Vector4f p = cloud_normals->points[nbrs_cluster_cloud[i1][j1]].getNormalVector4fMap();
		//					Eigen::Vector4f q = cloud_normals->points[nbrs_cluster_cloud[i2][j2]].getNormalVector4fMap();
		//					//	cout << "\nppppppis" << p[2];
		//					float n1 = p.norm() * q.norm();
		//					float r1 = p.dot(q);
		//					r1 = acos(r1 / n1);//*180 / M_PI;
		//					if (r1 > r1_max)
		//						r1_max = r1;
		//					
		//				}
		//			}
		//		}
		//	}
		//}
		
		/*if (argg == 1)
			r1_max1 = r1_max;
		else
			r1_max2 = r1_max;
		cout << "\nr1_max1 is" << r1_max1;
		cout << "\nr1_max2 is" << r1_max2;*/
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
		ofstream myfile, myfile1;
		std::stringstream ss;
		ss << argg << "feature.txt"; // `i` is automatically converted
		myfile.open(ss.str());
		//myfile.open(std::to_string(argg) + ".txt");
		myfile1.open("loc_array.txt");
		float dis_cen_2_point, dis_samecen_2_point;
		int sum_nbrs_int = 0;
		Eigen::Vector4f cloud1_axis, cloud2_axis;
		float r, n, angle;
		int last = 0;
		for (int i1 = 0; i1 < cluster_indices.size(); i1++)
		{
			for (int j1 = 0; j1 < K; j1++)
			{
				//std::cout << j1 << "..................... feature extraction for \n" << befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x;
				//intensity_cen = (int(befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].r) + int(befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].g) + int(befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].b)) / 3;
				/*cloud1_axis[0] = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x;
				cloud1_axis[1] = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y;
				cloud1_axis[2] = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z;*/
				//std::cout << "\ncenter intensity" << intensity_cen;
				for (int i2 = 0; i2 < cluster_indices.size();i2++)
				{
					//cout << "\n cen to cen distance" << dis_cen_2_cen[i1][i2] << "\n";
					
					if (dis_cen_2_cen[i1][i2] < 0.1) //0.01 // 100 //0.05 ///0.03
					{


						float dis_matched_point = 0, a2, b2, c2, a3 = 0, b3 = 0, c3 = 0, a4,b4,c4;
						float dis_matched_point1 = 0; 
						float dis_matched_point2 = 0;
						for (int j2 = 0; j2 < K; j2++)
						{
							//intensity_nbrs = (int(befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].r) + int(befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].g) + int(befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].b)) / 3;
						
							/*cloud2_axis[0] = befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].x;
							cloud2_axis[1] = befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].y;
							cloud2_axis[2] = befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].z;
*/
							Eigen::Vector4f p = cloud_normals->points[nbrs_cluster_cloud[i1][j1]].getNormalVector4fMap();
							Eigen::Vector4f q = cloud_normals->points[nbrs_cluster_cloud[i2][j2]].getNormalVector4fMap();
						//	cout << "\nppppppis" << p[2];
							float n1 = p.norm() * q.norm();
							float r11 = p.dot(q);
							float r1 = acos(r11 / n1);//*180 / M_PI;
						//	r1 = r1 * 180 / M_PI;
							//std::cout << "Estimated the normals" <<r1<< std::endl;
							
							
							//if (abs(intensity_cen - intensity_nbrs) < 5) ////////// color_cloud is main cloud.........color_cloud->points[nn_indices[0]] is centroid intensity of cluster 1("cloud")
							//{
						//if (r1 > (r1_max * 0.40))
							/*Eigen::Vector4f p1 = cloud_normals->points[nbrs_cluster_cloud[i1][0]].getNormalVector4fMap();
							Eigen::Vector4f q1 = cloud_normals->points[nbrs_cluster_cloud[i2][0]].getNormalVector4fMap();
							float n2 = p1.norm() * q1.norm();
							float r2 = p1.dot(q1);*/
							Eigen::Vector4f p1 = cloud_normals->points[nbrs_cluster_cloud[i1][0]].getNormalVector4fMap();
							Eigen::Vector4f q1 = cloud_normals->points[nbrs_cluster_cloud[i1][j1]].getNormalVector4fMap();
							Eigen::Vector4f p2 = cloud_normals->points[nbrs_cluster_cloud[i2][0]].getNormalVector4fMap();
							Eigen::Vector4f q2 = cloud_normals->points[nbrs_cluster_cloud[i2][j2]].getNormalVector4fMap();
							float n2 = p1.norm() * q1.norm();
							float n3 = p2.norm() * q2.norm();
						//	r2 = acos(r2 / n2);//*180 / M_PI;
							if (r1 > 0 & r1 < 3)
								{
									//r1 = r1 + 0.0000001;
								//cout << "\n" << "r1 is" << r1;
								
								//if (freq_count < 2)
								{
									a4 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].x), 2);
									b4 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].y), 2);
									c4 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].z), 2); 

									a3 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].x), 2);
									b3 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].y), 2);
									c3 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].z), 2);

									a2 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x), 2);
									b2 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y), 2);
									c2 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z), 2);
									float zx1 = befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].z;
									float zx2 = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z;
									float xx1 = befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].x;
									float xx2 = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x;
									float yx1 = befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].y;
									float yx2 = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y;
								//	
									
										//cout << "\np2222 is" << p[2] << "  q222 is" << q[2];
									
									//	dis_matched_point1 = abs( log(sqrt(a2 + b2 + c2) + sqrt(a2 + b2 + c2)  +0.000000001 ) * log(1 / r1) * (zx1+zx2) )   ;// (zx1+zx2)* (zx1+zx2) *  );//*log(sqrt(a3 + b3 + c3) + 0.00000001)* log(sqrt(a4 + b4 + c4) + 0.0000001)  );//*(intensity_nbrs* intensity_cen));  
									//	dis_matched_point2 = 1;//(zx1 + zx2);//((zx1 + zx2));
									//dis_matched_point = dis_matched_point + (dis_matched_point1 * dis_matched_point2);

dis_matched_point = dis_matched_point + log( abs(  log( (  n1*(sqrt(a2 + b2 + c2)  )) + 0.00000001)*  log(  ( (1/(r1) )*  (n3*sqrt(a3 + b3 + c3) + n2*sqrt(a4 + b4 + c4)) ) + 0.00000001)    ) );
//dis_matched_point = dis_matched_point + abs( ( log( (n1 * sqrt(a2 + b2 + c2)) +0.000000001) + log( (sqrt(a2 + b2 + c2)*n1) + 0.000000001) ) * log(1 / (r1 * 1)) *
                                                                                  //                      ( log( (n3 * sqrt(a3 + b3 + c3))+ 0.000000001) +  log( (n2 * sqrt(a4 + b4 + c4))+ 0.000000001)  ) );
							     //cout << "\n result is" << dis_matched_point;
								 //    dis_matched_point = pow(dis_matched_point, (1 - angle / 180));
								 //intensity_cen = intensity_cen + intensity_nbrs;
							    }
								//else
								//{
								//	a3 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2-1]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].x), 2);
								//	b3 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2-1]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].y), 2);
								//	c3 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2-1]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i2][j2]].z), 2);
								////	cout << "\ndis2 is" << sqrt(a3 + b3 + c3);cd
								//	//if ((sqrt(a3 + b3 + c3)) > 0)
								//	dis_matched_point = dis_matched_point + abs(log(sqrt(a3 + b3 + c3) + 0.0000001)  );//*(intensity_nbrs * intensity_cen)); //*log(r);
								//	//dis_matched_point = pow(dis_matched_point, (1 - angle / 180));
								//	//intensity_cen = intensity_cen + intensity_nbrs;
								//}

									freq_count++;
								//std::cout << "Angle to rotate: " << angle << std::endl;
								//std::cout << "cen intensity: " << intensity_cen << std::endl;
						      } 
								
							//last = j2;
						}
						//	cout << "\n dis_matched_point" << dis_matched_point << "\n";
							/*std::cout << "\n cent int is " << intensity_cen << "";
							std::cout << "\n freq of int is " << freq_count << "";
							std::cout << "\n dis is " << dis_cen_2_cen[i1][i2] << "";*/
							//add one more feature ...distance of intensity_cen to every center..........................................................
						/*float a, b, c;
						float a1, b1, c1;
						a = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x), 2);
						b = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y), 2);
						c = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][0]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z), 2);
						dis_cen_2_point = sqrt(a + b + c) + 0.000001;

						a1 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x), 2);
						b1 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y), 2);
						c1 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i1][0]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z), 2);
						dis_samecen_2_point = sqrt(a1 + b1 + c1) + 0.000001;
*/
						
						//if(intensity_cen>10)
						//features[i1][j1][i2] = abs((intensity_cen * freq_count) * log(dis_cen_2_point + dis_samecen_2_point));// *log(dis_cen_2_cen[i1][i2])); //+ dis_cen_2_cen[i1][i2]
					//	cout << "\nfreq is" << freq_count;
						///*a4 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][last]].x - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x), 2);
						//b4 = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][last]].y - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y), 2);
						//c4*/ = pow((befor_clustering_cloud->points[nbrs_cluster_cloud[i2][last]].z - befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z), 2);
						if (freq_count > 1)
							//features[i1][j1][i2] = abs(   (1)* freq_count * (dis_matched_point / (1 * 1))+ abs(log(sqrt(a4 + b4 + c4) + 0.0000001))   );//+dis_cen_2_cen[i1][i2])); //+ dis_cen_2_cen[i1][i2]
						    features[i1][j1][i2] = abs( freq_count  * (dis_matched_point) ) ;//+dis_cen_2_cen[i1][i2])); //+ dis_cen_2_cen[i1][i2]
						else

							features[i1][j1][i2] = 0;




						//else
						//	features[i1][j1][i2] = 0.1* abs((intensity_cen * freq_count) * log(dis_cen_2_point + dis_samecen_2_point)); //+ dis_cen_2_cen[i1][i2]

						freq_count = 1;
						



					}
					else ///////center distance else ////////if (dis_cen_2_cen[i1][i2] < 1) //0.1
					{
						features[i1][j1][i2] = 0;
					}
					
					myfile << features[i1][j1][i2] << " ";
				}
				myfile << "\n";



			}

		}
		myfile.close();

		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////SHOWING FEATURES//////////////////////////////////////////////////////////////////
		float loc_x, loc_y, loc_z;
		for (int i1 = 0; i1 < cluster_indices.size(); i1++)
		{
			for (int j1 = 0; j1 < K; j1++)
			{
				//for (int i2 = 0; i2 < cluster_indices.size(); i2++)
				//{
				//	//std::cout << "\n feature is" << features[i1][j1][i2] << "";
				//}
				//cout << "\n";

				loc_x = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].x;
				loc_y = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].y;
				loc_z = befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].z;


				myfile1 << loc_x << " " << loc_y << " " << loc_z << " ";
			/*	befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].r = 255;
				befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].g = 255;
				befor_clustering_cloud->points[nbrs_cluster_cloud[i1][j1]].b = 255;*/

			}
			myfile1 << "\n";
		}
		myfile1.close();



	
	



		if(argg==1)
		copyPointCloud(*befor_clustering_cloud, *befor_clustering_cloud_final1);
		else
			copyPointCloud(*befor_clustering_cloud, *befor_clustering_cloud_final2);

		
		
	}
	///////////////////////////////////MATCHING OF FEATURES///////////////////////////////////////////////////////////////////////
	//float loc_x1[100], loc_y1[100], loc_z1[100];
	//float loc_x2[100], loc_y2[100], loc_z2[100];



		std::ifstream infile1("1feature.txt");
		std::ifstream infile2("2feature.txt");
		std::string line1;
		std::string line2;
		// dynamically create array of pointers of size M
		float** data1 = new float* [M];

		// dynamically allocate memory of size N for each row
		for (int i = 0; i < M; i++)
			data1[i] = new float[N];

		// dynamically create array of pointers of size M
		float** data2 = new float* [M];

		// dynamically allocate memory of size N for each row
		for (int i = 0; i < M; i++)
			data2[i] = new float[N];
		std::string temp1;std::string temp2;
		float found1;
		float found2;
		int j1;		int j2;
		int i1 = 0, i2 = 0;
		
		while (std::getline(infile1, line1))
		{
			
			std::stringstream ss;
			ss << line1;

			j1 = 0;
			while (!ss.eof())
			{

				/* extracting word by word from stream */
				ss >> temp1;
				if (std::stringstream(temp1) >> found1)
					data1[i1][j1] = found1;

				//std::cout << data1[i][j1] <<" ";
				j1++;
				
			}
			
			//std::cout <<"\n";
			i1++;
		//	cout << "\nhere...." << i1 << " " << j1;

		}
		
		while (std::getline(infile2, line2))
		{
			
			std::stringstream ss;
			ss << line2;

			j2 = 0;
			while (!ss.eof())
			{

				/* extracting word by word from stream */
				ss >> temp2;
				if (std::stringstream(temp2) >> found2)
					data2[i2][j2] = found2;

				//std::cout << data2[i2][j2] <<" ";
				j2++;
			}
			//std::cout << "\n";
			i2++;

		}
		

		pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer1->setBackgroundColor(255, 255, 255);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(befor_clustering_cloud_final1);
		
		Eigen::Vector4f cloud1_axis, cloud2_axis;
		float r, n, angle;
		int feature = 0;
		float final_feature = 0;
		Eigen::Vector4f trans1, trans2, cent1, cent2;
		float mean_x1 = 0, mean_y1 = 0, mean_z1 = 0, mean_x2 = 0, mean_y2 = 0, mean_z2 = 0;
		float* loc_x1 = new float[M];
		float* loc_y1 = new float[M];
		float* loc_z1 = new float[M];
		float* loc_x2 = new float[M];
		float* loc_y2 = new float[M];
		float* loc_z2 = new float[M];
		 int cl_num = 0, counter1 = 0, counter2 = 0;

		 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
		 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
		 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>());
		 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final_out(new pcl::PointCloud<pcl::PointXYZRGB>());


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pose_cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	pose_cloud_in->width = 1500000;
		/*pose_cloud_in->width = 50000;
		pose_cloud_in->height = 1;
		pose_cloud_in->is_dense = false;
		pose_cloud_in->resize(pose_cloud_in->width* pose_cloud_in->height);*/
        pose_cloud_in->resize(befor_clustering_cloud_final2->width* befor_clustering_cloud_final2->height);

		/*cloud_in->width = 50000;
		cloud_in->height = 1;
		cloud_in->is_dense = false;
		cloud_in->resize(cloud_in->width* cloud_in->height);*/
		cloud_in->resize(befor_clustering_cloud_final2->width* befor_clustering_cloud_final2->height);

		/*cloud_in1->width = 200000;
		cloud_in1->height = 1;
		cloud_in1->is_dense = false;
		cloud_in1->resize(cloud_in1->width* cloud_in1->height);*/
		cloud_in1->resize(befor_clustering_cloud_final2->width* befor_clustering_cloud_final2->height);

		/*cloud_out->width = 50000;
		cloud_out->height = 1;
		cloud_out->is_dense = false;
		cloud_out->resize(cloud_out->width* cloud_out->height);*/
		cloud_out->resize(befor_clustering_cloud_final2->width* befor_clustering_cloud_final2->height);

		/*cloud_out1->width = 200000;
		cloud_out1->height = 1;
		cloud_out1->is_dense = false;
		cloud_out1->resize(cloud_out1->width* cloud_out1->height);*/
		cloud_out1->resize(befor_clustering_cloud_final2->width* befor_clustering_cloud_final2->height);

		cloud_final_out->width =10000;
		cloud_final_out->height = 1;
		cloud_final_out->is_dense = false;
		cloud_final_out->resize(cloud_final_out->width* cloud_final_out->height);

		int K1 = 1; /////nbrs size around keypoint


		
		//float threshold2 = 1;
		cout << "\n j2 isssssss" << floor(j2);
		//float threshold2 = j2* atof(argv[10]); //////without intensity  j2 * 0.5 /////////with intensity  j2 * 0.3
		float threshold2 = 0;
		float threshold1 = 0.00005;///0.0000005

		pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_false1;
		kdtree_false1.setInputCloud(befor_clustering_cloud_final1); //////original cloud
		std::vector<int> pointIdxNKNSearch_false1;
		std::vector<float> pointNKNSquaredDistance_false1;

		pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_false2;
		kdtree_false2.setInputCloud(befor_clustering_cloud_final2); //////original cloud
		std::vector<int> pointIdxNKNSearch_false2;
		std::vector<float> pointNKNSquaredDistance_false2;
		float size_false1[50000];
		float size_false2[50000];
		float size_false11[50000];
		float size_false12[50000];
		int sizee = 0;
		for (int i = 0;i < i1;i++)
		{ 
			

				for (int k = 0;k < i2;k++)
				{
					
						feature = 0;
						for (int j = 0; j < j1 - 1;j++)
						{

							for (int l = 0; l < j2 - 1;l++)
							{
							
								if (data1[i][j] > 0)
								{
									if (abs(data1[i][j] - data2[k][l]) < threshold1)
										feature++;

								}
								else
								{
									break;
								}
							}

						}
					//cout << "\nbbbbbbbbbfeature is" << feature;
						if (feature > floor(threshold2))// floor(threshold2))
						{
							// cout << "\nfeature is" << feature;
							final_feature++;


							//befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].r = 255; ////taking location of metched keypoints // floor(i/40)
							//befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].g = 255;
							//befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].b = 0;


							//befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].r = 255; ////taking location of metched keypoints
							//befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].g = 255;
							//befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].b = 0;
							//cout << "\n here";
							if (isnan(befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].x) == 1)
								break;

							if (isnan(befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].x) == 1)
								break;

							
						//	cout << cl_num <<"here\n";

						//	cout << "\n" << isnan(loc_x1[cl_num]);
							

							
						/*	mean_x1 = loc_x1[cl_num] + mean_x1;
							mean_y1 = loc_y1[cl_num] + mean_y1;
							mean_z1 = loc_z1[cl_num] + mean_z1;

							mean_x2 = loc_x2[cl_num] + mean_x2;
							mean_y2 = loc_y2[cl_num] + mean_y2;
							mean_z2 = loc_z2[cl_num] + mean_z2;*/

													
							/*trans_x = loc_x1[cl_num] - loc_x2[cl_num];
							trans_y = loc_y1[cl_num] - loc_y2[cl_num];
							trans_z = loc_z1[cl_num] - loc_z2[cl_num];*/
							

							//cout << "\n" << cloud_in->points[cl_num].x  << "" << cloud_in->points[cl_num].y << "" << cloud_in->points[cl_num].z;
							/*cout << "\n " << "i is" << i << "" << "j is" << k;*/
							/////////////take out false points...........................................................................................................................................
							

							/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr false_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>());
							false_cloud1->width = 200;
							false_cloud1->height = 1;
							false_cloud1->is_dense = false;
							false_cloud1->resize(false_cloud1->width* false_cloud1->height);
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr false_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());
							false_cloud2->width = 200;
							false_cloud2->height = 1;
							false_cloud2->is_dense = false;
							false_cloud2->resize(false_cloud2->width* false_cloud2->height);
*/
			     			pcl::PointXYZRGB searchPoint_false1;
							searchPoint_false1.x = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].x;
							searchPoint_false1.y = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].y;
							searchPoint_false1.z = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].z;

						
							float dist_false1 = 0; float mean_dist_false1 = 0; float variance1 = 0;
							if (kdtree_false1.nearestKSearch(searchPoint_false1, 10000, pointIdxNKNSearch_false1, pointNKNSquaredDistance_false1) > 0)
							{
								//cout << "\n points around detected point in scene" << pointNKNSquaredDistance_false1[1];

								for (int i = 0; i < pointIdxNKNSearch_false1.size(); ++i)
								{
									dist_false1 = dist_false1 + (pointNKNSquaredDistance_false1[i]); /////dist of neigb. from detected point in scene
									
								}
									//mean_dist_false1 = dist_false1 / pointIdxNKNSearch_false1.size();
									/*for (int i = 0; i < pointIdxNKNSearch_false1.size(); ++i)
									{
										variance1 += pow(pointNKNSquaredDistance_false1[i] - mean_dist_false1, 2);

									}
									variance1 = variance1 / pointIdxNKNSearch_false1.size();*/
									//variance1 = sqrt(variance1);
							}


							
							pcl::PointXYZRGB searchPoint_false2;
							searchPoint_false2.x = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].x;
							searchPoint_false2.y = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].y;
							searchPoint_false2.z = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].z;

						
							float dist_false2 = 0; float mean_dist_false2 = 0; float variance2 = 0;
							if (kdtree_false2.nearestKSearch(searchPoint_false2, 10000, pointIdxNKNSearch_false2, pointNKNSquaredDistance_false2) > 0)
							{
								//cout << "\n points around detected point in object" << pointIdxNKNSearch_false2.size();
								for (int i = 0; i < pointIdxNKNSearch_false2.size(); ++i)
								{
									
									dist_false2 = dist_false2 + (pointNKNSquaredDistance_false2[i]);  /////dist of neigb. from detected point in object
									
								}
								//mean_dist_false2 = dist_false2 / pointIdxNKNSearch_false2.size();
								/*for (int i = 0; i < pointIdxNKNSearch_false2.size(); ++i)
								{
									variance2 += pow(pointNKNSquaredDistance_false2[i] - mean_dist_false2, 2);

								}
								variance2 = variance2 / pointIdxNKNSearch_false2.size();*/
							//	variance2 = sqrt(variance2);
							}

							
							/*size_false11[cl_num] = variance1;
							size_false12[cl_num] = variance2;*/
							size_false1[cl_num] = dist_false1 / pointIdxNKNSearch_false1.size();
							size_false2[cl_num] = dist_false2 / pointIdxNKNSearch_false2.size();

							//   cout << "\n dis around detected point in scene" << abs(size_false1[cl_num]- size_false2[cl_num]);
							    //cout << "\n dis around detected point in object" << size_false2[cl_num];

							//	cout << "\n variance of detected point in scene" << size_false11[cl_num];
							//	cout << "\n variance of detected point in object" << size_false12[cl_num];

						// if ( abs(size_false1[cl_num] - size_false2[cl_num]) < atof(argv[6]) ) //10 //0.0001
						if ( abs(size_false1[cl_num] - size_false2[cl_num]) < false_removal_thresh ) //10 //0.0001

								//if((size_false11[cl_num] - size_false12[cl_num])< 5)
							    {
								
								//cout << "inside";
								loc_x1[cl_num] = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].x; ////taking location of metched keypoints
								loc_y1[cl_num] = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].y;
								loc_z1[cl_num] = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].z;

								loc_x2[cl_num] = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].x; ////taking location of metched keypoints
								loc_y2[cl_num] = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].y;
								loc_z2[cl_num] = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].z;

								cloud_final_out->points[cl_num].x = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].x;
								cloud_final_out->points[cl_num].y = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].y;
								cloud_final_out->points[cl_num].z = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].z;
								cloud_final_out->points[cl_num].r = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].r;
								cloud_final_out->points[cl_num].g = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].g;
								cloud_final_out->points[cl_num].b = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].b;
								//std::string number(boost::lexical_cast<std::string>(cl_num));
	
								sizee++;
							    }
							cl_num++;
						
					break;
						}

					

				}

			

			
		}
		float mean_cloud_in_x = 0; float mean_cloud_in_y = 0; float mean_cloud_in_z = 0;
		cout << "aaaaaaa" << sizee;
		int count_metched=0;
		int temp_feature;
		float temp_final_feature;
		int m = 0;
		for (int i = 0;i < i1;i++)
		{


			for (int k = 0;k < i2;k++)
			{

				temp_feature = 0;
				for (int j = 0; j < j1 - 1;j++)
				{

					for (int l = 0; l < j2 - 1;l++)
					{

						if (data1[i][j] > 0)
						{
							if (abs(data1[i][j] - data2[k][l]) < threshold1)
								temp_feature++;
						}
						else
						{
							break;
						}
					}

				}

				if (temp_feature > floor(threshold2))// floor(threshold2))  
				{


					if (isnan(befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].x) == 1)
						break;

					if (isnan(befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].x) == 1)
						break;

					//////////////////////////////////////////LOCATE DETECTED OBJECT//////////////////////////////////////////////////////////////////////////////////////////////////
					
						int detected_points = 0;
						pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree3;
						kdtree3.setInputCloud(cloud_final_out); //////original cloud
						pcl::PointXYZRGB searchPoint3;
						searchPoint3.x = cloud_final_out->points[m].x;
						searchPoint3.y = cloud_final_out->points[m].y;
						searchPoint3.z = cloud_final_out->points[m].z;
						
						std::vector<int> pointIdxNKNSearch3;
						std::vector<float> pointNKNSquaredDistance3;
						float argg11;
						// if ( abs(size_false1[m] - size_false2[m]) < atof(argv[6]) )///0.00001
						if ( abs(size_false1[m] - size_false2[m]) < false_removal_thresh )///0.00001

					   // if ((size_false11[m] - size_false12[m]) < 5)
						{
							
							if (kdtree3.radiusSearch(searchPoint3, 0.05 , pointIdxNKNSearch3, pointNKNSquaredDistance3) > 0)
							{
								//cout << "\nobject detected" << pointIdxNKNSearch3.size() << " " << final_feature << " " << final_feature * 0.01;
							// if ((pointIdxNKNSearch3.size() > (final_feature * atof(argv[5]) )) & (pointIdxNKNSearch3.size() < final_feature) )
							if ((pointIdxNKNSearch3.size() > (final_feature * matching_threshold )) & (pointIdxNKNSearch3.size() < final_feature) )

								//cout << "sizee is" << sizee;
								/*if (sizee < 50)
									 argg11 = 0.015;
								else
									 argg11 = 0.065;

									if ((pointIdxNKNSearch3.size() > (final_feature * argg11))& (pointIdxNKNSearch3.size() < final_feature))*/
								    {

								//	cout << "\nobject detected" << pointIdxNKNSearch3.size() << " " << final_feature <<" " << final_feature  * atof(argv[11]) <<"count "<< count_metched;
									//if (count_metched > 100)
									//	break;
								//	cout << "\nobject detected" << count_metched;
									count_metched++;
									/*for (int b = 0; b < pointIdxNKNSearch3.size(); ++b)
									{
											cloud_final_out->points[pointIdxNKNSearch3[b]].r = 255;
											cloud_final_out->points[pointIdxNKNSearch3[b]].g = 0;
											cloud_final_out->points[pointIdxNKNSearch3[b]].b = 0;



									}*/
									befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].r = 255;
									befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].g = 0;
									befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].b = 0;
									std::string number(boost::lexical_cast<std::string>(m));
									//		pcl::PointXYZ(befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].x, befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].y, befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].z), "line_" + number, 0);

										////////////////////add 20 points around keypoint/////////////////////////////////////////////////////////////////
									cloud_in->points[counter1].x = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].x;
									cloud_in->points[counter1].y = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].y;
									cloud_in->points[counter1].z = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].z;
								//	cloud_in->points[counter1].r = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].r;
									//cloud_in->points[counter1].g = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].g;
									//cloud_in->points[counter1].b = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].b;

									/*pose_cloud_in->points[counter1].x = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].x;
									pose_cloud_in->points[counter1].y = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].y;
									pose_cloud_in->points[counter1].z = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].z;
									pose_cloud_in->points[counter1].r = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].r;
									pose_cloud_in->points[counter1].g = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].g;
									pose_cloud_in->points[counter1].b = befor_clustering_cloud_final1->points[nbrs_cluster_cloud[int(i / K)][int(i % K)]].b;*/

									pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree1;

									kdtree1.setInputCloud(befor_clustering_cloud_final1); //////original cloud
									pcl::PointXYZRGB searchPoint1;

									searchPoint1.x = cloud_in->points[counter1].x;
									searchPoint1.y = cloud_in->points[counter1].y;
									searchPoint1.z = cloud_in->points[counter1].z;

									std::vector<int> pointIdxNKNSearch1(K1);
									std::vector<float> pointNKNSquaredDistance1(K1);
									/*
									cout << "\n jjjjj" << counter1;
									cout << "\n jjjjj" << pointIdxNKNSearch1.size();
									cout << "\n jjjjj" << pointNKNSquaredDistance1.size();
									cout << "\njjjj"<< searchPoint1;*/
									if (kdtree1.nearestKSearch(searchPoint1, K1, pointIdxNKNSearch1, pointNKNSquaredDistance1) > 0)
									{

									//	cout << "\nm is" << count_metched;

										for (int i = 0; i < pointIdxNKNSearch1.size(); i=i+1)
										{

											counter1++;
											cloud_in->points[counter1].x = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].x;
											cloud_in->points[counter1].y = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].y;
											cloud_in->points[counter1].z = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].z;
											//cloud_in->points[counter1].r = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].r;
											//cloud_in->points[counter1].g = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].g;
											//cloud_in->points[counter1].b = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].b;

										//	cloud_in->points[counter1].r = 255;
											//cloud_in->points[counter1].g = 0;
										//	cloud_in->points[counter1].b = 0;

											mean_cloud_in_x = mean_cloud_in_x  + cloud_in->points[counter1].x;
											mean_cloud_in_y = mean_cloud_in_y + cloud_in->points[counter1].y;
											mean_cloud_in_z = mean_cloud_in_z  + cloud_in->points[counter1].z;
											

											/*pose_cloud_in->points[counter1].x = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].x;
											pose_cloud_in->points[counter1].y = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].y;
											pose_cloud_in->points[counter1].z = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].z;
											pose_cloud_in->points[counter1].r = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].r;
											pose_cloud_in->points[counter1].g = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].g;
											pose_cloud_in->points[counter1].b = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].b;*/
										}
									}
									//////////////////////////////////////////////////////////////////////////////////////////////


									cloud_out->points[counter2].x = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].x;
									cloud_out->points[counter2].y = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].y;
									cloud_out->points[counter2].z = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].z;
									cloud_out->points[counter2].r = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].r;
									cloud_out->points[counter2].g = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].g;
									cloud_out->points[counter2].b = befor_clustering_cloud_final2->points[nbrs_cluster_cloud[int(k / K)][int(k % K)]].b;
									////////////////////add 20 points around keypoint
									pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree2;
									kdtree2.setInputCloud(befor_clustering_cloud_final2); //////original cloud
									pcl::PointXYZRGB searchPoint2;
									searchPoint2.x = cloud_out->points[counter2].x;
									searchPoint2.y = cloud_out->points[counter2].y;
									searchPoint2.z = cloud_out->points[counter2].z;

									std::vector<int> pointIdxNKNSearch2(K1);
									std::vector<float> pointNKNSquaredDistance2(K1);

									if (kdtree2.nearestKSearch(searchPoint2, K1, pointIdxNKNSearch2, pointNKNSquaredDistance2) > 0)
									{

										for (int i = 0; i < pointIdxNKNSearch2.size(); i=i+1)
										{
											counter2++;
											cloud_out->points[counter2].x = befor_clustering_cloud_final2->points[pointIdxNKNSearch2[i]].x;
											cloud_out->points[counter2].y = befor_clustering_cloud_final2->points[pointIdxNKNSearch2[i]].y;
											cloud_out->points[counter2].z = befor_clustering_cloud_final2->points[pointIdxNKNSearch2[i]].z;
											//cloud_out->points[counter2].r = befor_clustering_cloud_final2->points[pointIdxNKNSearch2[i]].r;
											//cloud_out->points[counter2].g = befor_clustering_cloud_final2->points[pointIdxNKNSearch2[i]].g;
											//cloud_out->points[counter2].b = befor_clustering_cloud_final2->points[pointIdxNKNSearch2[i]].b;
											cloud_out->points[counter2].r = 255;
											cloud_out->points[counter2].g = 0;
											cloud_out->points[counter2].b = 0;
										}
									}










								}
							}
						}
						/////////////////////SELECT 20 POINTS AROUND DETECTED OBJECT POINTS/////////////////////////////////////////////////////////////////























						counter1++;
						counter2++;
						
						m++;
						break;
					
				}
			}
		}

					//////////////////////////////////////////LOCATE DETECTED OBJECT//////////////////////////////////////////////////////////////////////////////////////////////////

	

		/////centroid of cloud_in(scene) and cloud_out(object)/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		std::cerr << "\n center of cloud_in is: " << mean_cloud_in_x/ count_metched <<" "<< mean_cloud_in_y/ count_metched << " "<< mean_cloud_in_z/ count_metched <<" \n" << std::endl;
		//////////////////////////////CENTER of MATCHED POINTS/////////////////////////////
		float center_matched_points_x = mean_cloud_in_x / count_metched;
		float	center_matched_points_y = mean_cloud_in_y / count_metched;
		float center_matched_points_z = mean_cloud_in_z / count_metched;
	   /////////////////////////////////////////////////////////////////////////////
		// int KK1 = atoi(argv[7]);
        int KK1 = bounding_box_points;
			pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree1;
		kdtree1.setInputCloud(befor_clustering_cloud_final1); //////original cloud
		pcl::PointXYZRGB searchPoint1;
		searchPoint1.x = mean_cloud_in_x/ count_metched;
		searchPoint1.y = mean_cloud_in_y/ count_metched;
		searchPoint1.z = mean_cloud_in_z/ count_metched;
		cout << "\nsearchPoint1.x is" << searchPoint1.x<<" "<< searchPoint1.y<<" "<< searchPoint1.z<<"\n";
		//	std::string number(boost::lexical_cast<std::string>(cl_num++));
		std::vector<int> pointIdxNKNSearch1(KK1);
		std::vector<float> pointNKNSquaredDistance1(KK1);

		if (kdtree1.nearestKSearch(searchPoint1, KK1, pointIdxNKNSearch1, pointNKNSquaredDistance1) > 0)
		{

			for (int i = 0; i < pointIdxNKNSearch1.size(); ++i)
			{

				cloud_in1->points[i].x = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].x;
				cloud_in1->points[i].y = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].y;
				cloud_in1->points[i].z = befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].z;
				/*if (i < 100)
				{
					befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].r = 255;
					befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].g = 255;
					befor_clustering_cloud_final1->points[pointIdxNKNSearch1[i]].b = 0;
				}*/

			}
		}

		// writer.write<pcl::PointXYZ>("cloud_in.pcd", *cloud_in1, false);
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud_in1);
		feature_extractor.compute();
		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		pcl::PointXYZ min_point_OBB;
		pcl::PointXYZ max_point_OBB;
		pcl::PointXYZ position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);
		feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues(major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter(mass_center);

		viewer1->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, min_point_AABB.z + 0.01, 1.0, 0.0, 0.0, "AABB");
	
		viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
		pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
		pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
		pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
		pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));

			   
		//////////////////////////////CUBE SURFACE/////////////////////////////
		float cube_min_x = min_point_AABB.x; float cube_min_y = min_point_AABB.y; float cube_min_z = min_point_AABB.z;
		float cube_max_x = max_point_AABB.x; float cube_max_y = max_point_AABB.y; float cube_max_z = min_point_AABB.z+0.01;
		/////////////////////////////////////////////////////////////////////////////



	

		viewer1->addPointCloud<pcl::PointXYZRGB>(befor_clustering_cloud_final1, "sample cloud");


		//std::cout << "\ncl_num is" << cl_num;
		std::cout << "\n final feature strength  is" << final_feature;
		std::cout << "\n final feature strength  is" << final_feature / (i1);

		tend = time(0);
		cout << "It took " << difftime(tend, tstart) << " second(s)." << endl;



	while (!viewer1->wasStopped())
	{
		viewer1->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ofstream myfile;
	// myfile.open("matched.txt");
	// myfile << count_metched << "\n" << searchPoint1.x << "\n" << searchPoint1.y<< "\n" << searchPoint1.z;
	// myfile.close();
	// deallocate memory using delete[] operator
    recognise_res.poseStamped.pose.position.x  = mean_cloud_in_x / count_metched;
    recognise_res.poseStamped.pose.position.y  = mean_cloud_in_y / count_metched;
    recognise_res.poseStamped.pose.position.z  = mean_cloud_in_z / count_metched;
    recognise_res.poseStamped.header.frame_id = "camera_depth_optical_frame";
    recognise_res.poseStamped.header.stamp = ros::Time::now();
	for (int i = 0; i < M; i++)
		delete[] data1[i];

	delete[] data1;

	// deallocate memory using delete[] operator
	for (int i = 0; i < M; i++)
		delete[] data2[i];

	delete[] data2;

/////////////////////////////////////////////////////////////
	for (int i = 0; i <500; ++i)
		//                  ^^^^^ not levelSize.x
	{
		for (int j = 0; j < 500; ++j)
			//                  ^^^^^^^^^^^ not levelSize.y
		{
			delete[] features[i][j];
		}
		delete[] features[i];

	}
	delete[] features;

	for (int i = 0; i < 500; i++)
		delete[] centroid[i];

	delete[] centroid;

//////////////////////////////////////////////////////////////
	// deallocate memory using delete[] operator
	for (int i = 0; i < 500; i++)
		delete[] nbrs_cluster_cloud[i];

	delete[] nbrs_cluster_cloud;
	//////////////////////////////////////////////////////////////	
	// deallocate memory using delete[] operator
	for (int i = 0; i < 500; i++)
		delete[] sq_dis_bwt_nbrs[i];

	delete[] sq_dis_bwt_nbrs;
	////////////////////////////////////////////////////////////// 
	// deallocate memory using delete[] operator
	for (int i = 0; i < 500; i++)
		delete[] dis_cen_2_cen[i];

	delete[] dis_cen_2_cen;
	/////////////////////////////////
	delete[] loc_x1;
	delete[] loc_y1;
	delete[] loc_z1;
	delete[] loc_x2;
	delete[] loc_y2;
	delete[] loc_z2;

    return true;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "fevor2_server");
    ros::NodeHandle nh;

	Fevor fevor2_detection = Fevor(&nh);

    ROS_INFO_STREAM("Fevor2.0 server started!");
    ros::spin();
	
	return (0);
}