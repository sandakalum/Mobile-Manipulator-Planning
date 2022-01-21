/*C++17*/
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator> 
#include <map>
#include <memory>
#include <vector>
#include <cstdio>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
// #include <pair>
/*PCL*/
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

/*ROS*/
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <kuka_msgs/PoseExtraction.h>
#include <kuka_msgs/RSDPoseExtraction.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 


typedef struct {
    int maxIterations = 100;
    float distanceThreshold = 0.01;
}PlanarCoeficients;



class RSD {
private:

    typedef pcl::PrincipalRadiiRSD  DescriptorType;
    typedef pcl::Normal NormalType;
    typedef pcl::PointXYZ PointType;
    typedef pcl::ReferenceFrame RFType;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    ros::NodeHandle nh;
    // ros::Subscriber point_cloud_sub;
    ros::ServiceServer rsd_server;
    tf2_ros::Buffer& tfBuffer;


    PlanarCoeficients planar_coeficients;
    int visualisation  = 0;
    std::map<std::string,PointCloud::Ptr> model_map;

    // PointCloud::Ptr leaf_cell_model;
    // PointCloud::Ptr cast1_mold_model;
    // PointCloud::Ptr box_model;
    PointCloud::Ptr leaf_cell_centroid;
    PointCloud::Ptr leaf_cell_centroid2;

    PointCloud::Ptr bowl;
    PointCloud::Ptr fuel_can;
    PointCloud::Ptr fuel_tank_tap;
    PointCloud::Ptr water_tap;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cloud_p;
    pcl::PCDWriter writer;
    std::string frame_id;
    int inverted = 0;
    std::vector<float> argv{ 0.0,0.0,0.0, 0.04, 0,0,0.006,0.006 }; 
    float leaf_size = 0.002;
    std::string directory = "/home/hector/catkin_ws/src/erl_rnt/data/";
    std::string directory_rsd = "/home/hector/RELIB-KMR-Bin-Picking/relib_ws/src/rsd/data/";
    std::string directory_rsd_centroid = "/home/hector/RELIB-KMR-Bin-Picking/relib_ws/src/rsd/data/v2/";


  
    std::vector<std::string> new_names = {"bowl_sampled.pcd", "fuel_can_sampled.pcd", "fuel_tank_tap_sampled.pcd","water_tap_sampled.pcd" };
    std::vector<std::string> new_names_centroid = {"leaf_cell_centroid.pcd", "bowl_centroid.pcd", "fuel_can_centroid.pcd", 
                                "fuel_tank_tap_centroid.pcd","water_tap_centroid.pcd" };


public:
    RSD(ros::NodeHandle* _nh, tf2_ros::Buffer& _tf_Buffer, int _vis, int _inv, float _thresh, float _m_sr, float _s_sr) :nh(*_nh), tfBuffer(_tf_Buffer), cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                 cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), leaf_cell_centroid(new PointCloud), leaf_cell_centroid2(new PointCloud), bowl(new PointCloud), fuel_can(new PointCloud), fuel_tank_tap(new PointCloud), water_tap(new PointCloud){
        
        
        rsd_server = _nh->advertiseService("/rsd_detect", &RSD::recognise_callback, this);

        visualisation = _vis;
        inverted = _inv;
        argv[3] = _thresh;
        argv[6] = _m_sr;
        argv[7] = _s_sr;

        pcl::io::loadPCDFile<pcl::PointXYZ>(directory_rsd_centroid + new_names_centroid[0], *leaf_cell_centroid);
        pcl::io::loadPCDFile<pcl::PointXYZ>(directory_rsd_centroid + new_names_centroid[1], *bowl);
        pcl::io::loadPCDFile<pcl::PointXYZ>(directory_rsd_centroid + new_names_centroid[2], *fuel_can);
        pcl::io::loadPCDFile<pcl::PointXYZ>(directory_rsd_centroid + new_names_centroid[3], *fuel_tank_tap);
        pcl::io::loadPCDFile<pcl::PointXYZ>(directory_rsd_centroid + new_names_centroid[4], *water_tap);

        model_map.insert(std::pair<std::string, PointCloud::Ptr>("leaf_cell",leaf_cell_centroid));
        model_map.insert(std::pair<std::string, PointCloud::Ptr>("object_a", bowl));
        model_map.insert(std::pair<std::string, PointCloud::Ptr>("object_b", fuel_can));
        model_map.insert(std::pair<std::string, PointCloud::Ptr>("object_c", fuel_tank_tap));
        model_map.insert(std::pair<std::string, PointCloud::Ptr>("object_d", water_tap));
            }
    ~RSD(){
        
    }
    // Callbacks
    bool recognise_callback(kuka_msgs::RSDPoseExtraction::Request& request, kuka_msgs::RSDPoseExtraction::Response& recognise_res);
};



bool RSD::recognise_callback(kuka_msgs::RSDPoseExtraction::Request& request, kuka_msgs::RSDPoseExtraction::Response& recognise_res)
{
    // Frist collect the neccesary params
    if(request.threshold != 0.0)          argv[3] = request.threshold;
    if(request.modelSearchRadius != 0.0)  argv[6] = request.modelSearchRadius;
    if(request.sceneSearchRadius != 0.0)  argv[7] = request.sceneSearchRadius;

    if(request.leafSize != 0.0 && request.leafSize < 0.0102){
        leaf_size = request.leafSize;
    }else{
        leaf_size = 0.002;
        ROS_WARN("Leaf size too big, using default value: 0.002");
    } 
    
    visualisation = int(request.visualise);

    //Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>);

    boost::shared_ptr<PointCloud const> new_scene_point_cloud;
    // Should wait for a new point cloud from the scene
    ROS_INFO("Waiting for new scene point cloud...");
    new_scene_point_cloud = ros::topic::waitForMessage<PointCloud>("camera/depth/color/points", nh);
    if(new_scene_point_cloud != NULL){
       
        copyPointCloud(*new_scene_point_cloud,*cloud);
    }
    // Starting planarsurfaces removal
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(planar_coeficients.maxIterations); //// 100
    seg.setDistanceThreshold(planar_coeficients.distanceThreshold); // bigger then more planner object cut..............less and extract.setNegative(false); then more planner object we got 0.005
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_p);

    ROS_INFO_STREAM("Planar surfaces removed...");


    /**
     * DownSampling
     * @param cloud_p
     * @param cloud_model 
     */
    pcl::PCLPointCloud2::Ptr cloud_p_pcl2(new pcl::PCLPointCloud2());

    // Casting to PCLPontCloud2
    pcl::toPCLPointCloud2(*cloud_p, *cloud_p_pcl2);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_p_pcl2);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_p_pcl2);

    ROS_INFO_STREAM("Scene cloud downsampled...");


    pcl::PCLPointCloud2::Ptr cloud_model_pcl2(new pcl::PCLPointCloud2());

    // Casting to PCLPontCloud2
    pcl::toPCLPointCloud2(*(model_map.at(request.object_name)), *cloud_model_pcl2);
    sor.setInputCloud(cloud_model_pcl2);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_model_pcl2);

    ROS_INFO_STREAM("Model cloud downsampled...");

    if(!inverted){
        // Copy back Scene pcl downsampled to PointCloud format
        pcl::fromPCLPointCloud2(*cloud_p_pcl2,*cloud_scene);
        ROS_INFO_STREAM("Scene caputred");

        // Copy back Model pcl downsampled to PointCloud format
        pcl::fromPCLPointCloud2(*cloud_model_pcl2,*cloud_model);
        ROS_INFO_STREAM("Model caputred");

    }else{
        // Copy back Model pcl downsampled to PointCloud format
        pcl::fromPCLPointCloud2(*cloud_model_pcl2,*cloud_scene);
        ROS_INFO_STREAM("Model caputred");

        // Copy back Scene pcl downsampled to PointCloud format
        pcl::fromPCLPointCloud2(*cloud_p_pcl2,*cloud_model);
        ROS_INFO_STREAM("Scene caputred");

    }
    // Read a PCD file from disk.
    // if(!inverted){
    //     copyPointCloud(*cloud_p,*cloud_scene);
    //     ROS_INFO_STREAM("Scene caputre");
    //     copyPointCloud(*(model_map.at(request.object_name)), *cloud_model);
    //     ROS_INFO_STREAM("model caputre");
    // }else
    // {
    //     copyPointCloud(*cloud_p,*cloud_model);
    //     ROS_INFO_STREAM("Scene caputre");
    //     copyPointCloud(*(model_map.at(request.object_name)), *cloud_scene);
    //     ROS_INFO_STREAM("model caputre");
    // }

    /**
     * RSD recognition.
     *
     * @param cloud Container whose values are summed.
     * @return nothing  of `values`, or 0.0 if `values` is empty.
     */

    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals_model(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_scene(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors_model(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors_scene(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());

    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());


    

    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_model) != 0)
    // {
    // 	return -1;
    // }
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_scene) != 0)
    // {
    // 	return -1;
    // }

    /*float size = argv[6];
    float size1 = atof(argv[7]);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_model);
    sor.setLeafSize(size, size, size);
    sor.filter(*cloud_model);

    pcl::VoxelGrid<pcl::PointXYZ> sor1;
    sor1.setInputCloud(cloud_scene);
    sor1.setLeafSize(size1, size1, size1);
    sor1.filter(*cloud_scene);*/


    float theta = argv[5];
    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    //Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_1.translation() << argv[4], 0.0, 0.0;
    transform_1.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud_model, *cloud_model, transform_1);
    // Note: you would usually perform downsampling now. It has been omitted here
    // for simplicity, but be aware that computation can take a long time.

    // Estimate the normals.
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_model);
    normalEstimation.setRadiusSearch(0.015);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals_model);

    normalEstimation.setInputCloud(cloud_scene);
    normalEstimation.compute(*normals_scene);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
//  Downsample Clouds to Extract keypoints
//

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(cloud_model);
    uniform_sampling.setRadiusSearch(argv[6]);
    uniform_sampling.filter(*model_keypoints);
    std::cout << "Model total points: " << cloud_model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

    uniform_sampling.setInputCloud(cloud_scene);
    uniform_sampling.setRadiusSearch(argv[7]);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << cloud_scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // RSD estimation object.

    pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
    rsd.setRadiusSearch(0.015);
    rsd.setInputCloud(model_keypoints);
    rsd.setInputNormals(normals_model);
    rsd.setSearchMethod(kdtree);// Search radius, to look for neighbors. Note: the value given here has to be// larger than the radius used to estimate the normals.

    rsd.setSearchSurface(cloud_model);
    rsd.compute(*descriptors_model);

    rsd.setInputCloud(scene_keypoints);
    rsd.setInputNormals(normals_scene);
    rsd.setSearchSurface(cloud_scene);
    rsd.compute(*descriptors_scene);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
//  Find Model-Scene Correspondences with KdTree
//
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(descriptors_model);
    for (size_t i = 0; i < descriptors_scene->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        //if (!pcl_isfinite(descriptors_scene->at(i).descriptor[0])) //skipping NaNs
        //{
        //continue;
        //}
        int found_neighs = match_search.nearestKSearch(descriptors_scene->at(i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;
    pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
    pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(0.015);

    rf_est.setInputCloud(model_keypoints);
    rf_est.setInputNormals(normals_model);
    rf_est.setSearchSurface(cloud_model);
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_keypoints);
    rf_est.setInputNormals(normals_scene);
    rf_est.setSearchSurface(cloud_scene);
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(argv[3]); /////important//////////0.023/////////////////////////////////////////////////////////////////////
    clusterer.setHoughThreshold(5);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(model_keypoints);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize(rototranslations, clustered_corrs);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
//  Output results
    float model_point_mean_x=0; float model_point_mean_y = 0; float model_point_mean_z = 0;
    int max_corres = 0; int index_corres = 1;
    //std::cout << "Model instances found: " << rototranslations.size() << std::endl;
    for (size_t i = 0; i < rototranslations.size(); ++i)
    {
    //	std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        //std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

        // Print the rotation matrix and translation vector
    
        
        if (clustered_corrs[i].size() > max_corres)
        {
            
            max_corres = clustered_corrs[i].size();
            index_corres = i;

        }

    }

    // pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    
    //  Visualization
    // if(visualisation){
    //     viewer.addPointCloud(cloud_scene, "scene_cloud");
    //     viewer.addCoordinateSystem(0.4,0.0,0.0,0.0,std::string("scene"),0);
    // }


    pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cloud_model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
    pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);

    pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());


    if(visualisation){
        // viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
    }
    tf::Matrix3x3 object_rotation_cf;
    tf::Vector3 object_translation_cf;
    for (size_t i = index_corres; i < index_corres+1; ++i)
    {
        Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);
        printf("\n");
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        printf("\n");
        printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

        // object_rotation_cf.setValue(rotation(0, 0),rotation(1, 0),rotation(2, 0),
        //                             rotation(0, 1),rotation(1, 1),rotation(2, 1),
        //                             rotation(0, 2),rotation(1, 2),rotation(2, 2)
        // );
        object_rotation_cf.setValue(rotation(0, 0),rotation(0, 1),rotation(0, 2),
                                    rotation(1, 0),rotation(1, 1),rotation(1, 2),
                                    rotation(2, 0),rotation(2, 1),rotation(2, 2)
        );
        object_translation_cf.setValue(translation(0),translation(1), translation(2));

        //pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cloud_model, *rotated_model, rototranslations[i]);  
        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        //pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
        
        //if(visualisation)
            // viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

            //if (show_correspondences_)
        {
            for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
            {
                std::stringstream ss_line;
                ss_line << "correspondence_line" << i << "_" << j;
                PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
                PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

                //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            //	viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
                model_point_mean_x = model_point_mean_x + model_point.x;
                model_point_mean_y = model_point_mean_y + model_point.y;
                model_point_mean_z = model_point_mean_z + model_point.z;
                
            }
        }
    }
    //ROS_INFO_STREAM("Before: "<<); 
    model_point_mean_x = model_point_mean_x / clustered_corrs[index_corres].size();
    model_point_mean_y = model_point_mean_y/clustered_corrs[index_corres].size();
    model_point_mean_z = model_point_mean_z/clustered_corrs[index_corres].size();
    //std::cout << model_point_mean_x << model_point_mean_y << model_point_mean_z;

    if(visualisation){

        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Correspondence Grouping", true));

        viewer->addPointCloud(cloud_scene, "scene_cloud");
        viewer->addCoordinateSystem(0.4,0.0,0.0,0.0, std::string("scene"),0);
        viewer->addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
        viewer->addPointCloud(rotated_model, rotated_model_color_handler, "instance");
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
        viewer->close();
    }else{
        ROS_INFO("NO interactive");
        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Correspondence Grouping", false));

        viewer->addPointCloud(cloud_scene, "scene_cloud");
        viewer->addCoordinateSystem(0.4,0.0,0.0,0.0, std::string("scene"),0);
        viewer->addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
        viewer->addPointCloud(rotated_model, rotated_model_color_handler, "instance");

        // viewer->sav
        // writer.write("home/hector/RELIB-KMR-Bin-Picking/detection_result.pcd", *);
        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce();
        // }
        // viewer->close()
    }
    //////////////////
    ros::Rate rate(10.0);
    geometry_msgs::TransformStamped transformStamped;
    //Eigen::Matrix4d camera2base;
    while (true){
        try{
            transformStamped = tfBuffer.lookupTransform("base_footprint", "camera_depth_optical_frame",
                                    ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
    /////////////////
    // CONVERT QUATERNION TO RPY FROM TRANSFORM_STAMPED

    tf::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w
    );
    tf::Matrix3x3 m(q);
    tf::Vector3 translation(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z
    );

    // Compute object pose respective to the world frame (base_footprint)
    tf::Transform camera2baseFootprint(m,translation);
    tf::Transform model2camera(object_rotation_cf, object_translation_cf);   
    tf::Vector3 model2baseFootprint = camera2baseFootprint*model2camera.getOrigin();

    ROS_INFO_STREAM("World x: "<<model2baseFootprint.getX()<<" Y: "<<model2baseFootprint.getY()<<" Z: "<<model2baseFootprint.getZ());
    recognise_res.poseStamped.header.frame_id= "base_footprint";
    recognise_res.poseStamped.pose.position.x = model2baseFootprint.getX();
    recognise_res.poseStamped.pose.position.y = model2baseFootprint.getY();
    recognise_res.poseStamped.pose.position.z = model2baseFootprint.getZ();


    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped2;
    
    transformStamped2.header.stamp = ros::Time::now();
    transformStamped2.header.frame_id = "base_footprint";
    transformStamped2.child_frame_id = "object_pose";
    transformStamped2.transform.translation.x = model2baseFootprint.getX();
    transformStamped2.transform.translation.y = model2baseFootprint.getY();
    transformStamped2.transform.translation.z = model2baseFootprint.getZ();

    tf2::Quaternion q2;
    q2.setRPY(0, 0, 0);
    transformStamped2.transform.rotation.x = q2.x();
    transformStamped2.transform.rotation.y = q2.y();
    transformStamped2.transform.rotation.z = q2.z();
    transformStamped2.transform.rotation.w = q2.w();
    br.sendTransform(transformStamped2);
    ROS_INFO("Boradcasting object pose! ");

    return true;

}
//base_footprint
//camera_depth_optical_frame

int main (int argc, char **argvv) 
{
    ros::init(argc, argvv, "rsd_detectionv2");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer2;
    tf2_ros::TransformListener tfListener(tfBuffer2);

    // ros::AsyncSpinner spinner(2); // Use 4 threads
    // spinner.start();

    int visualisation = 0, inverted = 0;
    float thresh = 0.04, model_search_radius = 0.006, scene_search_radius = 0.006;

    if(argvv[1] != NULL)
        visualisation = atoi(argvv[1]);
    if(argvv[2]!= NULL)
        inverted = atoi(argvv[2]);
    if(argvv[3]!= NULL)
        thresh = atof(argvv[3]);
    if(argvv[4]!= NULL)
        model_search_radius = atof(argvv[4]);
    if(argvv[5]!= NULL)
        scene_search_radius = atof(argvv[5]);

    RSD rsd_detection(&nh, tfBuffer2, visualisation, inverted, thresh, model_search_radius, scene_search_radius);

    ROS_INFO_STREAM("RSD server  ready!");

    // ros::waitForShutdown();
    ros::spin();
    return 0;
}