// #include <ods_chair.h>

// #define SMARTPAL4 1
// #define SMARTPAL5 0

// double C_OFFSET_X = 0.02;
// double C_OFFSET_Y = 0.14;
// double C_OFFSET_Z = 1.38;
// double C_OFFSET_YAW = 21.6;

// typedef struct{
//     double min_x;
//     double max_x;
//     double min_y;
//     double max_y;
//     double min_z;
//     double max_z;
// }Model;

// Model m_size;

// typedef struct{
//     double x;
//     double y;
//     double theta;
// }Robot;

// typedef struct{
//     double x;
//     double y;
//     double z;
//     double theta;
// }Furniture;

// Robot robot;
// Furniture chair;

// void transformation(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& model)
// {
//     std::cout << "transformation" << std::endl;

//     double camera_x = robot.x + C_OFFSET_Y*cos(robot.theta) + C_OFFSET_X*sin(robot.theta);
//     double camera_y = robot.y + C_OFFSET_Y*sin(robot.theta) - C_OFFSET_X*cos(robot.theta);
//     double camera_z = C_OFFSET_Z;
//     double camera_yaw = C_OFFSET_YAW*(PI/180.0);

//     std::cout << "chair position : " << chair.x << " " << chair.y << " " << chair.z << " " << chair.theta <<
//     std::endl;

//     robot.theta *= PI/180.0;
//     chair.theta *= PI/180.0;

//     Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

//     t1(0,0) = sin(robot.theta);    t1(0,1) = -sin(camera_yaw)*cos(robot.theta);    t1(0,2) =
//     cos(camera_yaw)*cos(robot.theta);    t1(0,3) = camera_x;
//     t1(1,0) = -cos(robot.theta);    t1(1,1) = -sin(camera_yaw)*sin(robot.theta);     t1(1,2) =
//     cos(camera_yaw)*sin(robot.theta);     t1(1,3) = camera_y;
//     t1(2,0) = 0;                    t1(2,1) = -cos(camera_yaw);                      t1(2,2) = -sin(camera_yaw);
//     t1(2,3) = camera_z;
//     t1(3,0) = 0;                    t1(3,1) = 0;                                    t1(3,2) = 0; t1(3,3) = 1;

//     pcl::transformPointCloud(cloud, cloud, t1);

//     t1(0,0) = cos(chair.theta);     t1(0,1) = sin(chair.theta);    t1(0,2) = 0;    t1(0,3) = chair.x;
//     t1(1,0) = -sin(chair.theta);    t1(1,1) = cos(chair.theta);    t1(1,2) = 0;    t1(1,3) = chair.y;
//     t1(2,0) = 0;                    t1(2,1) = 0;                   t1(2,2) = 1;    t1(2,3) = chair.z;
//     t1(3,0) = 0;                    t1(3,1) = 0;                   t1(3,2) = 0;    t1(3,3) = 1;

//     pcl::transformPointCloud(model, model, t1);

//     return;
// }

// void transformation2(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
// {
//     Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

//     t1(0,0) = 1;    t1(0,1) = 0;    t1(0,2) = 0;    t1(0,3) = -chair.x;
//     t1(1,0) = 0;   t1(1,1) = 1;    t1(1,2) = 0;    t1(1,3) = -chair.y;
//     t1(2,0) = 0;                    t1(2,1) = 0;                    t1(2,2) = 1;    t1(2,3) = -chair.z;
//     t1(3,0) = 0;                    t1(3,1) = 0;                    t1(3,2) = 0;    t1(3,3) = 1;

//     pcl::transformPointCloud(cloud, cloud, t1);

//     t1(0,0) = cos(-chair.theta);    t1(0,1) = sin(-chair.theta);    t1(0,2) = 0;    t1(0,3) = 0;
//     t1(1,0) = -sin(-chair.theta);   t1(1,1) = cos(-chair.theta);    t1(1,2) = 0;    t1(1,3) = 0;
//     t1(2,0) = 0;                    t1(2,1) = 0;                    t1(2,2) = 1;    t1(2,3) = 0;
//     t1(3,0) = 0;                    t1(3,1) = 0;                    t1(3,2) = 0;    t1(3,3) = 1;

//     pcl::transformPointCloud(cloud, cloud, t1);

//     return;
// }

// void filtering(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& model)
// {
//     std::cout << "filtering" << std::endl;

//     m_size.min_x = 100.0;
//     m_size.max_x = -100.0;
//     m_size.min_y = 100.0;
//     m_size.max_y = -100.0;
//     m_size.min_z = 100.0;
//     m_size.max_z = -100.0;

//     for(int i=0;i<model.points.size();i++){
//         if(model.points[i].x < m_size.min_x) m_size.min_x = model.points[i].x;
//         if(model.points[i].x > m_size.max_x) m_size.max_x = model.points[i].x;
//         if(model.points[i].y < m_size.min_y) m_size.min_y = model.points[i].y;
//         if(model.points[i].y > m_size.max_y) m_size.max_y = model.points[i].y;
//         if(model.points[i].z < m_size.min_z) m_size.min_z = model.points[i].z;
//         if(model.points[i].z > m_size.max_z) m_size.max_z = model.points[i].z;
//     }

//     pcl::PassThrough<pcl::PointXYZRGB> pass;
//     pass.setInputCloud (cloud.makeShared());
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimits (m_size.min_x-0.1, m_size.max_x+0.1);
//     pass.filter (cloud);

//     pass.setInputCloud (cloud.makeShared());
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimits (m_size.min_y-0.1, m_size.max_y+0.1);
//     pass.filter (cloud);

//     pass.setInputCloud (cloud.makeShared());
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (m_size.min_z-0.1, m_size.max_z+0.3);
//     pass.filter (cloud);

//     return;
// }

// //重心位置を求める
// pcl::PointXYZ g_pos(pcl::PointCloud<pcl::PointXYZ>& cloud)
// {
//     pcl::PointXYZ g;

//     for(int i=0;i < cloud.size();i++){
//         g.x += cloud.points[i].x;
//         g.y += cloud.points[i].y;
//         g.z += cloud.points[i].z;
//     }

//     g.x = g.x / cloud.size();
//     g.y = g.y / cloud.size();
//     g.z = g.z / cloud.size();

//     return g;
// }

// pcl::PointXYZ g_pos(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
// {
//     pcl::PointXYZ g;

//     for(int i=0;i < cloud.size();i++){
//         g.x += cloud.points[i].x;
//         g.y += cloud.points[i].y;
//         g.z += cloud.points[i].z;
//     }

//     g.x = g.x / cloud.size();
//     g.y = g.y / cloud.size();
//     g.z = g.z / cloud.size();

//     return g;
// }

// //グリッド化
// void grid(object_map& m)
// {
//     double max_x = -10.0;  double max_y = -10.0;  double max_z = -10.0;
//     double min_x = 10.0;  double min_y = 10.0;  double min_z = 10.0;
//     int int_max_x = -100; int int_min_x = 100; int int_max_y = -100; int int_min_y = 100;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     *tmp_cloud = m.cloud;

//     for(int i=0;i<tmp_cloud->size();i++){
//         /*if((tmp_cloud->points[i].x < -1.0)||(tmp_cloud->points[i].y < -1.0)||(tmp_cloud->points[i].z < -1.0)){
//             continue;
//         }*/
//         if(tmp_cloud->points[i].x > max_x) max_x = tmp_cloud->points[i].x;
//         if(tmp_cloud->points[i].x < min_x) min_x = tmp_cloud->points[i].x;
//         tmp_cloud->points[i].x = (int)(tmp_cloud->points[i].x*100);
//         if(tmp_cloud->points[i].x > int_max_x) int_max_x = tmp_cloud->points[i].x;
//         if(tmp_cloud->points[i].x < int_min_x) int_min_x = tmp_cloud->points[i].x;
//         if(tmp_cloud->points[i].y > max_y) max_y = tmp_cloud->points[i].y;
//         if(tmp_cloud->points[i].y < min_y) min_y = tmp_cloud->points[i].y;
//         tmp_cloud->points[i].y = (int)(tmp_cloud->points[i].y*100);
//         if(tmp_cloud->points[i].y > int_max_y) int_max_y = tmp_cloud->points[i].y;
//         if(tmp_cloud->points[i].y < int_min_y) int_min_y = tmp_cloud->points[i].y;
//         if(tmp_cloud->points[i].z > max_z) max_z = tmp_cloud->points[i].z;
//         if(tmp_cloud->points[i].z < min_z) min_z = tmp_cloud->points[i].z;
//         tmp_cloud->points[i].z = (int)(tmp_cloud->points[i].z*100);
//     }

//     m.size_x = max_x - min_x;
//     m.size_y = max_y - min_y;
//     m.size_z = max_z - min_z;

//     return;
// }

// //クラスタリング
// void clustering(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& chair, double th1,
// double th2, int id)
// {
//     // Creating the KdTree object for the search method of the extraction
//     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//     tree->setInputCloud (cloud.makeShared ());

//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//     ec.setClusterTolerance (th1);
//     ec.setMinClusterSize (th2);
//     ec.setMaxClusterSize (100000);
//     ec.setSearchMethod (tree);
//     ec.setInputCloud (cloud.makeShared ());
//     ec.extract (cluster_indices);

//     ///std::cout << "クラスタリング開始" << std::endl;
//     int m = 0;
//     float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
//     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end ();
//     ++it){
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);

//         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
//             cloud_cluster_rgb->points.push_back (cloud.points[*pit]);
//         }
//         cloud_cluster_rgb->width = cloud_cluster_rgb->points.size ();
//         cloud_cluster_rgb->height = 1;
//         cloud_cluster_rgb->is_dense = true;

//         if((id == 2) || (id == 3)){
//             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//             pcl::copyPointCloud(*cloud_cluster_rgb, *cloud_cluster);

//             for(int i=0;i<cloud_cluster_rgb->points.size();i++){
//                 cloud_cluster_rgb->points[i].r = colors[m%6][0];
//                 cloud_cluster_rgb->points[i].g = colors[m%6][1];
//                 cloud_cluster_rgb->points[i].b = colors[m%6][2];
//             }

//             object_map obj_m;
//             obj_m.cloud = *cloud_cluster;
//             obj_m.cloud_rgb = *cloud_cluster_rgb;
//             obj_m.g = g_pos(*cloud_cluster);
//             grid(obj_m);

//             //if(obj_m.g.z > 0.0) break;
//             //else if((obj_m.g.x < -0.16) || (0.16 < obj_m.g.x)) break;

//             obj_m.tf = id;

//             Object_Map.push_back(obj_m);
//             m++;

//             chair += *cloud_cluster_rgb;
//         }
//     }

//     return;
// }

// //重心位置合わせ
// void g_matching(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& model)
// {
//     pcl::PointXYZ g1, g2;
//     Eigen::Matrix4f r(Eigen::Matrix4f::Identity());

//     g1 = g_pos(cloud);
//     g2 = g_pos(model);

//     r(0,0) = 1;  r(0,1) = 0;  r(0,2) = 0;  r(0,3) = g2.x - g1.x;
//     r(1,0) = 0;  r(1,1) = 1;  r(1,2) = 0;  r(1,3) = g2.y - g1.y;
//     r(2,0) = 0;  r(2,1) = 0;  r(2,2) = 1;  r(2,3) = g2.z - g1.z;
//     r(3,0) = 0;  r(3,1) = 0;  r(3,2) = 0;  r(3,3) = 1;
//     pcl::transformPointCloud (cloud, cloud, r);

//     return;
// }

// // 背もたれ位置合わせ
// void back_matching(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& model)
// {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointXYZ g1, g2;
//     double min_x=1000.0, max_x = -1000.0, min_y=1000.0, max_y=-1000.0;

//     pcl::PassThrough<pcl::PointXYZRGB> pass;
//     pass.setInputCloud (cloud.makeShared());
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (m_size.max_z*3.0/4.0, m_size.max_z);
//     pass.filter (*tmp);

//     Eigen::Matrix4f r(Eigen::Matrix4f::Identity());

//     g1.x = (m_size.max_x + m_size.min_x)/2.0;
//     g1.y = (m_size.max_y + m_size.min_y)/2.0;
//     g1.z = 0;

//     std::cout << g1 << std::endl;

//     r(0,0) = 0;  r(0,1) = 0;  r(0,2) = 0;  r(0,3) = -g1.x;
//     r(1,0) = 0;  r(1,1) = 0;  r(1,2) = 0;  r(1,3) = -g1.y;
//     r(2,0) = 0;  r(2,1) = 0;  r(2,2) = 1;  r(2,3) = -g1.z;
//     r(3,0) = 0;  r(3,1) = 0;  r(3,2) = 0;  r(3,3) = 1;

//     pcl::transformPointCloud (*tmp, *tmp, r);

//     for(int i=0;i<tmp->points.size();i++){
//         if(tmp->points[i].x < min_x) min_x = tmp->points[i].x;
//         if(tmp->points[i].x > max_x) max_x = tmp->points[i].x;
//         if(tmp->points[i].y < min_y) min_y = tmp->points[i].y;
//         if(tmp->points[i].y > max_y) max_y = tmp->points[i].y;
//     }

//     g2.x = (max_x - min_x)/2.0;
//     g2.y = (max_y - min_y)/2.0;
//     g2.z = 0;

//     std::cout << max_x << " " << min_x << std::endl;
//     std::cout << max_y << " " << min_y << std::endl;
//     std::cout << g2 << std::endl;

//     double th, rad;
//     rad = atan2(g2.x, g2.y);
//     th = rad*(double)(180/PI);

//     std::cout << th << std::endl;
//     std::cout << rad << std::endl;

//     r(0,0) =  cos(-rad);  r(0,1) = sin(-rad);  r(0,2) = 0;  r(0,3) = 0;
//     r(1,0) = -sin(-rad);  r(1,1) = cos(-rad);  r(1,2) = 0;  r(1,3) = 0;
//     r(2,0) =          0;  r(2,1) =         0;  r(2,2) = 1;  r(2,3) = 0;
//     r(3,0) =          0;  r(3,1) =         0;  r(3,2) = 0;  r(3,3) = 1;
//     pcl::transformPointCloud (cloud, cloud, r);

//     return;
// }

// //ダウンサンプリングを行う
// void downsampling(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float th)
// {
//     pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//     sor.setInputCloud (cloud.makeShared());
//     sor.setLeafSize (th, th, th);
//     sor.filter (cloud);

//     return;
// }

// // レジストレーション
// void registration(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& dsp_cloud,
// pcl::PointCloud<pcl::PointXYZRGB>& model, Eigen::Matrix4f& m)
// {
//     pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//     icp.setInputSource(dsp_cloud.makeShared ());
//     icp.setInputTarget(model.makeShared ());
//     pcl::PointCloud<pcl::PointXYZRGB> Final;
//     icp.align(Final);
//     m = icp.getFinalTransformation();
//     std::cout << m << std::endl;

//     pcl::transformPointCloud (cloud, cloud, m);
//     pcl::transformPointCloud (dsp_cloud, dsp_cloud, m);

//     return;
// }

// void make_chairvoxel(pcl::PointCloud<pcl::PointXYZRGB>& model)
// {
//     for(int k=0;k<SIZE_Z;k++){
//         for(int j=0;j<SIZE_Y;j++){
//             for(int i=0;i<SIZE_X;i++){
//                 C_voxel[i][j][k] = 0;
//             }
//         }
//     }

//     for(int i=0;i<model.points.size();i++){
//         int X = 33*(model.points[i].x-m_size.min_x);
//         int Y = 33*(model.points[i].y-m_size.min_y);
//         int Z = 33*(model.points[i].z-m_size.min_z);

//         if(!(C_voxel[X][Y][Z])){
//             C_voxel[X][Y][Z] = 1;
//         }
//     }

//     return;
// }

// // remove chair
// void remove_chair(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& rmc)
// {
//     for(int i=0;i<cloud.points.size();i++){
//         //if(cloud.points[i].z > 0.0) continue;
//         int X = 33*(cloud.points[i].x-m_size.min_x);
//         int Y = 33*(cloud.points[i].y-m_size.min_y);
//         int Z = 33*(cloud.points[i].z-m_size.min_z);

//         if(((X < 0) || (SIZE_X <= X)) || ((Y < 0) || (SIZE_Y <= Y)) || ((Z < 2.0) || (SIZE_Z <= Z))){
//             continue;
//         }

//         if(!(C_voxel[X][Y][Z])){
//             rmc.push_back(cloud.points[i]);
//         }
//     }

//     return;
// }

// void segment_matching()
// {
//     pcl::PointXYZ g1, g2;

//     for(int j=0;j<Object_Map.size();j++){
//         int c = 0;
//         if(Object_Map[j].tf != 2) continue;
//         g1 = Object_Map[j].g;

//         for(int i=0;i<Object_Map.size();i++){
//             if(Object_Map[i].tf != 3) continue;
//             g2 = Object_Map[i].g;
//             double dis = 0.0;
//             dis = sqrt((g2.x-g1.x)*(g2.x-g1.x)+(g2.y-g1.y)*(g2.y-g1.y)+(g2.z-g1.z)*(g2.z-g1.z));

//             if(dis < 0.05){
//                 if((abs(Object_Map[j].size_x - Object_Map[i].size_x) < 0.05) && (abs(Object_Map[j].size_y -
//                 Object_Map[i].size_y) < 0.05) && (abs(Object_Map[j].size_z - Object_Map[i].size_z) < 0.02)){
//                     std::cout << "同じ物品" << i << " " << j << std::endl;
//                     std::cout << "size : " << abs(Object_Map[j].size_x - Object_Map[i].size_x) << " " <<
//                     abs(Object_Map[j].size_y - Object_Map[i].size_y) << " " << abs(Object_Map[j].size_z -
//                     Object_Map[i].size_z) << std::endl;
//                     Object_Map[i].tf = 0;
//                     Object_Map[j].tf = 0;
//                     c = 1;
//                     break;
//                 }
//                 else std::cout << i << " " << j << " 大きさが違う" << abs(Object_Map[j].size_x -
//                 Object_Map[i].size_x) << " " << abs(Object_Map[j].size_y - Object_Map[i].size_y) << " " <<
//                 abs(Object_Map[j].size_z - Object_Map[i].size_z) << std::endl;
//             }
//             if(c == 0)	std::cout << i << " " << j << " 近くに物品なし" << dis << std::endl;
//         }
//         //if(c == 0) Object_Map[j].tf = 1;
//     }

//     return;
// }

// ////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////
// //main function
// // to detect change of objects belong to shelf
// ////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////
// bool change_detection (tms_msg_ss::ods_furniture::Request &req, tms_msg_ss::ods_furniture::Response &res)
// {
//     // local variable declaration
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

//     // init point cloud visualization
//     pcl::visualization::CloudViewer viewer("Cloud Viewer");

//     //*******************************
//     //capture point cloud
//     //*******************************
//     tms_msg_ss::ods_pcd srv;
//     srv.request.id = 3;
//     if(commander_to_kinect_capture.call(srv)){
//         pcl::fromROSMsg (srv.response.cloud, *cloud1);
//     }
//     res.cloud = srv.response.cloud;

//     // std::stringstream ss1;
//     // int id;
//     // std::cout << "file id : ";
//     // scanf("%d", &id);
//     // ss1 << "/home/kuwahata/デスクトップ/実験/chair/extract/ex/chair" << id << ".pcd";

//     //pcl::io::loadPCDFile ("/tmp/tms_ss_ods/ods_change_detection/chair/sample.pcd", *cloud1);
//     //pcl::io::loadPCDFile (ss1.str(), *cloud1);
//     pcl::toROSMsg (*cloud1, res.cloud);

//     //*******************************
//     //input PointCloud data
//     //*******************************
//     pcl::fromROSMsg (req.cloud, *cloud2);
//     pcl::fromROSMsg (req.model, *model);
//     chair.x = req.furniture.position.x/1000.0;
//     chair.y = req.furniture.position.x/1000.0;
//     chair.z = req.furniture.position.x/1000.0;
//     chair.theta = req.furniture.orientation.z;
//     robot.x = req.robot.x/1000.0;
//     robot.y = req.robot.y/1000.0;
//     robot.theta = req.robot.theta;

//     //*******************************
//     // place capture data to model
//     //*******************************
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr tfm_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

//     transformation(*cloud1, *model);

//     pcl::copyPointCloud(*model, *tfm_cloud);

//     for(int i=0;i<tfm_cloud->points.size();i++){
//         tfm_cloud->points[i].r = 255;
//         tfm_cloud->points[i].g = 0;
//         tfm_cloud->points[i].b = 0;
//     }

//     *tfm_cloud += *cloud1;

//     pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/tfm_cloud.pcd", *tfm_cloud);

//     //*******************************
//     // filtering by using model
//     //*******************************
//     filtering(*cloud1, *model);

//     pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/filter.pcd", *cloud1);

//     // matching back
//     //back_matching(*cloud1, *model);

//     // downsampling
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr dsp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::copyPointCloud(*cloud1, *dsp_cloud);

//     downsampling(*dsp_cloud, 0.01);

//     // icp algorithm
//     std::cout << "registration1" << std::endl;
//     Eigen::Matrix4f m;
//     int n =0;

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp1 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::copyPointCloud(*model, *tmp1);

//     for(int i=0;i<tmp1->points.size();i++){
//         tmp1->points[i].r = 255;
//         tmp1->points[i].g = 0;
//         tmp1->points[i].b = 0;
//     }

//     while (1){

//         registration(*cloud1, *dsp_cloud, *model, m);

//         if((double)(m(0,0)+m(1,1)+m(2,2)+m(3,3)) >= 4){
//             if(n > 5) break;
//         }

//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//         pcl::copyPointCloud(*cloud1, *tmp2);

//         for(int i=0;i<tmp2->points.size();i++){
//             tmp2->points[i].r = 255;
//             tmp2->points[i].g = 255;
//             tmp2->points[i].b = 255;
//         }

//         *m_cloud = *tmp1 + *tmp2;

//         viewer.showCloud (m_cloud);
//         n++;
//     }
//     pcl::io::savePCDFile ("/tmp/tms_ss_ods/ods_change_detection/chair/registration1.pcd", *m_cloud, false);

//     // init c_voxel
//     std::cout << "init chair_voxel" << std::endl;
//     make_chairvoxel(*model);

//     // remove chair
//     std::cout << "remove chair" << std::endl;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rmc1 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rmc2 (new pcl::PointCloud<pcl::PointXYZRGB>);

//     // std::stringstream ss3;
//     // ss3 << "/home/kuwahata/デスクトップ/実験/chair/extract/ex/obj_cloud" << id-1 << ".pcd";
//     // pcl::io::loadPCDFile (ss3.str(), *rmc2);

//     remove_chair(*cloud1, *rmc1);
//     //remove_chair(*cloud2, *rmc2);

//     pcl::io::savePCDFile ("/tmp/tms_ss_ods/ods_change_detection/chair/remove_chair1.pcd", *rmc1, false);
//     //pcl::io::savePCDFile ("/tmp/tms_ss_ods/ods_change_detection/chair/remove_chair2.pcd", *rmc2, false);

//     transformation2(*rmc1);

//     // detect object
//     std::cout << "detect object" << std::endl;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj1 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj2 (new pcl::PointCloud<pcl::PointXYZRGB>);

//     clustering(*rmc1, *obj1, 0.02, 400, 2);
//     clustering(*rmc2, *obj2, 0.02, 400, 3);

//     pcl::io::savePCDFile ("/tmp/tms_ss_ods/ods_change_detection/chair/obj_cloud1.pcd", *obj1, false);

//     // segment matching
//     std::cout << "segment matching" << std::endl;
//     segment_matching();

//     // display result
//     std::cout << "display result" << std::endl;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr result1 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr result2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::copyPointCloud(*cloud1, *result1);
//     pcl::copyPointCloud(*cloud2, *result2);

//     for(int i=0;i<result1->points.size();i++){
//         result1->points[i].r = 255;
//         result1->points[i].g = 255;
//         result1->points[i].b = 255;
//     }

//     for(int i=0;i<result2->points.size();i++){
//         result2->points[i].r = 255;
//         result2->points[i].g = 255;
//         result2->points[i].b = 255;
//     }

//     FILE* fp;
//     if((fp = fopen("/tmp/tms_ss_ods/ods_change_detection/chair/change_position.txt", "w")) == NULL){
//         std::cout << "file open error!!" << std::endl;
//         return 0;
//     }

//     for(int i=0;i<Object_Map.size();i++){
//         std::cout << i << " → " << Object_Map[i].tf << std::endl;
//         std::stringstream ss2;
//         ss2 << "/tmp/tms_ss_ods/ods_change_detection/chair/object_" << i << ".pcd";
//         if(Object_Map[i].cloud_rgb.size() != 0){
//             pcl::io::savePCDFile (ss2.str (), Object_Map[i].cloud_rgb, true);
//         }
//         else{
//             std::cout << "no cloud_rgb data" << std::endl;
//         }
//         if(Object_Map[i].tf == 2){
//             fprintf(fp, "%d %lf %lf, %lf\n", Object_Map[i].tf, Object_Map[i].g.x, Object_Map[i].g.y,
//             Object_Map[i].g.z);
//             *result1 += Object_Map[i].cloud_rgb;
//         }
//         else if(Object_Map[i].tf == 3){
//             fprintf(fp, "%d %lf %lf, %lf\n", Object_Map[i].tf, Object_Map[i].g.x, Object_Map[i].g.y,
//             Object_Map[i].g.z);
//             *result2 += Object_Map[i].cloud_rgb;
//         }
//     }

//     if(result1->size() != 0){
//         pcl::io::savePCDFile ("/tmp/tms_ss_ods/ods_change_detection/chair/result1.pcd", *result1, true);
//     }
//     if(result2->size() != 0){
//         pcl::io::savePCDFile ("/tmp/tms_ss_ods/ods_change_detection/chair/result2.pcd", *result2, true);
//     }

//     fclose(fp);

//     for(int i=0;i<Object_Map.size();i++){
//         if((Object_Map[i].tf == 2) || (Object_Map[i].tf == 3)){
//             std::cout << Object_Map[i].g << std::endl;
//             geometry_msgs::Pose pose;
//             pose.position.x = Object_Map[i].g.x; pose.position.y = Object_Map[i].g.y; pose.position.z =
//             Object_Map[i].g.z;
//             pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0; pose.orientation.w = 0.0;
//             res.objects.poses.push_back(pose);
//         }
//     }

//     return true;
// }

// int main(int argc, char** argv){
//     printf("init\n");
//     ros::init(argc, argv, "ods_chair_model");
//     ros::NodeHandle n;

//     service = n.advertiseService("chg_dt_chair", change_detection);

//     commander_to_kinect_capture = n.serviceClient<tms_msg_ss::ods_pcd>("capture_cloud");

//     ros::spin();

//     return 0;
// }

#include <ods_chair.h>

void transformation(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& model)
{
  std::cout << "transformation" << std::endl;

  double camera_x = robot.x + sensor.x * cos(robot.theta) - sensor.y * sin(robot.theta);
  double camera_y = robot.y + sensor.x * sin(robot.theta) + sensor.y * cos(robot.theta);
  double camera_z = sensor.z;
  double camera_pitch = sensor.theta * (PI / 180.0);

  robot.theta *= PI / 180.0;
  chair.theta *= PI / 180.0;

  Eigen::Matrix4f t1(Eigen::Matrix4f::Identity());

  t1(0, 0) = sin(robot.theta);
  t1(0, 1) = -sin(camera_pitch) * cos(robot.theta);
  t1(0, 2) = cos(camera_pitch) * cos(robot.theta);
  t1(0, 3) = camera_x;
  t1(1, 0) = -cos(robot.theta);
  t1(1, 1) = -sin(camera_pitch) * sin(robot.theta);
  t1(1, 2) = cos(camera_pitch) * sin(robot.theta);
  t1(1, 3) = camera_y;
  t1(2, 0) = 0;
  t1(2, 1) = -cos(camera_pitch);
  t1(2, 2) = -sin(camera_pitch);
  t1(2, 3) = camera_z;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(cloud, cloud, t1);

  t1(0, 0) = cos(chair.theta);
  t1(0, 1) = sin(chair.theta);
  t1(0, 2) = 0;
  t1(0, 3) = chair.x;
  t1(1, 0) = -sin(chair.theta);
  t1(1, 1) = cos(chair.theta);
  t1(1, 2) = 0;
  t1(1, 3) = chair.y;
  t1(2, 0) = 0;
  t1(2, 1) = 0;
  t1(2, 2) = 1;
  t1(2, 3) = chair.z;
  t1(3, 0) = 0;
  t1(3, 1) = 0;
  t1(3, 2) = 0;
  t1(3, 3) = 1;

  pcl::transformPointCloud(model, model, t1);

  return;
}

void filtering(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& model)
{
  std::cout << "filtering" << std::endl;

  m_size.min_x = 100.0;
  m_size.max_x = -100.0;
  m_size.min_y = 100.0;
  m_size.max_y = -100.0;
  m_size.min_z = 100.0;
  m_size.max_z = -100.0;

  for (int i = 0; i < model.points.size(); i++)
  {
    if (model.points[i].x < m_size.min_x)
      m_size.min_x = model.points[i].x;
    if (model.points[i].x > m_size.max_x)
      m_size.max_x = model.points[i].x;
    if (model.points[i].y < m_size.min_y)
      m_size.min_y = model.points[i].y;
    if (model.points[i].y > m_size.max_y)
      m_size.max_y = model.points[i].y;
    if (model.points[i].z < m_size.min_z)
      m_size.min_z = model.points[i].z;
    if (model.points[i].z > m_size.max_z)
      m_size.max_z = model.points[i].z;
  }

  pcl::PassThrough< pcl::PointXYZRGB > pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(m_size.min_x - 0.2, m_size.max_x + 0.2);
  pass.filter(cloud);

  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("y");
  pass.setFilterLimits(m_size.min_y - 0.2, m_size.max_y + 0.2);
  pass.filter(cloud);

  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(m_size.min_z - 0.1, m_size.max_z + 0.3);
  pass.filter(cloud);

  return;
}

void downsampling(pcl::PointCloud< pcl::PointXYZRGB >& cloud, float th)
{
  pcl::VoxelGrid< pcl::PointXYZRGB > sor;
  sor.setInputCloud(cloud.makeShared());
  sor.setLeafSize(th, th, th);
  sor.filter(cloud);

  return;
}

pcl::PointXYZ g_pos(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  pcl::PointXYZ g;

  for (int i = 0; i < cloud.size(); i++)
  {
    g.x += cloud.points[i].x;
    g.y += cloud.points[i].y;
    g.z += cloud.points[i].z;
  }

  g.x = g.x / cloud.size();
  g.y = g.y / cloud.size();
  g.z = g.z / cloud.size();

  return g;
}

void registration(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& model,
                  pcl::PointCloud< pcl::PointXYZRGB >& cloud_out, pcl::PointCloud< pcl::PointXYZRGB >& tmp_rgb,
                  Eigen::Matrix4f& m)
{
  pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
  icp.setInputSource(cloud.makeShared());
  icp.setInputTarget(model.makeShared());
  icp.setUseReciprocalCorrespondences(true);
  icp.setMaxCorrespondenceDistance(0.05);
  pcl::PointCloud< pcl::PointXYZRGB > Final;
  icp.align(Final);
  m = icp.getFinalTransformation();

  pcl::transformPointCloud(cloud, cloud, m);
  pcl::transformPointCloud(tmp_rgb, tmp_rgb, m);
  pcl::copyPointCloud(model, cloud_out);
  cloud_out += cloud;

  return;
}

void make_chairvoxel(pcl::PointCloud< pcl::PointXYZRGB >& model)
{
  for (int k = 0; k < SIZE_Z; k++)
  {
    for (int j = 0; j < SIZE_Y; j++)
    {
      for (int i = 0; i < SIZE_X; i++)
      {
        C_voxel[i][j][k] = 0;
      }
    }
  }

  for (int i = 0; i < model.points.size(); i++)
  {
    int X = (100 / VOXEL_SIZE) * (model.points[i].x - m_size.min_x);
    int Y = (100 / VOXEL_SIZE) * (model.points[i].y - m_size.min_y);
    int Z = (100 / VOXEL_SIZE) * (model.points[i].z - m_size.min_z);

    if (!(C_voxel[X][Y][Z]))
    {
      C_voxel[X][Y][Z] = 1;
    }
  }

  return;
}

void remove_chair(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& rmc)
{
  for (int i = 0; i < cloud.points.size(); i++)
  {
    if (cloud.points[i].z <= 0.43)
      continue;
    int X = (100 / VOXEL_SIZE) * (cloud.points[i].x - m_size.min_x);
    int Y = (100 / VOXEL_SIZE) * (cloud.points[i].y - m_size.min_y);
    int Z = (100 / VOXEL_SIZE) * (cloud.points[i].z - m_size.min_z);

    if (((X < 0) || (SIZE_X <= X)) || ((Y < 0) || (SIZE_Y <= Y)) || ((Z < 2.0) || (SIZE_Z <= Z)))
      continue;

    if (!(C_voxel[X][Y][Z]))
      rmc.push_back(cloud.points[i]);
  }

  return;
}

//高さ平均値
double average(object_map m)
{
  int sum = 0;
  int number = 0;

  for (int i = 0; i < MAP_X; i++)
  {
    for (int j = 0; j < MAP_Y; j++)
    {
      if (m.map[i][j] != 0)
      {
        sum += m.map[i][j];
        number++;
      }
    }
  }

  return (double)sum / number;
}

//高さの分散
double variance(object_map m, double avg)
{
  int sum = 0;
  int number = 0;
  for (int i = 0; i < MAP_X; i++)
  {
    for (int j = 0; j < MAP_Y; j++)
    {
      sum += (m.map[i][j] - avg) * (m.map[i][j] - avg);
      number++;
    }
  }

  return (double)sum / number;
}

void make_elevation(object_map& m)
{
  std::cout << "make_elevation" << std::endl;
  double max_x = -10.0;
  double max_y = -10.0;
  double max_z = -10.0;
  double min_x = 10.0;
  double min_y = 10.0;
  double min_z = 10.0;
  int int_max_x = -100;
  int int_min_x = 100;
  int int_max_y = -100;
  int int_min_y = 100;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  *tmp_cloud = m.cloud_rgb;

  for (int i = 0; i < tmp_cloud->size(); i++)
  {
    if (tmp_cloud->points[i].x > max_x)
      max_x = tmp_cloud->points[i].x;
    if (tmp_cloud->points[i].x < min_x)
      min_x = tmp_cloud->points[i].x;
    tmp_cloud->points[i].x = (int)(tmp_cloud->points[i].x * 100);

    if (tmp_cloud->points[i].y > max_y)
      max_y = tmp_cloud->points[i].y;
    if (tmp_cloud->points[i].y < min_y)
      min_y = tmp_cloud->points[i].y;
    tmp_cloud->points[i].y = (int)(tmp_cloud->points[i].y * 100);

    if (tmp_cloud->points[i].z > max_z)
      max_z = tmp_cloud->points[i].z;
    if (tmp_cloud->points[i].z < min_z)
      min_z = tmp_cloud->points[i].z;
    tmp_cloud->points[i].z = (int)(tmp_cloud->points[i].z * 100);
  }

  m.max_x = max_x;
  m.min_x = min_x;
  m.max_y = max_y;
  m.min_y = min_y;
  m.max_z = max_z;
  m.min_z = min_z;
  int_min_x = 100 * min_x;
  int_min_y = 100 * min_y;
  int_max_x = 100 * max_x;
  int_max_y = 100 * max_y;

  downsampling(*tmp_cloud, 1.0f);

  m.size_x = int_max_x - int_min_x + 1;
  m.size_y = int_max_y - int_min_y + 1;

  std::cout << m.min_x << " " << m.max_x << std::endl;
  std::cout << m.min_y << " " << m.max_y << std::endl;
  std::cout << m.size_x << " " << m.size_y << std::endl;

  for (int i = 0; i < MAP_X; i++)
  {
    for (int j = 0; j < MAP_Y; j++)
    {
      m.map[i][j] = 0.0;
    }
  }
  for (int i = 0; i < tmp_cloud->size(); i++)
  {
    if (tmp_cloud->points[i].z >
        m.map[(int)tmp_cloud->points[i].x - int_min_x][(int)tmp_cloud->points[i].y - int_min_y])
      m.map[(int)tmp_cloud->points[i].x - int_min_x][(int)tmp_cloud->points[i].y - int_min_y] = tmp_cloud->points[i].z;
  }

  return;
}

void segmentation(pcl::PointCloud< pcl::PointXYZRGB >& cloud, double th, int c)
{
  std::cout << "segmentation" << std::endl;

  pcl::PointXYZ g;
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_rgb(new pcl::PointCloud< pcl::PointXYZRGB >);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGB >);
  tree->setInputCloud(cloud.makeShared());

  std::vector< pcl::PointIndices > cluster_indices;
  pcl::EuclideanClusterExtraction< pcl::PointXYZRGB > ec;
  ec.setClusterTolerance(th);
  ec.setMinClusterSize(200);
  ec.setMaxClusterSize(300000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud.makeShared());
  ec.extract(cluster_indices);

  std::cout << "クラスタリング開始" << std::endl;
  int m = 0;

  for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_cluster_rgb(new pcl::PointCloud< pcl::PointXYZRGB >);
    pcl::PointCloud< pcl::PointXYZHSV >::Ptr cloud_cluster_hsv(new pcl::PointCloud< pcl::PointXYZHSV >);
    pcl::PointXYZ g;

    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster_rgb->points.push_back(cloud.points[*pit]);
    }
    cloud_cluster_rgb->width = cloud_cluster_rgb->points.size();
    cloud_cluster_rgb->height = 1;
    cloud_cluster_rgb->is_dense = true;

    //クラスタ重心を求める
    g = g_pos(*cloud_cluster_rgb);

    if (((g.x < m_size.min_x) || (m_size.max_x < g.x)) || ((g.y < m_size.min_y) || (m_size.max_y < g.y)))
    {
      continue;
    }

    m++;

    if (1)
    {
      object_map map;
      map.cloud_rgb = *cloud_cluster_rgb;
      map.g = g;
      map.tf = c;
      make_elevation(map);

      // RGB -> HSV
      pcl::PointCloudXYZRGBtoXYZHSV(*cloud_cluster_rgb, *cloud_cluster_hsv);

      for (int i = 0; i < MAP_H; i++)
      {
        for (int j = 0; j < MAP_S; j++)
        {
          map.histogram[i][j] = 0.000;
        }
      }

      map.cloud_hsv = *cloud_cluster_hsv;
      // H-Sヒストグラムの作成
      for (int i = 0; i < cloud_cluster_hsv->points.size(); i++)
      {
        if (((int)(255 * cloud_cluster_hsv->points[i].h) / ((256 / MAP_H) * 360) >= 32) ||
            ((int)(255 * cloud_cluster_hsv->points[i].h) / ((256 / MAP_H) * 360) < 0))
          continue;
        if (((int)(255 * cloud_cluster_hsv->points[i].s) / (256 / MAP_H) >= 32) ||
            ((int)(255 * cloud_cluster_hsv->points[i].s) / (256 / MAP_H) < 0))
          continue;

        // 正規化のため,セグメントの点数で割る.
        map.histogram[(int)(255 * cloud_cluster_hsv->points[i].h) /
                      ((256 / MAP_H) * 360)][(int)(255 * cloud_cluster_hsv->points[i].s) / (256 / MAP_H)] +=
            1.000 / (float)cloud_cluster_hsv->points.size();
      }

      Object_Map.push_back(map);

      *tmp_rgb += *cloud_cluster_rgb;
    }
  }

  pcl::copyPointCloud(*tmp_rgb, cloud);

  return;
}

int compare_histogram(int a, int b)
{
  // 変数宣言
  float distance = 0.000;
  TYPE max1, max2;
  object_map object1, object2;

  // 初期化
  max1.value = -1;
  max2.value = -1;
  object1 = Object_Map[a];
  object2 = Object_Map[b];

  // 2つのヒストグラムの距離，それぞれの最頻値をとる箇所
  for (int i = 0; i < MAP_H; i++)
  {
    for (int j = 0; j < MAP_S; j++)
    {
      if (Bhattacharyya == 1)
        distance += sqrt(object1.histogram[i][j] * object2.histogram[i][j]);
      else
        distance += abs(object1.histogram[i][j] - object2.histogram[i][j]);

      if (object1.histogram[i][j] > max1.value)
      {
        max1.index_h = i;
        max1.index_s = j;
        max1.value = object1.histogram[i][j];
      }
      if (object2.histogram[i][j] > max2.value)
      {
        max2.index_h = i;
        max2.index_s = j;
        max2.value = object2.histogram[i][j];
      }
    }
  }

  std::cout << "compare Object_Map_" << a << " Object_Map_" << b << std::endl;
  std::cout << "max1 : " << max1.index_h << " " << max1.index_s << " " << max1.value << std::endl;
  std::cout << "max2 : " << max2.index_h << " " << max2.index_s << " " << max2.value << std::endl;
  std::cout << distance << std::endl;
  std::cout << std::endl;

  if (Bhattacharyya == 1)
  {
    if (distance >= 0.75)
      return 1;
    else
      return 0;
  }
  else
  {
    if (distance <= 1.00)
      return 1;
    else
      return 0;
  }

  return 0;
}

void segment_matching()
{
  pcl::PointXYZ g1, g2;

  for (int i = 0; i < Object_Map.size(); i++)
  {
    int c = 0;
    if (Object_Map[i].tf != 1)
      continue;
    g1 = Object_Map[i].g;

    for (int j = 0; j < Object_Map.size(); j++)
    {
      if (Object_Map[j].tf != 2)
        continue;
      g2 = Object_Map[j].g;

      double dis;
      dis = sqrt((g2.x - g1.x) * (g2.x - g1.x) + (g2.y - g1.y) * (g2.y - g1.y));
      if (dis < 0.07)
      {
        if ((abs(Object_Map[j].size_x - Object_Map[i].size_x) < 5) &&
            (abs(Object_Map[j].size_y - Object_Map[i].size_y) < 5))
        {
          double avg1, avg2;
          avg1 = average(Object_Map[j]);
          avg2 = average(Object_Map[i]);

          if (abs(avg1 - avg2) < 1.5)
          {
            if (abs(sqrt((double)variance(Object_Map[j], avg1)) - sqrt((double)variance(Object_Map[i], avg2))) < 10)
            {
              if (compare_histogram(i, j))
              {
                std::cout << "同じ物品" << i << " " << j << std::endl;
                Object_Map[i].tf = 0;
                Object_Map[j].tf = 0;
                c = 1;
                break;
              }
              else
                std::cout << i << " " << j << " 色が違う" << std::endl;
            }
            else
              std::cout << i << " " << j << " 高さの分散が違う " << abs(sqrt((double)variance(Object_Map[j], avg1)) -
                                                                        sqrt((double)variance(Object_Map[i], avg2)))
                        << std::endl;
          }
          else
            std::cout << i << " " << j << " 高さの平均が違う" << abs(avg1 - avg2) << " " << avg1 << " " << avg2
                      << std::endl;
        }
        else
          std::cout << i << " " << j << " 広さが違う" << abs(Object_Map[j].size_x - Object_Map[i].size_x) << " "
                    << abs(Object_Map[j].size_y - Object_Map[i].size_y) << std::endl;
      }
      else
        std::cout << i << " " << j << " 近くに物品なし" << dis << std::endl;
    }
    if (c == 0)
      std::cout << i << " 変化箇所" << std::endl;
  }

  return;
}

bool change_detection(tms_msg_ss::ods_furniture::Request& req, tms_msg_ss::ods_furniture::Response& res)
{
  //*******************************
  // initialize
  //*******************************
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr model(new pcl::PointCloud< pcl::PointXYZRGB >);

  std::cout << mkdir("/tmp/tms_ss_ods", S_IRUSR | S_IWUSR | S_IXUSR) << std::endl;
  std::cout << mkdir("/tmp/tms_ss_ods/ods_change_detection", S_IRUSR | S_IWUSR | S_IXUSR) << std::endl;
  std::cout << mkdir("/tmp/tms_ss_ods/ods_change_detection/chair", S_IRUSR | S_IWUSR | S_IXUSR) << std::endl;

  //*******************************
  // capture point cloud
  //*******************************
  tms_msg_ss::ods_pcd srv;

  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
  {
    pcl::fromROSMsg(srv.response.cloud, *cloud1);
  }
  pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/input.pcd", *cloud1);

  //    pcl::fromROSMsg (req.model, *model);

  // pcl::io::loadPCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/input.pcd", *cloud1);
  pcl::io::loadPCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/model_chair.pcd", *model);

  //*******************************
  // input PointCloud data
  //*******************************
  // chair.x = req.furniture.position.x/1000.0;
  // chair.y = req.furniture.position.y/1000.0;
  // chair.z = req.furniture.position.z/1000.0;
  // shelf.theta = req.furniture.orientation.z;
  // robot.x = req.robot.x/1000.0;
  // robot.y = req.robot.y/1000.0;
  // robot.theta = req.robot.theta;
  // sensor.x = req.sensor.position.x;
  // sensor.y = req.sensor.position.y;
  // sensor.z = req.sensor.position.z;
  // sensor.theta = req.sensor.orientation.y;
  chair.x = 2.5;
  chair.y = 2.5;
  chair.z = 0.0;
  chair.theta = 0.0;
  robot.x = 2.5;
  robot.y = 1.0;
  robot.theta = 90.0;
  sensor.x = 0.0;
  sensor.y = 0.0;
  sensor.z = 1.2;
  sensor.theta = 20.0;

  std::cout << robot.x << " " << robot.y << " " << robot.theta << std::endl;

  //***************************
  // transform input cloud
  //***************************
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tfm_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);

  transformation(*cloud1, *model);

  for (int i = 0; i < model->points.size(); i++)
  {
    model->points[i].r = 255;
    model->points[i].g = 0;
    model->points[i].b = 0;
  }

  *tfm_cloud = *model + *cloud1;

  if (!(tfm_cloud->points.size()))
  {
    std::cout << "tfm_cloud has no point" << std::endl;
    return false;
  }

  else
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/tfm_cloud1.pcd", *tfm_cloud);

  //***************************
  // filtering by using model
  //***************************
  std::cout << "filtering" << std::endl;

  filtering(*cloud1, *model);

  if (!(cloud1->points.size()))
  {
    std::cout << "filtered cloud has no point" << std::endl;
    return false;
  }

  else
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/filter.pcd", *cloud1);

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr dsp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::copyPointCloud(*cloud1, *dsp_cloud);

  downsampling(*dsp_cloud, 0.01);

  //*******************************
  // registration new Info's chair to memory Info's chair
  //*******************************
  std::cout << "registration chairs" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr rgs_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr view_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  Eigen::Matrix4f m;

  int n = 0;
  while (1)
  {
    registration(*dsp_cloud, *model, *rgs_cloud, *cloud1, m);

    if ((double)(m(0, 0) + m(1, 1) + m(2, 2) + m(3, 3)) >= 4)
    {
      if (n > 2)
        break;
    }
    n++;
  }

  pcl::copyPointCloud(*cloud1, *view_cloud);

  if (!(rgs_cloud->size() != 0))
  {
    std::cout << "registered cloud has no point" << std::endl;
    return false;
  }

  else
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/rgs_cloud.pcd", *rgs_cloud, true);

  //***************************
  // init C_voxel
  //***************************
  std::cout << "init chair_voxel" << std::endl;

  make_chairvoxel(*model);

  //***************************
  // remove chair
  //***************************
  std::cout << "remove chair" << std::endl;

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr rms_cloud1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr rms_cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);

  pcl::io::loadPCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/memory.pcd", *rms_cloud2);

  remove_chair(*cloud1, *rms_cloud1);

  if (!(rms_cloud1->size() != 0))
  {
    std::cout << "removed chair cloud has no point" << std::endl;
    return false;
  }

  else
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/remove_chair1.pcd", *rms_cloud1, false);

  //*******************************
  // segmentation
  //*******************************
  segmentation(*rms_cloud1, 0.015, 1);

  if (rms_cloud2->points.size() != 0)
    segmentation(*rms_cloud2, 0.015, 2);

  if (rms_cloud1->size() != 0)
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/memory.pcd", *rms_cloud1, true);
  if (rms_cloud2->size() != 0)
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/obj_cloud2.pcd", *rms_cloud2, true);

  //*******************************
  // detect change place
  //*******************************
  segment_matching();

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_rgb1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_rgb2(new pcl::PointCloud< pcl::PointXYZRGB >);

  //***************************************
  // rewrite Object data on TMS database
  //***************************************
  tms_msg_db::tmsdb_ods_object_data srv3;
  ros::Time time = ros::Time::now() + ros::Duration(9 * 60 * 60);

  int c = 0;
  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  for (int i = 0; i < Object_Map.size(); i++)
  {
    std::cout << i << " → " << Object_Map[i].tf << std::endl;

    if ((Object_Map[i].tf == 1) || (Object_Map[i].tf == 2))
    {
      std::stringstream ss;
      ss << "/tmp/tms_ss_ods/ods_change_detection/chair/result_rgb" << i << ".pcd";
      if (Object_Map[i].cloud_rgb.size() != 0)
      {
        pcl::io::savePCDFile(ss.str(), Object_Map[i].cloud_rgb, true);

        for (int j = 0; j < Object_Map[i].cloud_rgb.points.size(); j++)
        {
          Object_Map[i].cloud_rgb.points[j].r = colors[c % 6][0];
          Object_Map[i].cloud_rgb.points[j].g = colors[c % 6][1];
          Object_Map[i].cloud_rgb.points[j].b = colors[c % 6][2];
        }
        c++;
      }
      else
        std::cout << "no cloud_rgb data" << std::endl;

      if (Object_Map[i].tf == 1)
      {
        *tmp_rgb1 += Object_Map[i].cloud_rgb;

        tms_msg_db::tmsdb_data data;

        data.tMeasuredTime = time;
        data.iType = 5;
        data.iID = 53;
        data.fX = 1000.0 * (Object_Map[i].g.x - 1.0);
        data.fY = 1000.0 * (Object_Map[i].g.y - 1.5);
        data.fZ = 700.0;
        data.fTheta = 0.0;
        data.iPlace = 14;
        data.iState = 1;

        srv3.request.srvTMSInfo.push_back(data);

        geometry_msgs::Pose pose;
        pose.position.x = Object_Map[i].g.x;
        pose.position.y = Object_Map[i].g.y;
        pose.position.z = Object_Map[i].g.z;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        res.objects.poses.push_back(pose);

        std::cout << Object_Map[i].g.x << " " << Object_Map[i].g.y << " " << Object_Map[i].g.z << std::endl;
      }

      else if (Object_Map[i].tf == 2)
        *tmp_rgb2 += Object_Map[i].cloud_rgb;
    }
  }

  if (srv3.request.srvTMSInfo.size() != 0)
  {
    if (commander_to_ods_object_data.call(srv3))
      std::cout << "Rewrite TMS database!!" << std::endl;
  }

  if ((tmp_rgb1->size() == 0) && (tmp_rgb2->size() == 0))
  {
    std::cout << "No change on chair!!" << std::endl;
    return true;
  }

  if (tmp_rgb1->size() != 0)
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/result1.pcd", *tmp_rgb1, true);
  if (tmp_rgb2->size() != 0)
    pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/result2.pcd", *tmp_rgb2, true);

  *view_cloud += *tmp_rgb1;
  pcl::io::savePCDFile("/tmp/tms_ss_ods/ods_change_detection/chair/view_result.pcd", *view_cloud, true);

  //    pcl::toROSMsg (*cloud1, res.cloud);

  return true;
}

int main(int argc, char** argv)
{
  std::cout << "wait" << std::endl;
  ros::init(argc, argv, "ods_changedt_chair");
  ros::NodeHandle n;

  service = n.advertiseService("chg_dt_chair", change_detection);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("ods_capture");
  commander_to_ods_object_data = n.serviceClient< tms_msg_db::tmsdb_ods_object_data >("tmsdb_ods_object_data");

  ros::spin();

  return 0;
}
