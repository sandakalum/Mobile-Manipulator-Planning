#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <vector>

void passfilter(pcl::PointCloud< pcl::PointXYZRGB >& cloud)
{
  // Create the filtering object
  pcl::PassThrough< pcl::PointXYZRGB > pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1.0, 1.0);
  pass.filter(cloud);
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.0, 1.0);
  pass.filter(cloud);
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 2.0);
  pass.filter(cloud);

  return;
}

void sift3D_keypoints(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZI >& keypoints)
{
  pcl::SIFTKeypoint< pcl::PointXYZRGB, pcl::PointXYZI > sift3D;

  sift3D.setInputCloud(cloud.makeShared());
  sift3D.setScales(0.01, 5, 3);
  sift3D.setMinimumContrast(0.0);
  sift3D.compute(keypoints);

  return;
}

void harris3D_keypoints(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZI >& keypoints)
{
  std::cout << "detect harris3D keypoints" << std::endl;
  pcl::PointCloud< pcl::Normal >::Ptr cloud_normals(new pcl::PointCloud< pcl::Normal >);
  // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::HarrisKeypoint3D< pcl::PointXYZRGB, pcl::PointXYZI > harris3D;
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

  /*ne.setSearchMethod(tree);
  ne.setInputCloud(cloud.makeShared());
  ne.setRadiusSearch(0.03);
  ne.compute(*cloud_normals);*/

  harris3D.setInputCloud(cloud.makeShared());
  // harris3D.setNormals(cloud_normals);
  harris3D.setNonMaxSupression(true);
  // harris3D.setKSearch(1);
  harris3D.setRadius(0.03);
  harris3D.setRadiusSearch(0.03);
  harris3D.setMethod(pcl::HarrisKeypoint3D< pcl::PointXYZRGB, pcl::PointXYZI >::NOBLE);
  harris3D.compute(keypoints);

  return;
}

void FPFH_descriptors(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZI >& keypoints,
                      pcl::PointCloud< pcl::FPFHSignature33 >& features)
{
  pcl::Feature< pcl::PointXYZRGB, pcl::FPFHSignature33 >::Ptr feature_extractor(
      new pcl::FPFHEstimation< pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33 >);
  pcl::FeatureFromNormals< pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33 >::Ptr feature_from_normals =
      boost::dynamic_pointer_cast< pcl::FeatureFromNormals< pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33 > >(
          feature_extractor);
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr kpts(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::NormalEstimation< pcl::PointXYZRGB, pcl::Normal > ne;
  pcl::PointCloud< pcl::Normal >::Ptr normals(new pcl::PointCloud< pcl::Normal >);

  tree->setInputCloud(cloud.makeShared());

  feature_extractor->setSearchMethod(
      tree);  // pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>()));
  feature_extractor->setRadiusSearch(0.1);

  kpts->points.resize(keypoints.points.size());
  pcl::copyPointCloud(keypoints, *kpts);
  feature_extractor->setInputCloud(kpts);

  ne.setSearchMethod(tree);  // pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  ne.setRadiusSearch(0.1);
  ne.setInputCloud(kpts);
  ne.compute(*normals);

  feature_from_normals->setInputNormals(normals);
  feature_extractor->compute(features);

  return;
}

void find_correspondences(pcl::PointCloud< pcl::FPFHSignature33 >& features1,
                          pcl::PointCloud< pcl::FPFHSignature33 >& features2, std::vector< int >& correspondences)
{
  pcl::KdTreeFLANN< pcl::FPFHSignature33 > descriptor_kdtree;
  const int k = 1;
  std::vector< int > k_indices(k);
  std::vector< float > k_squared_distances(k);

  correspondences.resize(features1.size());

  // Use a kdTree to search for the nearest matches in feature space
  descriptor_kdtree.setInputCloud(features2.makeShared());

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  for (size_t i = 0; i < features1.size(); i++)
  {
    descriptor_kdtree.nearestKSearch(features1, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }

  return;
}

void filter_correspondences(pcl::PointCloud< pcl::PointXYZI >& keypoints1,
                            pcl::PointCloud< pcl::PointXYZI >& keypoints2, std::vector< int >& correspondences1,
                            std::vector< int >& correspondences2, pcl::CorrespondencesPtr correspondences)
{
  std::vector< std::pair< unsigned, unsigned > > corr;
  pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZI > rejector;

  for (unsigned i = 0; i < correspondences1.size(); ++i)
  {
    if (correspondences2[correspondences1[i]] == i)
    {
      corr.push_back(std::make_pair(i, correspondences1[i]));
    }
  }
  correspondences->resize(corr.size());

  for (unsigned i = 0; i < corr.size(); i++)
  {
    (*correspondences)[i].index_query = corr[i].first;
    (*correspondences)[i].index_match = corr[i].second;
  }

  rejector.setInputSource(keypoints1.makeShared());
  rejector.setInputTarget(keypoints2.makeShared());
  rejector.setInputCorrespondences(correspondences);
  rejector.getCorrespondences(*correspondences);

  return;
}

Eigen::Matrix4f initial_transformation(pcl::PointCloud< pcl::PointXYZI >& keypoints1,
                                       pcl::PointCloud< pcl::PointXYZI >& keypoints2,
                                       pcl::CorrespondencesPtr correspondences,
                                       pcl::PointCloud< pcl::PointXYZRGB >& cloud1,
                                       pcl::PointCloud< pcl::PointXYZRGB >& cloud2,
                                       pcl::PointCloud< pcl::PointXYZRGB >& transformed)
{
  Eigen::Matrix4f initial_transformation_matrix_ = Eigen::Matrix4f::Identity();
  pcl::registration::TransformationEstimation< pcl::PointXYZI, pcl::PointXYZI >::Ptr transformation_estimation(
      new pcl::registration::TransformationEstimationSVD< pcl::PointXYZI, pcl::PointXYZI >);

  transformation_estimation->estimateRigidTransformation(keypoints1, keypoints2, *correspondences,
                                                         initial_transformation_matrix_);
  pcl::transformPointCloud(cloud1, transformed, initial_transformation_matrix_);

  return initial_transformation_matrix_;
}

/*Eigen::Matrix4f final_transformation(pcl::PointCloud<pcl::PointXYZRGB>& transformed,
pcl::PointCloud<pcl::PointXYZRGB>& registered, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    Eigen::Matrix4f final_transformation_matrix_=Eigen::Matrix4f::Identity ();
    pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration (new
pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);

    return ;
}*/

int main(int argc, char** argv)
{
  // init
  std::cout << "init" << std::endl;
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);

  // load point cloud data
  std::cout << "load point cloud data" << std::endl;

  pcl::io::loadPCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/sample1.pcd", *cloud1);
  pcl::io::loadPCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/sample2.pcd", *cloud2);

  // filter input cloud
  std::cout << "filter input cloud" << std::endl;

  passfilter(*cloud1);
  passfilter(*cloud2);

  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/filter1.pcd", *cloud1);
  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/filter2.pcd", *cloud2);

  // compute sets of keypoints
  std::cout << "compute sets of keypoints" << std::endl;

  pcl::PointCloud< pcl::PointXYZI >::Ptr keypoints1(new pcl::PointCloud< pcl::PointXYZI >);
  pcl::PointCloud< pcl::PointXYZI >::Ptr keypoints2(new pcl::PointCloud< pcl::PointXYZI >);

  // sift3D_keypoints(*cloud1, *keypoints1);
  // sift3D_keypoints(*cloud2, *keypoints2);
  harris3D_keypoints(*cloud1, *keypoints1);
  harris3D_keypoints(*cloud2, *keypoints2);

  for (int i = 0; i < cloud1->points.size(); i++)
  {
    cloud1->points[i].r = 255;
    cloud1->points[i].g = 255;
    cloud1->points[i].b = 255;
  }

  for (int i = 0; i < cloud2->points.size(); i++)
  {
    cloud2->points[i].r = 255;
    cloud2->points[i].g = 255;
    cloud2->points[i].b = 255;
  }

  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/input1.pcd", *cloud1);
  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/input2.pcd", *cloud2);
  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/keypoints1.pcd",
                       *keypoints1);
  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/keypoints2.pcd",
                       *keypoints2);

  std::cout << "keypoints size : " << keypoints1->points.size() << std::endl;
  std::cout << "keypoints size : " << keypoints2->points.size() << std::endl;

  // compute feature descriptors
  std::cout << "compute feature descriptors" << std::endl;

  pcl::PointCloud< pcl::FPFHSignature33 >::Ptr features1(new pcl::PointCloud< pcl::FPFHSignature33 >);
  pcl::PointCloud< pcl::FPFHSignature33 >::Ptr features2(new pcl::PointCloud< pcl::FPFHSignature33 >);

  FPFH_descriptors(*cloud1, *keypoints1, *features1);
  FPFH_descriptors(*cloud2, *keypoints2, *features2);

  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/features1.pcd",
                       *features1);
  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/features2.pcd",
                       *features2);

  // match features to find correspondences
  std::cout << "match features to find correspondences" << std::endl;

  std::vector< int > correspondences1;
  std::vector< int > correspondences2;

  find_correspondences(*features1, *features2, correspondences1);
  find_correspondences(*features2, *features1, correspondences2);

  // filter correspondences
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

  filter_correspondences(*keypoints1, *keypoints2, correspondences1, correspondences2, correspondences);

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr corr_cloud1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr corr_cloud2(new pcl::PointCloud< pcl::PointXYZRGB >);

  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  for (int i = 0; i < correspondences->size(); i++)
  {
    if (i >= 6)
      break;
    pcl::PointXYZRGB p1, p2;
    p1.x = keypoints1->points[(*correspondences)[i].index_query].x;
    p1.y = keypoints1->points[(*correspondences)[i].index_query].y;
    p1.z = keypoints1->points[(*correspondences)[i].index_query].z;
    p1.r = colors[i][0];
    p1.g = colors[i][1];
    p1.b = colors[i][2];
    p2.x = keypoints2->points[(*correspondences)[i].index_match].x;
    p2.y = keypoints2->points[(*correspondences)[i].index_match].y;
    p2.z = keypoints2->points[(*correspondences)[i].index_match].z;
    p2.r = colors[i][0];
    p2.g = colors[i][1];
    p2.b = colors[i][2];

    corr_cloud1->push_back(p1);
    corr_cloud2->push_back(p2);
  }

  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/corr_cloud1.pcd",
                       *corr_cloud1);
  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/corr_cloud2.pcd",
                       *corr_cloud2);

  // Initial transformation
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformed(new pcl::PointCloud< pcl::PointXYZRGB >);
  Eigen::Matrix4f initial_transformation_matrix =
      initial_transformation(*keypoints1, *keypoints2, correspondences, *cloud1, *cloud2, *transformed);

  *transformed += *cloud2;

  pcl::io::savePCDFile("src/ods_correspondence_estimation/data/ods_correspondence_estimation/transformed.pcd",
                       *transformed);

  // Final transformation
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Eigen::Matrix4f final_transformation_matrix = final_transformation(*transformed , *registered, *cloud2);

  return 0;
}
