#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

void clustering(pcl::PointCloud< pcl::PointXYZRGB >& cloud, pcl::PointCloud< pcl::PointXYZRGB >& bed)
{
  std::cout << "clustering" << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree< pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGB >);
  tree->setInputCloud(cloud.makeShared());

  std::vector< pcl::PointIndices > cluster_indices;
  pcl::EuclideanClusterExtraction< pcl::PointXYZRGB > ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(1000);
  ec.setMaxClusterSize(3000000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud.makeShared());
  ec.extract(cluster_indices);

  int m = 0;
  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  for (std::vector< pcl::PointIndices >::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_cluster_rgb(new pcl::PointCloud< pcl::PointXYZRGB >);

    for (std::vector< int >::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster_rgb->points.push_back(cloud.points[*pit]);
    }
    cloud_cluster_rgb->width = cloud_cluster_rgb->points.size();
    cloud_cluster_rgb->height = 1;
    cloud_cluster_rgb->is_dense = true;

    for (int i = 0; i < cloud_cluster_rgb->points.size(); i++)
    {
      cloud_cluster_rgb->points[i].r = colors[m % 6][0];
      cloud_cluster_rgb->points[i].g = colors[m % 6][1];
      cloud_cluster_rgb->points[i].b = colors[m % 6][2];
    }

    m++;
    bed += *cloud_cluster_rgb;
  }

  return;
}

int main()
{
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_filtered(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_cluster(new pcl::PointCloud< pcl::PointXYZRGB >);

  // Fill in the cloud data
  pcl::io::loadPCDFile("src/ods_make_model/data/denoising/model_bed.pcd", *cloud);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval< pcl::PointXYZRGB > sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  pcl::io::savePCDFile("src/make_model/data/denoising/denoise.pcd", *cloud_filtered);

  sor.setNegative(true);
  sor.filter(*cloud_filtered);

  pcl::io::savePCDFile("src/make_model/data/denoising/noise.pcd", *cloud_filtered);

  clustering(*cloud, *cloud_cluster);

  pcl::io::savePCDFile("src/make_model/data/denoising/cluster.pcd", *cloud_cluster);

  return (0);
}
