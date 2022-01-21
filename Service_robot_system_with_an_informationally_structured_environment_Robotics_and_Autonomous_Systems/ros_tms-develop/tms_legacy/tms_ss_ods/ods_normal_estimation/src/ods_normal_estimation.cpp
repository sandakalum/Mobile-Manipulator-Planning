#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointType;

int main()
{
  pcl::PointCloud< PointType >::Ptr cloud(new pcl::PointCloud< PointType >);

  pcl::io::loadPCDFile("src/ods_normal_estimation/data/ods_normal_estimation/sample.pcd", *cloud);

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  // pcl::visualization::PCLVisualizer viewer2("Cloud Viewer2");

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation< PointType, pcl::Normal > ne;
  ne.setInputCloud(cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is
  // given).
  pcl::search::KdTree< PointType >::Ptr tree(new pcl::search::KdTree< PointType >());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud< pcl::Normal >::Ptr cloud_normals(new pcl::PointCloud< pcl::Normal >);
  pcl::Normal n;
  double n_x = 0.0, n_y = 0.0, n_z = 0.0, n_c = 0.0;
  int count = 0;

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  // Compute the features
  ne.compute(*cloud_normals);

  for (int i = 0; i < cloud_normals->points.size(); i++)
  {
    if (cloud_normals->points[i].normal_x != 0.0)
    {
      n_x += cloud_normals->points[i].normal_x;
      n_y += cloud_normals->points[i].normal_y;
      n_z += cloud_normals->points[i].normal_z;
      n_c += cloud_normals->points[i].curvature;
      count++;
    }
    /*else{
        std::cout << "no normal vector" << std::endl;
    }*/
  }

  n.normal_x = n_x / (double)count;
  n.normal_y = n_y / (double)count;
  n.normal_z = n_z / (double)count;
  n.curvature = n_c / (double)count;

  std::cout << cloud->points.size() << " " << cloud_normals->size() << " " << count << std::endl;
  std::cout << cloud_normals->points[0] << std::endl;
  std::cout << cloud_normals->points[0].normal_x << " " << cloud_normals->points[0].normal_y << " "
            << cloud_normals->points[0].normal_z << " " << cloud_normals->points[0].curvature << std::endl;
  std::cout << cloud->points[0] << std::endl;
  std::cout << n.normal_x << " " << n.normal_y << " " << n.normal_z << " " << n.curvature << std::endl;

  while (!viewer.wasStopped())
  {
    viewer.addPointCloudNormals< PointType, pcl::Normal >(cloud, cloud_normals);
    // viewer2.addPointCloud<pcl::PointXYZRGB>(cloud);
    viewer.spinOnce(100);
    // viewer2.spinOnce(100);
  }

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}
