#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
  //宣言
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);

  // small shelf
  /*for(double z=0.00;z<0.290;z+=0.005){
      for(double y=0.00;y<0.911;y+=0.005){
          for(double x=0.00;x<0.420;x+=0.005){
              if(z < 0.275){
                  if(((0.015<y)&&(y<0.303)) || ((0.319<y)&&(y<0.598)) || ((0.614<y)&&(y<0.896))){
                      if((0.015<x)&&(x<0.405)){
                          //std::cout << "not point!!" << std::endl;
                          continue;
                      }
                  }
              }
              pcl::PointXYZI tmp;
              tmp.x = x;
              tmp.y = y;
              tmp.z = z;
              tmp.intensity = 0;
              cloud->push_back(tmp);
          }
      }
  }*/

  // big shelf
  for (double z = 0.00; z < 1.812; z += 0.01)
  {
    for (double y = 0.00; y < 0.292; y += 0.01)
    {
      for (double x = 0.00; x < 0.802; x += 0.01)
      {
        if (y < 0.277)
        {
          if (((0.020 < x) && (x < 0.391)) || ((0.411 < x) && (x < 0.782)))
          {
            if ((0.030 < z) && (z < 0.264))
            {
              continue;
            }
            else if ((0.281 < z) && (z < 0.505))
            {
              continue;
            }
            else if ((0.522 < z) && (z < 0.820))
            {
              continue;
            }
            else if ((0.837 < z) && (z < 1.070))
            {
              continue;
            }
            else if ((1.087 < z) && (z < 1.329))
            {
              continue;
            }
            else if ((1.346 < z) && (z < 1.626))
            {
              continue;
            }
            else if ((1.646 < z) && (z < 1.771))
            {
              continue;
            }
          }
        }
        pcl::PointXYZ tmp;
        tmp.x = x;
        tmp.y = y;
        tmp.z = -z;
        // tmp.intensity = 0;
        cloud->push_back(tmp);
      }
    }
  }

  pcl::io::savePCDFile("catkin_ws/src/ros_tms/tms_ss/tms_ss_ods/ods_make_model/data/shelf/model.pcd", *cloud);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  while (!viewer.wasStopped())
  {
    viewer.showCloud(cloud);
  }
  return 0;
}
