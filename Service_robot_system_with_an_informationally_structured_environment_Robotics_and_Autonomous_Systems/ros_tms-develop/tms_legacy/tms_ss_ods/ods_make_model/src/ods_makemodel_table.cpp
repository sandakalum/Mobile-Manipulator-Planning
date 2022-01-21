#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#define TABLE 1

int main()
{
  //宣言
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);

  // table1
  if (TABLE == 1)
  {
    for (double z = 0.000; z <= 0.700; z += 0.01)
    {
      for (double y = 0.000; y <= 0.800; y += 0.01)
      {
        for (double x = 0.000; x <= 0.800; x += 0.01)
        {
          int i = 0;
          if (z <= 0.000)
            i = 1;

          else if (z <= 0.080)
          {
            if (((x < 0.005) || (0.795 < x)) && ((y < 0.005) || (0.795 < y)))
              continue;
            else if (((x < 0.010) || (0.790 < x)) && ((0.060 < y) && (y < 0.740)))
              continue;
            else if (((0.060 < x) && (x < 0.740)) && ((y < 0.010) || (0.790 < y)))
              continue;
            else if (((0.030 < x) && (x < 0.770)) && ((0.030 < y) && (y < 0.770)))
              continue;
            else
              i = 1;
          }

          else if (z < 0.690)
          {
            if (((0.005 <= x) && (x <= 0.060)) && ((0.005 <= y) && (y <= 0.060)))
              i = 1;
            else if (((0.005 <= x) && (x <= 0.060)) && ((0.740 <= y) && (y <= 0.795)))
              i = 1;
            else if (((0.740 <= x) && (x <= 0.795)) && ((0.005 <= y) && (y <= 0.060)))
              i = 1;
            else if (((0.740 <= x) && (x <= 0.795)) && ((0.740 <= y) && (y <= 0.795)))
              i = 1;
          }

          else
            i = 1;

          if (i == 1)
          {
            pcl::PointXYZ tmp;
            tmp.x = x;
            tmp.y = y;
            tmp.z = -z;
            cloud->push_back(tmp);
          }
        }
      }
    }
    pcl::io::savePCDFile("catkin_ws/src/ros_tms/tms_ss/tms_ss_ods/ods_make_model/data/table/model_table1.pcd", *cloud,
                         false);
  }

  // table2
  if (TABLE == 2)
  {
    for (double z = 0.000; z <= 0.750; z += 0.01)
    {
      for (double y = 0.000; y <= 0.610; y += 0.01)
      {
        for (double x = 0.000; x <= 1.440; x += 0.01)
        {
          int i = 0;
          if (z <= 0.020)
            i = 1;

          else if (z <= 0.090)
          {
            if (0.015 <= y)
              i = 1;
            else if ((0.025 <= x) && (x <= 1.415))
              i = 1;
          }

          else if (z <= 0.73)
          {
            if (0.015 <= y)
              if ((x <= 0.025) || (1.415 <= x))
                i = 1;
          }

          else
            i = 1;

          if (i == 1)
          {
            pcl::PointXYZ tmp;
            tmp.x = x;
            tmp.y = y;
            tmp.z = -z;
            cloud->push_back(tmp);
          }
        }
      }
    }
    pcl::io::savePCDFile("catkin_ws/src/ros_tms/tms_ss/tms_ss_ods/ods_make_model/data/table/model_table2.pcd", *cloud,
                         false);
  }

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  while (!viewer.wasStopped())
  {
    viewer.showCloud(cloud);
  }
  return 0;
}
