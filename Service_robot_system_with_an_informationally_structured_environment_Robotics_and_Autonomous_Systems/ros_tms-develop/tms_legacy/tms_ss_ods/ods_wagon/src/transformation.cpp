#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#define PI 3.1415926535

int main()
{
  pcl::PointXYZ cloud, point;

  cloud.x = 1.0;
  cloud.y = 1.0;
  cloud.z = 1.0;

  double robot_x = 1.0;
  double robot_y = 1.0;
  double robot_theta = 0.0 * PI / 180.0;

  double C_OFFSET_X = 0.1;
  double C_OFFSET_Y = 0.05;
  double C_OFFSET_Z = 1.0;
  double C_OFFSET_PITCH = 45.0;

  double camera_x = robot_x + C_OFFSET_X * cos(robot_theta) - C_OFFSET_Y * sin(robot_theta);
  double camera_y = robot_y + C_OFFSET_X * sin(robot_theta) + C_OFFSET_Y * cos(robot_theta);
  double camera_z = C_OFFSET_Z;
  double camera_pitch = C_OFFSET_PITCH * (PI / 180.0);

  std::cout << "before : " << cloud.x << " " << cloud.y << " " << cloud.z << std::endl;
  std::cout << "camera : " << camera_x << " " << camera_y << " " << camera_z << " " << camera_pitch << std::endl;

  Eigen::Affine3f t(Eigen::Affine3f::Identity());

  t(0, 0) = sin(robot_theta);
  t(0, 1) = -sin(camera_pitch) * cos(robot_theta);
  t(0, 2) = cos(camera_pitch) * cos(robot_theta);
  t(1, 0) = -cos(robot_theta);
  t(1, 1) = -sin(camera_pitch) * sin(robot_theta);
  t(1, 2) = cos(camera_pitch) * sin(robot_theta);
  t(2, 0) = 0;
  t(2, 1) = -cos(camera_pitch);
  t(2, 2) = -sin(camera_pitch);
  //    t(0,0) = sin(robot_theta);    t(0,1) = sin(camera_pitch)*cos(robot_theta);    t(0,2) =
  //    cos(camera_pitch)*cos(robot_theta);
  //    t(1,0) = cos(robot_theta);   t(1,1) = -sin(camera_pitch)*sin(robot_theta);   t(1,2) =
  //    -cos(camera_pitch)*sin(robot_theta);
  //    t(2,0) = 0;                   t(2,1) = cos(camera_pitch);                    t(2,2) = -sin(camera_pitch);

  point = pcl::transformPoint(cloud, t);

  point.x += camera_x;
  point.y += camera_y;
  point.z += camera_z;

  std::cout << "after : " << point.x << " " << point.y << " " << point.z << std::endl;

  return 0;
}
