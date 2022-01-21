#include <math.h>
#include <limits.h>

#include <cstdint>
#include <map>

// C++11
#include <memory>

#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "myutility.h"
#include "MyCalibration.h"

#define INVALID_PIXEL_VALUE 0

extern bool g_DebugFlg;

Eigen::Vector2i g_points[4];
int g_picking_counter = -1;
Eigen::Vector2i g_picked_points[CORRESPOND_POINTS];

void mouseCallback(int event, int x, int y, int flag, void*)
{
  static int point = 0;
  std::string desc;
  switch (event)
  {
    case cv::EVENT_LBUTTONUP:
      if (flag & cv::EVENT_FLAG_LBUTTON)
      {
        g_points[point][0] = x;
        g_points[point][1] = y;
        point++;
        if (point == 4)
        {
          point = 0;
        }
      }
      break;
    case cv::EVENT_RBUTTONUP:
      if (g_picking_counter >= 0 && (flag & cv::EVENT_FLAG_RBUTTON))
      {
        g_picked_points[g_picking_counter][0] = x;
        g_picked_points[g_picking_counter][1] = y;
        std::cout << "Picked [" << g_picking_counter << "]: "
                  << "(" << x << ", " << y << ")" << std::endl;
        g_picking_counter++;
        if (g_picking_counter >= CORRESPOND_POINTS)
        {
          g_picking_counter = -1;
        }
      }
      break;
  }
  return;
}

int MyCalibration::initialize(pcl::visualization::CloudViewer& viewer, uint8_t* depth_frame_region,
                              uint8_t* color_frame_region)
{
  // correct_mirroring << -1,0,0,0,1,0,0,0,1;
  {  // Setting streams
    if (depth_frame_region != NULL)
    {
      depth_frame = depth_frame_region;
    }
    else
    {
      std::cout << "Failed to set pointer of depth frame." << std::endl;
      return -1;
    }
    if (color_frame_region != NULL)
    {
      color_frame = color_frame_region;
    }
    else
    {
      std::cout << "Failed to set pointer of color frame." << std::endl;
      return -1;
    }
  }
  cloud->width = CAMERA_RESOLUTION_X;
  cloud->height = CAMERA_RESOLUTION_Y;
  cloud->is_dense = false;
  cloud->points.resize(cloud->height * cloud->width);
  this->viewer = &viewer;

  int i;
  g_picking_counter = 0;
  for (i = 0; i < CORRESPOND_POINTS; i++)
  {
    g_picked_points[i] = Eigen::Vector2i::Zero();
  }
  return 0;
}

int MyCalibration::extractPlanePoints()
{
  int i, j;
  std::vector< Eigen::Vector3f > all_points;

  // Sort points
  int min_cross;
  int tmp_cross;
  int min_cross_num;
  Eigen::Vector2i tmp_vec[3];
  for (i = 0; i < 4; i++)
  {
    min_cross_num = i;
    min_cross = INT_MAX;
    for (j = i; j < 4; j++)
    {
      if (i == 0)
      {
        tmp_vec[0] = g_points[j];
        tmp_vec[1] = g_points[i];
      }
      else
      {
        tmp_vec[0] = g_points[j] - g_points[i - 1];
        tmp_vec[1] = g_points[i] - g_points[i - 1];
      }

      tmp_cross = tmp_vec[0][0] * tmp_vec[1][1] - tmp_vec[0][1] * tmp_vec[1][0];
      if (tmp_cross < min_cross)
      {
        min_cross = tmp_cross;
        min_cross_num = j;
      }
    }
    tmp_vec[3] = g_points[i];
    g_points[i] = g_points[min_cross_num];
    g_points[min_cross_num] = tmp_vec[3];
  }

  // Detect closed region
  tmp_vec[0] = Eigen::Vector2i::Zero(2);
  for (i = 0; i < 4; i++)
  {
    tmp_vec[0] += g_points[i] / 4;
  }

  // Setting points
  int check;
  bool check_flg;
  std::vector< char > points_attribute;
  Eigen::Vector3f tmp_vec3[1];

  points_on_plane.clear();
  all_points.clear();
  all_points.resize(CAMERA_RESOLUTION_X * CAMERA_RESOLUTION_Y, Eigen::Vector3f(0, 0, 0));
  other_points.clear();
  points_attribute.clear();
  points_attribute.resize(CAMERA_RESOLUTION_X * CAMERA_RESOLUTION_Y, (char)0x00);

  // Get static point cloud
  for (i = 0; i < CAMERA_RESOLUTION_Y; i++)
  {
    for (j = 0; j < CAMERA_RESOLUTION_X; j++)
    {
      float* p;
      p = (float*)&points->data[(i * CAMERA_RESOLUTION_X + j) * sizeof(float) * 4];
      all_points[i * CAMERA_RESOLUTION_X + j][0] = *p * 1000;
      p++;
      all_points[i * CAMERA_RESOLUTION_X + j][1] = *p * 1000;
      p++;
      all_points[i * CAMERA_RESOLUTION_X + j][2] = *p * 1000;
    }
  }

  std::vector< Eigen::Vector2i > loop_stack;
  loop_stack.clear();
  loop_stack.push_back(Eigen::Vector2i(tmp_vec[0][0], tmp_vec[0][1]));
  while (!loop_stack.empty())
  {  // Detecting points on plane : 0x01
    tmp_vec[0] = loop_stack.back();
    loop_stack.pop_back();
    if (points_attribute[tmp_vec[0][0] + tmp_vec[0][1] * CAMERA_RESOLUTION_X] & 0x01)
    {
      continue;
    }
    for (i = 0; i < 4; i++)
    {
      check = 0;
      check_flg = true;
      tmp_vec[1] = tmp_vec[0] - g_points[i];
      if (i == 3)
      {
        tmp_vec[2] = g_points[0] - g_points[i];
      }
      else
      {
        tmp_vec[2] = g_points[i + 1] - g_points[i];
      }
      check = (tmp_vec[2][0] * tmp_vec[1][1] - tmp_vec[2][1] * tmp_vec[1][0]);
      if (check > 0)
      {
        check_flg = false;
        break;
      }
    }
    if (check_flg)
    {
      // This point is on the plane. (because user chooses a region of a plane)
      points_attribute[tmp_vec[0][0] + tmp_vec[0][1] * CAMERA_RESOLUTION_X] |= 0x01;
      if (!(points_attribute[tmp_vec[0][0] + 1 + tmp_vec[0][1] * CAMERA_RESOLUTION_X] & 0x01))
      {
        loop_stack.push_back(Eigen::Vector2i(tmp_vec[0][0] + 1, tmp_vec[0][1]));
      }
      if (!(points_attribute[tmp_vec[0][0] - 1 + tmp_vec[0][1] * CAMERA_RESOLUTION_X] & 0x01))
      {
        loop_stack.push_back(Eigen::Vector2i(tmp_vec[0][0] - 1, tmp_vec[0][1]));
      }
      if (!(points_attribute[tmp_vec[0][0] + (tmp_vec[0][1] + 1) * CAMERA_RESOLUTION_X] & 0x01))
      {
        loop_stack.push_back(Eigen::Vector2i(tmp_vec[0][0], tmp_vec[0][1] + 1));
      }
      if (!(points_attribute[tmp_vec[0][0] + (tmp_vec[0][1] - 1) * CAMERA_RESOLUTION_X] & 0x01))
      {
        loop_stack.push_back(Eigen::Vector2i(tmp_vec[0][0], tmp_vec[0][1] - 1));
      }
    }
  }

  const int radius = 3;
  int k;
  int now;
  for (k = 0; k < CORRESPOND_POINTS; k++)
  {  // Detecting points around picked points : 0x02
    unsigned int _cnt = 0;
    picked_points[k] = Eigen::Vector3f::Zero();
    for (i = g_picked_points[k][1] - radius; i <= g_picked_points[k][1] + radius; i++)
    {
      for (j = -sqrt(pow(radius, 2.0) - pow((i - g_picked_points[k][1]), 2.0)) + g_picked_points[k][0];
           j <= sqrt(pow(radius, 2.0) - pow((i - g_picked_points[k][1]), 2.0)) + g_picked_points[k][0]; j++)
      {
        if (i >= 0 && i < CAMERA_RESOLUTION_Y && j >= 0 && j < CAMERA_RESOLUTION_X)
        {  // picked_points_around
          now = i * CAMERA_RESOLUTION_X + j;
          points_attribute[now] |= 0x02;
          picked_points[k] += all_points[now];
          _cnt++;
        }
      }
    }
    picked_points[k] = picked_points[k] / (double)_cnt;
  }

  for (i = 0; i < CAMERA_RESOLUTION_Y; i++)
  {  // Add to vector as attribute
    for (j = 0; j < CAMERA_RESOLUTION_X; j++)
    {
      now = (i * CAMERA_RESOLUTION_X + j);
      if (points_attribute[now] == 0x00)
      {
        other_points.push_back(&all_points[now]);
      }
      if (points_attribute[now] & 0x01)
      {
        points_on_plane.push_back(&all_points[now]);
        if ((i + j) % 2 == 0)
        {
          image.at< uchar >(i, j) = 255;
        }
      }
      if (points_attribute[now] & 0x02)
      {
        picked_points_around.push_back(&all_points[now]);
      }
    }
  }

  std::cout << "Extracted number of points: " << points_on_plane.size() << std::endl;

  cv::imshow("Depth image", image);

  return 0;
}

int MyCalibration::viewPoints()
{
  int i;
  std::vector< Eigen::Vector3f* >::iterator it;
  Eigen::Vector3f* ptmp;
  Eigen::Vector3f tmp;

  cloud->clear();
  pcl::PointXYZRGB point;

  for (it = other_points.begin(); it != other_points.end(); it++)
  {
    ptmp = *it;
    tmp = *ptmp;
    point.x = tmp[0] / 1000.0;
    point.y = tmp[1] / 1000.0;
    point.z = tmp[2] / 1000.0;
    point.r = 255;
    point.g = tmp[2] / 5;
    point.b = 255;
    if (point.z < 10.0)
    {
      cloud->push_back(point);
    }
  }
  for (it = points_on_plane.begin(); it != points_on_plane.end(); it++)
  {
    ptmp = *it;
    tmp = *ptmp;
    point.x = tmp[0] / 1000.0;
    point.y = tmp[1] / 1000.0;
    point.z = tmp[2] / 1000.0;
    point.r = 255;
    point.g = 255;
    point.b = tmp[2] / 5;
    cloud->push_back(point);
  }
#ifdef _DEBUG
  for (it = picked_points_around.begin(); it != picked_points_around.end(); it++)
  {  // Draw around of picked points
    ptmp = *it;
    tmp = *ptmp;
    point.x = tmp[0] / 1000.0;
    point.y = tmp[1] / 1000.0;
    point.z = tmp[2] / 1000.0;
    point.r = tmp[2] / 5;
    point.g = 255;
    point.b = 255;
    cloud->push_back(point);
  }
#endif  // _DEBUG
  for (i = 0; i < CORRESPOND_POINTS; i++)
  {  // Draw picked points
    point.x = picked_points[i][0] / 1000.0;
    point.y = picked_points[i][1] / 1000.0;
    point.z = picked_points[i][2] / 1000.0;
    point.r = picked_points[i][2] / 5;
    point.g = picked_points[i][2] / 5;
    point.b = 255;
    cloud->push_back(point);
  }
  viewer->showCloud(cloud);
  return 0;
}

inline double _E(double phi_x, double phi_y, double phi_z, double t_x, double t_y, double t_z,
                 const Eigen::Vector3f* X_w, const Eigen::Vector3f* X_c)
{
  int i;
  double ret;
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(phi_z, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(phi_y, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(phi_x, Eigen::Vector3d::UnitX());
  ret = 0.0;
  for (i = 0; i < CORRESPOND_POINTS; i++)
  {
    Eigen::Vector3d tmp;
    tmp = X_w[i].cast< double >() - (rot * (X_c[i].cast< double >()) + Eigen::Vector3d(t_x, t_y, t_z));
    ret += pow(tmp.norm(), 2.0);
  }
  return ret;
}

inline int find_corner(const cv::Mat& image, const cv::Size pattern_size, std::vector< cv::Point2f >& corners,
                       bool& pattern_found)
{
  cv::Mat gray(image.rows, image.cols, CV_8UC1);
  switch (image.type())
  {
    case CV_8UC1:
      gray = image;
      break;
    case CV_8UC3:
      cv::cvtColor(image, gray, CV_BGR2GRAY);
      break;
    default:
      std::cerr << "Unexpected image type." << std::endl;
      return -1;
  }
  pattern_found =
      cv::findChessboardCorners(gray, pattern_size, corners,
                                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  if (pattern_found)
  {
    cv::cornerSubPix(gray, corners, cv::Size(15, 15), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 15, 0.01));
  }
  return 0;
}

pcl::PointXYZRGB _s, _ex, _ey, _ez;
void viewerDrawAxis(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.removeShape("axis-x");
  viewer.removeShape("axis-y");
  viewer.removeShape("axis-z");
  viewer.addLine< pcl::PointXYZRGB >(_s, _ex, 255, 0, 0, "axis-x");
  viewer.addLine< pcl::PointXYZRGB >(_s, _ey, 0, 255, 0, "axis-y");
  viewer.addLine< pcl::PointXYZRGB >(_s, _ez, 0, 0, 255, "axis-z");
  return;
}

extern double d_roll, d_pitch, d_yaw;
extern double d_tx, d_ty, d_tz;
int MyCalibration::calcurateExtrinsicParameters(double convertion_th, int convertion_method, const float pattern_width)
{
  int i, j, k;
  double t_x, t_y, t_z;
  double phi_x, phi_y, phi_z;
  Eigen::Vector3d avg_vec;
  Eigen::Matrix3d cov_mat;

  Eigen::Vector3d tmp;

  avg_vec = Eigen::Vector3d::Zero();
  cov_mat = Eigen::Matrix3d::Zero();

  Eigen::Vector3d ax[3];
  Eigen::Vector3d ax_cam[3];
  ax_cam[0] = Eigen::Vector3d::UnitX();
  ax_cam[1] = Eigen::Vector3d::UnitY();
  ax_cam[2] = Eigen::Vector3d::UnitZ();

  {  // calcuration of average
    for (i = 0; i < 3; i++)
    {
      for (k = 0; k < points_on_plane.size(); k++)
      {
        tmp = points_on_plane[k]->cast< double >();
        avg_vec[i] += tmp[i] / points_on_plane.size();
      }
    }

    // calcuration of covariance
    for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 3; j++)
      {
        for (k = 0; k < points_on_plane.size(); k++)
        {
          tmp = points_on_plane[k]->cast< double >();
          cov_mat(i, j) += ((tmp[i] - avg_vec[i]) * (tmp[j] - avg_vec[j])) / points_on_plane.size();
        }
      }
    }
  }

  {  // Calcurate Z-axis
    Eigen::EigenSolver< Eigen::Matrix3d > solver(cov_mat);
    double eigen_min = std::numeric_limits< double >::infinity();
    for (i = 0; i < 3; i++)
    {
      if (solver.eigenvalues()(i).real() < eigen_min)
      {
        j = i;
        eigen_min = solver.eigenvalues()(i).real();
      }
    }
    Eigen::Matrix3cd axis_world;
    axis_world = solver.eigenvectors();

    ax[2][0] = axis_world(0, j).real();
    ax[2][1] = axis_world(1, j).real();
    ax[2][2] = axis_world(2, j).real();
    if (ax[2].dot(ax_cam[2]) > 0)
    {
      ax[2] = -ax[2];
    }
    ax[2] = ax[2] / ax[2].norm();
  }

  // Estimate other 4 parameters
  {  // Set world points
    for (int x = 0; x < 5; x++)
    {
      for (int y = 0; y < 6; y++)
      {
        world_points[x * 6 + y] = Eigen::Vector3f(x * pattern_width, y * pattern_width, 0.0);
      }
    }
  }

  // For Debug ===
  if (g_DebugFlg)
  {
    Eigen::Matrix3d _debug_mat = Eigen::Matrix3d::Zero();
    _debug_mat = Eigen::AngleAxisd(d_yaw / 180 * M_PI, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(d_pitch / 180 * M_PI, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(d_roll / 180 * M_PI, Eigen::Vector3d::UnitX());

    Eigen::Vector3d _debug_vec = Eigen::Vector3d(d_tx, d_ty, d_tz);

    for (i = 0; i < CORRESPOND_POINTS; i++)
    {
      picked_points[i] = _debug_mat.cast< float >().inverse() * (world_points[i] - _debug_vec.cast< float >());
      std::cout << "i: " << i << "\t" << picked_points[i][0] << ", " << picked_points[i][1] << ", "
                << picked_points[i][2] << std::endl;
    }
    ax[2] = _debug_mat.inverse() * Eigen::Vector3d::UnitZ();
  }
  // === For Debug

  double n;
  phi_x = atan2(ax[2](1), ax[2](2));
  phi_y = atan2(-ax[2](0), sqrt(pow(ax[2][1], 2.0) + pow(ax[2][2], 2.0)));
  Eigen::Matrix3d rot_rp;
  rot_rp = Eigen::AngleAxisd(phi_y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(phi_x, Eigen::Vector3d::UnitX());
  Eigen::Vector3d rotatedRP_point;

  t_z = 0.0;
  for (i = 0; i < CORRESPOND_POINTS; i++)
  {
    rotatedRP_point = rot_rp * picked_points[i].cast< double >();
    // t_z
    t_z += (world_points[i][2] - rotatedRP_point[2]) / CORRESPOND_POINTS;
  }

  /* Convertion calcuration */
  phi_z = 0.0;
  t_x = 0.0;
  t_y = 0.0;
  Eigen::Vector3d grad_E = Eigen::Vector3d::Ones();

  unsigned long long int cnt;
  // Steepest descent method
  if (convertion_method == 1)
  {
    const double step = 0.00000002;
    const double grad_th = convertion_th;
    cnt = 0;
    while (grad_E.norm() > grad_th)
    {
      Eigen::Matrix3d rot;
      rot = Eigen::AngleAxisd(phi_z, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(phi_y, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(phi_x, Eigen::Vector3d::UnitX());

      grad_E = Eigen::Vector3d::Zero();
      for (i = 0; i < CORRESPOND_POINTS; i++)
      {
        Eigen::Vector3d rotated_point = rot * (picked_points[i].cast< double >());
        Eigen::Vector3d world_point = world_points[i].cast< double >();
        grad_E[0] += 2.0 * ((world_point[0] - t_x) * rotated_point[1] - (world_point[1] - t_y) * rotated_point[0]);
        grad_E[1] -= 2.0 * (world_point[0] - (rotated_point[0] + t_x));
        grad_E[2] -= 2.0 * (world_point[1] - (rotated_point[1] + t_y));
      }
      phi_z -= step * grad_E[0];
      t_x -= step * grad_E[1];
      t_y -= step * grad_E[2];
      if (phi_z > M_PI)
      {
        phi_z -= 2.0 * M_PI;
      }
      if (phi_z <= -M_PI)
      {
        phi_z += 2.0 * M_PI;
      }
      cnt++;

      std::cout << "k: " << cnt << "---" << std::endl
                << "phi_z: " << phi_z << "\t"
                << "t_x: " << t_x << "\t"
                << "t_y: " << t_y << std::endl
                << "E: " << _E(phi_x, phi_y, phi_z, t_x, t_y, t_z, world_points, picked_points) << "\t"
                << "grad_E: " << grad_E[0] << "\t" << grad_E[1] << "\t" << grad_E[2] << std::endl
                << "|grad_E|: " << grad_E.norm() << std::endl
                << std::endl;
    }
  }
  // Perturbation method
  else if (convertion_method == 2)
  {
    double E = _E(phi_x, phi_y, phi_z, t_x, t_y, t_z, world_points, picked_points);
    const double _th = convertion_th;
    const double alpha = 0.001;
    const double beta = 0.01;
    const double gamma = 0.01;
    double E_prev;
    cnt = 0;
    do
    {
      E_prev = E;

      double min;
      min = std::numeric_limits< double >::infinity();
      for (i = 0; i < 6; i++)
      {
        switch (i)
        {
          case 0:
            E = _E(phi_x, phi_y, phi_z + alpha, t_x, t_y, t_z, world_points, picked_points);
            if (min > E)
            {
              min = E;
              j = i;
            }
            break;
          case 1:
            E = _E(phi_x, phi_y, phi_z - alpha, t_x, t_y, t_z, world_points, picked_points);
            if (min > E)
            {
              min = E;
              j = i;
            }
            break;
          case 2:
            E = _E(phi_x, phi_y, phi_z, t_x + beta, t_y, t_z, world_points, picked_points);
            if (min > E)
            {
              min = E;
              j = i;
            }
            break;
          case 3:
            E = _E(phi_x, phi_y, phi_z, t_x - beta, t_y, t_z, world_points, picked_points);
            if (min > E)
            {
              min = E;
              j = i;
            }
            break;
          case 4:
            E = _E(phi_x, phi_y, phi_z, t_x, t_y + gamma, t_z, world_points, picked_points);
            if (min > E)
            {
              min = E;
              j = i;
            }
            break;
          case 5:
            E = _E(phi_x, phi_y, phi_z, t_x, t_y - gamma, t_z, world_points, picked_points);
            if (min > E)
            {
              min = E;
              j = i;
            }
            break;
        }
        E = min;
      }
      switch (j)
      {
        case 0:
          phi_z += alpha;
          if (phi_z > M_PI)
          {
            phi_z -= 2.0 * M_PI;
          }
          break;
        case 1:
          phi_z -= alpha;
          if (phi_z < -M_PI)
          {
            phi_z += 2.0 * M_PI;
          }
          break;
        case 2:
          t_x += beta;
          break;
        case 3:
          t_x -= beta;
          break;
        case 4:
          t_y += gamma;
          break;
        case 5:
          t_y -= gamma;
          break;
      }
      cnt++;
      if (cnt % 1000 == 0)
      {
        std::cout << "delta_E " << cnt << "-> " << E_prev - E << std::endl;
      }
    } while (E_prev - E > _th);
  }
  else
  {
    std::cerr << "Error::Requrered unexpected method." << std::endl;
    return 1;
  }

  std::cout << "Tried " << cnt << " times for convertion calcuration of extrinsic parameters" << std::endl;
  std::cout << "rpy[deg]: " << phi_x * 180.0 / M_PI << ", " << phi_y * 180.0 / M_PI << ", " << phi_z * 180.0 / M_PI
            << std::endl;

  std::cout << "t[mm]: " << t_x << ", " << t_y << ", " << t_z << std::endl;

  {  // Draw axis on cloud viewer for certification
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    rotation =
        Eigen::AngleAxisd(phi_z, ax_cam[2]) * Eigen::AngleAxisd(phi_y, ax_cam[1]) * Eigen::AngleAxisd(phi_x, ax_cam[0]);
    Eigen::Vector3d translation = Eigen::Vector3d(t_x, t_y, t_z);
    tmp = Eigen::Vector3d::Zero();
    // tmp = rotation * tmp + translation;
    tmp = rotation.inverse() * (tmp - translation);
    _s.x = (tmp[0]) / 1000.0;
    _s.y = (tmp[1]) / 1000.0;
    _s.z = (tmp[2]) / 1000.0;
    _s.r = 0;
    _s.g = 0;
    _s.b = 0;
    // axis 1
    tmp = Eigen::Vector3d::UnitX() * 50;
    tmp = rotation.inverse() * (tmp - translation);
    _ex.x = (tmp[0]) / 1000.0;
    _ex.y = (tmp[1]) / 1000.0;
    _ex.z = (tmp[2]) / 1000.0;
    _ex.r = 255;
    _ex.g = 0;
    _ex.b = 0;
    // axis 2
    tmp = Eigen::Vector3d::UnitY() * 50;
    tmp = rotation.inverse() * (tmp - translation);
    _ey.x = (tmp[0]) / 1000.0;
    _ey.y = (tmp[1]) / 1000.0;
    _ey.z = (tmp[2]) / 1000.0;
    _ey.r = 0;
    _ey.g = 255;
    _ey.b = 0;
    // axis 3
    tmp = Eigen::Vector3d::UnitZ() * 50;
    tmp = rotation.inverse() * (tmp - translation);
    _ez.x = (tmp[0]) / 1000.0;
    _ez.y = (tmp[1]) / 1000.0;
    _ez.z = (tmp[2]) / 1000.0;
    _ez.r = 0;
    _ez.g = 0;
    _ez.b = 255;
    viewer->runOnVisualizationThread(viewerDrawAxis);
    viewer->showCloud(cloud);
  }

  return 0;
}

int MyCalibration::startPickingPoints()
{
  g_picking_counter = 0;
  return 0;
}

int MyCalibration::pickPointsAutomatically(int pattern_rows, int pattern_cols)
{
  int i, j;
  int key;
  cv::Mat image_chess(CAMERA_RESOLUTION_Y, CAMERA_RESOLUTION_X, CV_8UC3);

  // openni::VideoFrameRef frame;
  bool pattern_found;
  std::vector< cv::Point2f > corners;
  cv::namedWindow("Detecting points");
  for (i = 0; i < image.rows; i++)
  {
    for (j = 0; j < image.cols; j++)
    {
      image_chess.at< cv::Vec3b >(i, j) = cv::Vec3b(255, 255, 255);
    }
  }

  if (color_frame != NULL)
  {  // Detect corners with color image.
    image_chess = cv::Mat(CAMERA_RESOLUTION_Y, CAMERA_RESOLUTION_X, CV_8UC3, color_frame);
    cv::cvtColor(image_chess, image_chess, CV_BGR2RGB);
  }
  else
  {                      // Detect corners with depth image.
    if (key == 1048681)  // key == 'i' ?
    {                    // Clean image
      for (i = 0; i < CAMERA_RESOLUTION_Y; i++)
      {
        for (j = 0; j < CAMERA_RESOLUTION_X; j++)
        {
          image.at< cv::Vec3b >(i, j) = cv::Vec3b(255, 255, 255);
        }
      }
    }
    // Fill with brack at invalid pixel
    float* dp;
    // dp = (openni::DepthPixel*)depth_frame;
    for (i = 0; i < image.rows; i++)
    {
      for (j = 0; j < image.cols; j++)
      {
        int now = (i * CAMERA_RESOLUTION_X + j);
        if (*(float*)(dp + now) == INVALID_PIXEL_VALUE)
        {
          image.at< cv::Vec3b >(i, j) = cv::Vec3b(0, 0, 0);
        }
      }
    }
  }
  find_corner(image_chess, cv::Size(pattern_cols, pattern_rows), corners, pattern_found);
  cv::drawChessboardCorners(image_chess, cv::Size(pattern_cols, pattern_rows), (cv::Mat)corners, pattern_found);
  cv::imshow("Detecting points", image_chess);
  if (!pattern_found)
  {
    int key = cv::waitKey(10);
    return key;
  }

  std::cout << "Detected corners" << std::endl;
  key = cv::waitKey(0);
  cv::destroyWindow("Detecting points");

  g_points[0][0] = (int)corners[0].x;
  g_points[0][1] = (int)corners[0].y;
  g_points[1][0] = (int)corners[(pattern_rows - 1) * pattern_cols].x;
  g_points[1][1] = (int)corners[(pattern_rows - 1) * pattern_cols].y;
  g_points[2][0] = (int)corners[pattern_cols - 1].x;
  g_points[2][1] = (int)corners[pattern_cols - 1].y;
  g_points[3][0] = (int)corners[pattern_rows * pattern_cols - 1].x;
  g_points[3][1] = (int)corners[pattern_rows * pattern_cols - 1].y;

  for (i = 0; i < pattern_rows; i++)
  {
    for (j = 0; j < pattern_cols; j++)
    {
      g_picked_points[i * pattern_cols + j][0] = (int)corners[i * pattern_cols + j].x;
      g_picked_points[i * pattern_cols + j][1] = (int)corners[i * pattern_cols + j].y;
    }
  }

  return 0;
}

void MyCalibration::getDepthFrameCallback(const sensor_msgs::Image::ConstPtr& frame)
{
  int i, j;
  uint8_t* p;
  for (i = 0, p = depth_frame; i < frame->height * frame->step; i++, p++)
  {
    *p = frame->data[i];
  }
  cv::Mat image_tmp = cv::Mat(frame->height, frame->width, CV_16UC1, depth_frame);
  image_tmp.convertTo(image, CV_8UC1, 255.0 / 10000.0);
  // cv::imshow("Depth image", image);
  return;
}

void MyCalibration::getColorFrameCallback(const sensor_msgs::Image::ConstPtr& frame)
{
  int i;
  uint8_t* p;
  for (i = 0, p = color_frame; i < frame->height * frame->step; i++, p++)
  {
    *p = frame->data[i];
  }
  // cv::Mat image_tmp = cv::Mat(frame->height, frame->width, CV_8UC3, color_frame);
  // cv::cvtColor(image_tmp,image_tmp,CV_BGR2RGB);
  // cv::imshow("Color image", image_tmp);
  // cv::waitKey(10);
  return;
}

void MyCalibration::getPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& points)
{
  this->points = points;
  return;
}
