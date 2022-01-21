#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <erl_rnt/util.hpp>
#include <erl_rnt/status.hpp>
#include <erl_rnt/Recognise.h>
#include <erl_rnt/Features.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>

#define MATCH_THRESHOLD 10
#define FILE 0

using namespace cv;

// TEMP VARIABLES
std::string dir = "~/catkin_ws/src/erl_rnt/data/";
std::string scene = dir + "";

rs2::video_frame *color_frame = NULL;
rs2::points points;
float crop_x = 0; float crop_y = 0; int matched = 0; float crop_z = 0;

// TODO: Refract all the fevor_pose_estimation call for a function "system()" is weird/slow ...
bool recognise_callback(erl_rnt::Recognise::Request& recognise_req, erl_rnt::Recognise::Response& recognise_res)
{
  // Export color_frame from realsense camera to ply
  // points.export_to_ply("/tmp/rs2_points.ply", *color_frame);

  // This is a default cpp program from PCL library
  // system("pcl_ply2pcd /tmp/rs2_points.ply /tmp/rs2_points.pcd");

  // system("region_growing_features /tmp/rs2_points.pcd Dhigh_intel_module_ref.pcd /tmp/feature_1.txt /tmp/feature_2.txt 0 0 0.00335 0.00335 0.0375 0");
  if(FILE == 0){
    //system("rosrun erl_rnt fevor_pose_estimation ~/catkin_ws/src/erl_rnt/data/scene_plate.pcd ~/catkin_ws/src/erl_rnt/data/plate.pcd 3 0.000895 0.02 30000");
  }
  else if(FILE == 1){
    //system("rosrun erl_rnt fevor_pose_estimation ~/catkin_ws/src/erl_rnt/data/scene0.pcd ~/catkin_ws/src/erl_rnt/data/cover.pcd 3 0.0001 0.015 30000");

  }




  std::string line;
  // std::ifstream recognition_results_filestream_in("/tmp/recognition_results.txt");
  std::ifstream recognition_results_filestream_in("matched.txt");

  if (recognition_results_filestream_in.is_open())
  {
    int i;
    for (i = 0; getline(recognition_results_filestream_in, line); i++)
    {
      if (i == 0)
      matched = stoi(line);
      if (i == 1)
      crop_x = stof(line);
      if (i == 2)
      crop_y = stof(line);
      if (i == 3)
      crop_z = stof(line);
    }
    recognition_results_filestream_in.close();
  }
  else{
    ROS_INFO_STREAM("Matched file did not open");
  }

  if(matched > MATCH_THRESHOLD)
  {
    recognise_res.status = BATTERY_RECOGNISED;
  }
  else
  {
    recognise_res.status = NOTHING_RECOGNISED;
  }
  recognise_res.header.frame_id = recognise_req.header.frame_id;
  recognise_res.header.stamp = ros::Time::now();

  return true;
}


// MAIN
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "erl_rnt_node");
  ros::NodeHandle nh;

  ros::ServiceServer recognise_server = nh.advertiseService("/erl_leaf/recognise", recognise_callback);

  ros::Publisher features_pub = nh.advertise<erl_rnt::Features>("/erl_leaf/features", 100);

  rs2::colorizer color_map;
  rs2::decimation_filter dec;
  rs2::spatial_filter spat;

  // Create a simple OpenGL window for rendering:
  window app(1280, 720, "RealSense Pointcloud Example");
  // Construct an object to manage view state
  glfw_state app_state;
  // register callbacks to allow manipulation of the pointcloud
  register_glfw_callbacks(app, app_state);

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Start streaming with default recommended configuration
  pipe.start();

  while (app && ros::ok()) // Application still alive?
  {
    float x = 0; float y = 0;

    // Wait for the next set of frames from the camera
    rs2::frameset frames = pipe.wait_for_frames();

    if (color_frame) delete color_frame;
    color_frame = new rs2::video_frame(frames.get_color_frame());

    // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color_frame
    if (!(*color_frame))
    {
      delete color_frame;
      color_frame = new rs2::video_frame(frames.get_infrared_frame());
    }

    // Tell pointcloud object to map to this color_frame frame
    pc.map_to(*color_frame);

    auto depth_frame = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth_frame);

    /////////////////////////////////////////////////////////distance/////////////////////////////////////////////////
    float width = depth_frame.get_width();
    float height = depth_frame.get_height();
    float dist_to_center = depth_frame.get_distance(width / 2, height / 2);

    //////////////Print the distance////////////////////////
    // ROS_INFO_STREAM("The depth camera is facing an object " << dist_to_center << " meters away.");
    // ROS_INFO_STREAM("Matched => "<<matched);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::spinOnce();

    if (matched > MATCH_THRESHOLD)
    {
      stbi_write_png("/tmp/color_frame.png", color_frame->get_width(), color_frame->get_height(), color_frame->get_bytes_per_pixel(), color_frame->get_data(), color_frame->get_stride_in_bytes());
      Mat src = imread("/tmp/color_frame.png", IMREAD_COLOR);
      Mat dst;
      cvtColor(src, dst, COLOR_RGB2GRAY);

      //////////////////////////////////////////////////////////////////// map 3D to 2D........................../////////////////////////////////
      x = crop_x / -crop_z;
      y = -crop_y / -crop_z;

      const struct rs2_intrinsics intrin = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();;
      if (intrin.model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY)
      {
        std::cout << "model1\n";
        float r2 = x * x + y * y;
        float f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2;
        x *= f;
        y *= f;
        float dx = x + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x);
        float dy = y + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y);
        x = dx;
        y = dy;
      }
      if (intrin.model == RS2_DISTORTION_FTHETA)
      {
        std::cout << "model2\n";
        float r = sqrt(x * x + y * y);
        float rd = (1.0f / intrin.coeffs[0] * atan(2 * r * tan(intrin.coeffs[0] / 2.0f)));
        x *= rd / r;
        y *= rd / r;
      }

      x = x * intrin.fx + intrin.ppx;
      y = y * intrin.fy + intrin.ppy;

      line(src, Point(x, y), Point(x, y), (255, 0, 0), 20);
      ///////////////////////////////////////////////////////////////////////////detecting ROI for module/////////////////////////////////////////////////////////
      Mat gray;
      float min_x, min_y; float dist = 0; float temp_dist = 10000; Point center; float mindist = 10000;
      cvtColor(src, gray, COLOR_BGR2GRAY);
      medianBlur(gray, gray, 5);
      std::vector<Vec3f> circles;
      HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
        gray.rows / 8,  // change this value to detect circles with different distances to each other
        100, 22, 1, 10 // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
      );
      for (size_t i = 0; i < circles.size(); i++)
      {
        Vec3i c = circles[i];
        center = Point(c[0], c[1]);
        // circle center
        circle(src, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle(src, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
        dist = pow((x - center.x), 2) + pow((y - center.y), 2);       //calculating Euclidean distance
        dist = sqrt(dist);
        if (dist < mindist)
        {
          mindist = dist;
          min_x = center.x; min_y = center.y;
        }

      }
      line(src, Point(min_x, min_y), Point(min_x + 1, min_y + 1), (255, 0, 0), 50);
      ////distance from one center to another////////
      float second_x = 0, second_y = 0, third_x = 0, third_y = 0, fourth_x = 0, fourth_y = 0; int count_circle = 1;
      /////detect lines for proof of true circles////////

      std::vector<Vec4i> linesP; // will hold the results of the detection
      Canny(dst, dst, 255, 100, 3);
      HoughLinesP(dst, linesP, 1, CV_PI / 180, 150, 40, 5); // runs the actual detection
      for (size_t i = 0; i < linesP.size(); i++)
      {
        Vec4i l = linesP[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);

      }

      int c_daigonal = 0, c_long = 0, c_short_l = 0, c_short_s = 0; int diff_c_x = 300; int near_lines_circle = 300; float short_s_x = 0, short_s_y = 0;
      int dis_flag_d = 0; int dis_flag_ll = 0; int dis_flag_sl = 0; int dis_flag_ss = 0;
      int daig_count = 0; int long_count = 0; int short_l_count = 0; int short_s_count = 0;
      for (size_t i = 0; i < circles.size(); i++)
      {
        Vec3i c = circles[i];
        center = Point(c[0], c[1]);
        // circle center
        circle(src, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
        dist = pow((min_x - center.x), 2) + pow((min_y - center.y), 2);       //calculating Euclidean distance
        dist = sqrt(dist);
        std::cout << "dist is\n" << dist << "\n";

        if (dist > 610 && dist < 650)
        {
          daig_count++;
          float dis_true_cir1 = 0, dis_true_cir2 = 0, dis_true_cir3 = 0, dis_true_cir4 = 0;

          for (size_t i = 0; i < linesP.size(); i++) ////check for true circles/////////////////////////
          {
            Vec4i l = linesP[i];


            dis_true_cir1 = pow((l[0] - center.x), 2) + pow((l[1] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir1 = sqrt(dis_true_cir1);
            dis_true_cir2 = pow((l[2] - center.x), 2) + pow((l[3] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir2 = sqrt(dis_true_cir2);
            std::cout << "\n it is daigonal" << " " << " true dis is " << dis_true_cir1 << " " << dis_true_cir2 << "\n";
            if ((center.x - x) > 10)
            {
              if (abs(center.x - diff_c_x - l[0]) < 60 || abs(center.x - diff_c_x - l[2]) < 60)
              c_daigonal++;

              std::cout << "\n transfer point1" << " " << center.x << " " << center.x - diff_c_x - l[0];
            }
            if ((center.x - x) < -10)
            {
              if (abs(center.x + diff_c_x - l[0]) < 60 || abs(center.x + diff_c_x - l[2]) < 60)
              c_daigonal++;

              std::cout << "\n transfer point2" << " " << center.x << " " << center.x + diff_c_x - l[2];
            }
            if ((center.y - y) > 10)
            {
              if (abs(center.y - diff_c_x - l[1]) < 60 || abs(center.y - diff_c_x - l[3]) < 60)
              c_daigonal++;
            }
            if ((center.y - y) < -10)
            {
              if (abs(center.y + diff_c_x - l[1]) < 60 || abs(center.y + diff_c_x - l[3]) < 60)
              c_daigonal++;
            }

            if (dis_true_cir1 < near_lines_circle || dis_true_cir2 < near_lines_circle)
            {
              dis_flag_d = 1;
            }
            if (c_daigonal > 0 && dis_flag_d == 1)
            {
              if (daig_count > 1) ////distance from x,y matched keypoints center
              {
                float dis_temp1 = pow((x - center.x), 2) + pow((y - center.y), 2);       //calculating Euclidean distance
                dis_temp1 = sqrt(dis_temp1);
                float dis_temp2 = pow((x - second_x), 2) + pow((y - second_y), 2);       //calculating Euclidean distance
                dis_temp2 = sqrt(dis_temp2);
                if (dis_temp1 < dis_temp2)
                {
                  second_x = center.x; second_y = center.y;
                  line(src, Point(second_x, second_y), Point(second_x + 1, second_y + 1), Scalar(0, 0, 255), 20);
                  std::cout << "\n \nalternate is passed " << "\n \n";
                }
              }
              else
              {
                std::cout << "\n proof of daigonal" << "\n";
                second_x = center.x; second_y = center.y;
                line(src, Point(second_x, second_y), Point(second_x + 1, second_y + 1), (255, 0, 0), 20);
              }

            }


          }
        }

        if (dist > 510 && dist < 560)
        {
          long_count++;
          float dis_true_cir1 = 0, dis_true_cir2 = 0;
          for (size_t i = 0; i < linesP.size(); i++) ////check for true circles/////////////////////////
          {
            Vec4i l = linesP[i];
            dis_true_cir1 = pow((l[0] - center.x), 2) + pow((l[1] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir1 = sqrt(dis_true_cir1);
            dis_true_cir2 = pow((l[2] - center.x), 2) + pow((l[3] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir2 = sqrt(dis_true_cir2);
            std::cout << "\n it is long line" << " " << " true dis is " << dis_true_cir1 << " " << dis_true_cir2 << "\n";
            if ((center.x - x) > 10)
            {
              if (abs(center.x - diff_c_x - l[0]) < 60 || abs(center.x - diff_c_x - l[2]) < 60)
              c_long++;

              std::cout << "\n transfer point1" << " " << center.x << " " << center.x - diff_c_x - l[0];
            }
            if ((center.x - x) < -10)
            {
              if (abs(center.x + diff_c_x - l[0]) < 60 || abs(center.x + diff_c_x - l[2]) < 60)
              c_long++;

              std::cout << "\n transfer point2" << " " << center.x << " " << center.x + diff_c_x - l[2];
            }
            if ((center.y - y) > 10)
            {
              if (abs(center.y - diff_c_x - l[1]) < 60 || abs(center.y - diff_c_x - l[3]) < 60)
              c_long++;
            }
            if ((center.y - y) < -10)
            {
              if (abs(center.y + diff_c_x - l[1]) < 60 || abs(center.y + diff_c_x - l[3]) < 60)
              c_long++;
            }

            if (dis_true_cir1 < near_lines_circle || dis_true_cir2 < near_lines_circle)
            {
              dis_flag_ll = 1;

            }
            if (c_long > 0 && dis_flag_ll == 1)
            {
              if (long_count > 1) ////distance from x,y matched keypoints center
              {
                float dis_temp1 = pow((x - center.x), 2) + pow((y - center.y), 2);       //calculating Euclidean distance
                dis_temp1 = sqrt(dis_temp1);
                float dis_temp2 = pow((x - third_x), 2) + pow((y - third_y), 2);       //calculating Euclidean distance
                dis_temp2 = sqrt(dis_temp2);
                if (dis_temp1 < dis_temp2)
                {
                  third_x = center.x; third_y = center.y;
                  line(src, Point(third_x, third_y), Point(third_x + 1, third_y + 1), Scalar(0, 0, 255), 20);
                  std::cout << "\n \nalternate is passed " << "\n \n";
                }
              }
              else
              {
                third_x = center.x; third_y = center.y;
                line(src, Point(third_x, third_y), Point(third_x + 1, third_y + 1), (255, 0, 0), 20);
              }

            }

          }
        }
        int ff = 0;
        if (dist > 350 && dist < 390)
        {
          short_l_count++;
          float dis_true_cir1 = 0, dis_true_cir2 = 0;
          for (size_t i = 0; i < linesP.size(); i++) ////check for true circles/////////////////////////
          {
            Vec4i l = linesP[i];
            dis_true_cir1 = pow((l[0] - center.x), 2) + pow((l[1] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir1 = sqrt(dis_true_cir1);
            dis_true_cir2 = pow((l[2] - center.x), 2) + pow((l[3] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir2 = sqrt(dis_true_cir2);
            std::cout << "it is short long line" << " " << " true dis is " << dis_true_cir1 << " " << dis_true_cir2 << "\n";
            if ((center.x - x) > 10)
            {
              if (abs(center.x - diff_c_x - l[0]) < 60 || abs(center.x - diff_c_x - l[2]) < 60)
              c_short_l++;

              std::cout << "\n transfer point1" << " " << center.x << " " << abs(center.x - diff_c_x - l[0]) << " " << abs(center.x - diff_c_x - l[2]);
            }
            if ((center.x - x) < -10)
            {
              if (abs(center.x + diff_c_x - l[0]) < 60 || abs(center.x + diff_c_x - l[2]) < 60)
              c_short_l++;

              std::cout << "\n transfer point2" << " " << center.x << " " << abs(center.x + diff_c_x - l[0]) << " " << abs(center.x + diff_c_x - l[2]);

            }
            if ((center.y - y) > 10)
            {
              if (abs(center.y - diff_c_x - l[1]) < 60 || abs(center.y - diff_c_x - l[3]) < 60)
              c_short_l++;
            }
            if ((center.y - y) < -10)
            {
              if (abs(center.y + diff_c_x - l[1]) < 60 || abs(center.y + diff_c_x - l[3]) < 60)
              c_short_l++;
            }

            if (dis_true_cir1 < near_lines_circle || dis_true_cir2 < near_lines_circle)
            {
              dis_flag_sl = 1;
            }
            if (c_short_l > 0 && dis_flag_sl == 1)
            {
              if (short_l_count > 1) ////distance from x,y matched keypoints center
              {
                float dis_temp1 = pow((x - center.x), 2) + pow((y - center.y), 2);       //calculating Euclidean distance
                dis_temp1 = sqrt(dis_temp1);
                float dis_temp2 = pow((x - fourth_x), 2) + pow((y - fourth_y), 2);       //calculating Euclidean distance
                dis_temp2 = sqrt(dis_temp2);
                if (dis_temp1 < dis_temp2)
                {
                  fourth_x = center.x; fourth_y = center.y;
                  line(src, Point(fourth_x, fourth_y), Point(fourth_x + 1, fourth_y + 1), Scalar(0, 0, 255), 20);
                  short_s_x = fourth_x;  short_s_y = fourth_y;
                  std::cout << "\n \nalternate is passed " << "\n \n";
                }
              }
              else
              {
                ff = 1;
                fourth_x = center.x; fourth_y = center.y;
                line(src, Point(fourth_x, fourth_y), Point(fourth_x + 1, fourth_y + 1), (255, 0, 0), 20);
                short_s_x = fourth_x;  short_s_y = fourth_y;
              }

            }

          }
        }

        if (dist > 270 && dist < 325 && ff == 0)
        {
          short_s_count++;
          float dis_true_cir1 = 0, dis_true_cir2 = 0;
          for (size_t i = 0; i < linesP.size(); i++) ////check for true circles/////////////////////////
          {
            Vec4i l = linesP[i];
            dis_true_cir1 = pow((l[0] - center.x), 2) + pow((l[1] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir1 = sqrt(dis_true_cir1);
            dis_true_cir2 = pow((l[2] - center.x), 2) + pow((l[3] - center.y), 2);       //calculating Euclidean distance
            dis_true_cir2 = sqrt(dis_true_cir2);
            std::cout << "it is short short line" << " " << " true dis is " << dis_true_cir1 << " " << dis_true_cir2 << "\n";
            //////////////////////////////////////////////////////
            if ((center.x - x) > 10)
            {
              if (abs(center.x - diff_c_x - l[0]) < 60 || abs(center.x - diff_c_x - l[2]) < 60)
              c_short_s++;

              std::cout << "\n transfer point1" << " " << center.x << " " << center.x - diff_c_x - l[0];
            }
            if ((center.x - x) < -10)
            {
              if (abs(center.x + diff_c_x - l[0]) < 60 || abs(center.x + diff_c_x - l[2]) < 60)
              c_short_s++;

              std::cout << "\n transfer point2" << " " << center.x << " " << center.x + diff_c_x - l[2];
            }
            if ((center.y - y) > 10)
            {
              if (abs(center.y - diff_c_x - l[1]) < 60 || abs(center.y - diff_c_x - l[3]) < 60)
              c_short_s++;
            }
            if ((center.y - y) < -10)
            {
              if (abs(center.y + diff_c_x - l[1]) < 60 || abs(center.y + diff_c_x - l[3]) < 60)
              c_short_s++;
            }
            ////////////////////////////////////////////////

            if (dis_true_cir1 < near_lines_circle || dis_true_cir2 < near_lines_circle)
            {
              dis_flag_ss = 1;
            }
            if (c_short_s > 0 && dis_flag_ss == 1)
            {
              if (short_s_count > 1) ////distance from x,y matched keypoints center
              {
                float dis_temp1 = pow((x - center.x), 2) + pow((y - center.y), 2);       //calculating Euclidean distance
                dis_temp1 = sqrt(dis_temp1);
                float dis_temp2 = pow((x - fourth_x), 2) + pow((y - fourth_y), 2);       //calculating Euclidean distance
                dis_temp2 = sqrt(dis_temp2);
                if (dis_temp1 < dis_temp2)
                {
                  fourth_x = center.x; fourth_y = center.y;
                  line(src, Point(fourth_x, fourth_y), Point(fourth_x + 1, fourth_y + 1), Scalar(0, 0, 255), 20);
                  std::cout << "\n \nalternate is passed " << "\n \n";
                }
              }
              else
              {
                fourth_x = center.x; fourth_y = center.y;
                line(src, Point(fourth_x, fourth_y), Point(fourth_x + 1, fourth_y + 1), (255, 0, 0), 20);
              }

            }


          }
        }


      }
      //////////////////////////////////////////////center of all 4 points/////////////////////////////////////////////
      int center_4points_x = 0; int center_4points_y = 0;
      center_4points_x = (min_x + second_x + third_x + fourth_x) / 4;
      center_4points_y = (min_y + second_y + third_y + fourth_y) / 4;
      line(src, Point(center_4points_x, center_4points_y), Point(center_4points_x + 5, center_4points_y), Scalar(100, 100, 255), 40);
      //////////if 3 points are found...we can estimate 4th one//////////////////////
      if (second_x > 0)
      count_circle++;
      if (third_x > 0)
      count_circle++;
      if (fourth_x > 0)
      count_circle++;
      float xchange = 0; float ychange = 0; int xflag = 0; int yflag = 0;
      std::cout << "\ncount of circles are " << count_circle << "\n";
      if (count_circle == 3)
      {
        if (abs(min_x - second_x) > 200 && (second_x > 0))
        {
          xflag++;
          xchange = min_x - second_x;
        }
        if (abs(min_x - second_y) > 200 && (second_y > 0))
        {
          yflag++;
          ychange = min_y - second_y;
        }

        if (abs(min_x - third_x) > 200 && (third_x > 0))
        {
          xflag++;
          xchange = min_x - third_x;
        }
        if (abs(min_x - third_y) > 200 && (third_y > 0))
        {
          yflag++;
          ychange = min_y - third_y;
        }

        if (abs(min_x - fourth_x) > 200 && (fourth_x > 0))
        {
          xflag++;
          xchange = min_x - fourth_x;
        }
        if (abs(min_x - fourth_y) > 200 && (fourth_y > 0))
        {
          yflag++;
          ychange = min_y - fourth_y;
        }
        std::cout << "\nxflag" << xflag;// << "" << xchange;
        std::cout << "\nyflag" << yflag;// << "" << ychange;
        float rem_point_x; float rem_point_y;
        if (xflag == 1 && yflag == 1)
        {
          rem_point_x = min_x - xchange;
          rem_point_y = min_y - ychange;

        }

        if (xflag == 1 && yflag == 2)
        {
          rem_point_x = min_x - xchange;
          rem_point_y = min_y;

        }
        if (xflag == 2 && yflag == 1)
        {
          rem_point_x = min_x;
          rem_point_y = min_y - ychange;

        }
        std::cout << "\nx" << rem_point_x;
        std::cout << "\ny" << rem_point_y;
        line(src, Point(rem_point_x, rem_point_y), Point(rem_point_x + 1, rem_point_y + 1), Scalar(0, 0, 255), 20);
        //////////we can estimate 4th one//////////////////////

      }
      ///////////////////////////finding terminal of module/////////////////////
      float aX; float aY; float bX; float bY; float t_x, t_y, t1_x, t1_y, t2_x, t2_y;
      if (short_s_x > 0)
      {
        t_x = (min_x + short_s_x) / 2.01; t_y = (min_y + short_s_y) / 2.01;
        t1_x = float(min_x + (0.9 / 3.0) * (short_s_x - min_x)); t1_y = float(min_y + (0.9 / 3.0) * (short_s_y - min_y));
        t2_x = float(min_x + (2.07 / 3.0) * (short_s_x - min_x)); t2_y = float(min_y + (2.07 / 3.0) * (short_s_y - min_y));
        aX = min_x; aY = min_y;  bX = short_s_x; bY = short_s_y;
      }
      else
      {
        t_x = (second_x + third_x) / 2.01; t_y = (second_y + third_y) / 2.01;
        t1_x = float(second_x + (0.9 / 3.0) * (third_x - second_x)); t1_y = float(second_y + (0.9 / 3.0) * (third_y - second_y));
        t2_x = float(second_x + (2.07 / 3.0) * (third_x - second_x)); t2_y = float(second_y + (2.07 / 3.0) * (third_y - second_y));
        aX = second_x; aY = second_y;  bX = third_x; bY = third_y;
      }
      ///////////////////////draw perpendicular line on t_x nad t_y//////////////////////////////////////////////
      float vX = bX - aX;
      float vY = bY - aY;
      float mag = sqrt(vX * vX + vY * vY);
      vX = vX / mag;
      vY = vY / mag;
      float temp = vX;
      vX = 0 - vY;
      vY = temp;
      float cX = t_x + vX * 30; float c1X = t1_x + vX * 30; float c2X = t2_x + vX * 30;
      float cY = t_y + vY * 30; float c1Y = t1_y + vY * 30; float c2Y = t2_y + vY * 30;
      float dX = t_x - vX * 30; float d1X = t1_x - vX * 30; float d2X = t2_x - vX * 30;
      float dY = t_y - vY * 30; float d1Y = t1_y - vY * 30; float d2Y = t2_y - vY * 30;

      std::cout << "\nmin_x " << min_x << "min_y " << min_y << "t1_x " << t1_x << " t1_y" << t1_y;

      float dis1 = pow((center_4points_x - cX), 2) + pow((center_4points_y - cY), 2);       //calculating Euclidean distance
      dis1 = sqrt(dis1);
      float dis2 = pow((center_4points_x - dX), 2) + pow((center_4points_y - dY), 2);       //calculating Euclidean distance
      dis2 = sqrt(dis2);

      if (dis1 > dis2)
      {
        line(src, Point(cX, cY), Point(cX + 1, cY + 1), Scalar(0, 0, 255), 20);
        line(src, Point(c1X, c1Y), Point(c1X + 1, c1Y + 1), Scalar(0, 0, 255), 20);
        line(src, Point(c2X, c2Y), Point(c2X + 1, c2Y + 1), Scalar(0, 0, 255), 20);
      }
      else
      {
        line(src, Point(dX, dY), Point(dX + 1, dY + 1), Scalar(0, 0, 255), 20);
        line(src, Point(d1X, d1Y), Point(d1X + 1, d1Y + 1), Scalar(0, 0, 255), 20);
        line(src, Point(d2X, d2Y), Point(d2X + 1, d2Y + 1), Scalar(0, 0, 255), 20);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////

      imshow("detected circles", src);
      //////////////////////////////////Map 2D to 3D/////////////////////////////x = x * intrin.fx + intrin.ppx;/////////////////////////////////
      float pc_center_x = 0, pc_center_y = 0, pc_center_z = 0; float pc_hole1_x = 0, pc_hole1_y = 0, pc_hole1_z = 0;
      float pc_hole2_x = 0, pc_hole2_y = 0, pc_hole2_z = 0; 	float pc_hole3_x = 0, pc_hole3_y = 0, pc_hole3_z = 0; 	float pc_hole4_x = 0, pc_hole4_y = 0, pc_hole4_z = 0;
      float pc_term1_x = 0, pc_term1_y = 0, pc_term1_z = 0;   float pc_term2_x = 0, pc_term2_y = 0, pc_term2_z = 0;   float pc_term3_x = 0, pc_term3_y = 0, pc_term3_z = 0;
      pc_center_x = -crop_z * ((center_4points_x - intrin.ppx) / intrin.fx);
      pc_center_y = crop_z * ((center_4points_y - intrin.ppy) / intrin.fy);
      pc_center_z = crop_z;

      pc_hole1_x = -crop_z * ((min_x - intrin.ppx) / intrin.fx);
      pc_hole1_y = crop_z * ((min_y - intrin.ppy) / intrin.fy);
      pc_hole1_z = crop_z;

      pc_hole2_x = -crop_z * ((second_x - intrin.ppx) / intrin.fx);
      pc_hole2_y = crop_z * ((second_y - intrin.ppy) / intrin.fy);
      pc_hole2_z = crop_z;

      pc_hole3_x = -crop_z * ((third_x - intrin.ppx) / intrin.fx);
      pc_hole3_y = crop_z * ((third_y - intrin.ppy) / intrin.fy);
      pc_hole3_z = crop_z;

      pc_hole4_x = -crop_z * ((fourth_x - intrin.ppx) / intrin.fx);
      pc_hole4_y = crop_z * ((fourth_y - intrin.ppy) / intrin.fy);
      pc_hole4_z = crop_z;

      pc_term1_x = -crop_z * ((t_x - intrin.ppx) / intrin.fx);
      pc_term1_y = crop_z * ((t_y - intrin.ppy) / intrin.fy);
      pc_term1_z = crop_z;

      pc_term2_x = -crop_z * ((t1_x - intrin.ppx) / intrin.fx);
      pc_term2_y = crop_z * ((t1_y - intrin.ppy) / intrin.fy);
      pc_term2_z = crop_z;

      pc_term3_x = -crop_z * ((t2_x - intrin.ppx) / intrin.fx);
      pc_term3_y = crop_z * ((t2_y - intrin.ppy) / intrin.fy);
      pc_term3_z = crop_z;
      std::ofstream tracking_results_filestream_out;
      tracking_results_filestream_out.open("/tmp/tracking_results.txt");
      tracking_results_filestream_out << pc_center_x << "\n" << pc_center_y << "\n" << pc_center_z << "\n" << pc_hole1_x << "\n" << pc_hole1_y << "\n" << pc_hole1_z << "\n" << pc_hole2_x <<
      "\n" << pc_hole2_y << "\n" << pc_hole2_z << "\n" << pc_hole3_x << "\n" << pc_hole3_y << "\n" << pc_hole3_z << "\n" << pc_hole4_x << "\n" << pc_hole4_y <<
      "\n" << pc_hole4_z << "\n" << pc_term1_x << "\n" << pc_term1_y << "\n" << pc_term1_z << "\n" << pc_term2_x << "\n" << pc_term2_y << "\n" << pc_term2_z <<
      "\n" << pc_term3_x << "\n" << pc_term3_y << "\n" << pc_term3_z;
      tracking_results_filestream_out.close();

      erl_rnt::Features features_msg;

      geometry_msgs::Pose2D center_feature;
      center_feature.x = center_4points_x;
      center_feature.y = center_4points_y;
      features_msg.feature_names.push_back("center");
      features_msg.features.push_back(center_feature);

      geometry_msgs::Pose2D hole_1_feature;
      hole_1_feature.x = min_x;
      hole_1_feature.y = min_y;
      features_msg.feature_names.push_back("hole_1");
      features_msg.features.push_back(hole_1_feature);

      geometry_msgs::Pose2D hole_2_feature;
      hole_2_feature.x = second_x;
      hole_2_feature.y = second_y;
      features_msg.feature_names.push_back("hole_2");
      features_msg.features.push_back(hole_2_feature);

      geometry_msgs::Pose2D hole_3_feature;
      hole_3_feature.x = third_x;
      hole_3_feature.y = third_y;
      features_msg.feature_names.push_back("hole_3");
      features_msg.features.push_back(hole_3_feature);

      geometry_msgs::Pose2D hole_4_feature;
      hole_4_feature.x = fourth_x;
      hole_4_feature.y = fourth_y;
      features_msg.feature_names.push_back("hole_4");
      features_msg.features.push_back(hole_4_feature);

      geometry_msgs::Pose2D term_1_feature;
      term_1_feature.x = t_x;
      term_1_feature.y = t_y;
      features_msg.feature_names.push_back("term_1");
      features_msg.features.push_back(term_1_feature);

      geometry_msgs::Pose2D term_2_feature;
      term_2_feature.x = t1_x;
      term_2_feature.y = t1_y;
      features_msg.feature_names.push_back("term_2");
      features_msg.features.push_back(term_2_feature);

      geometry_msgs::Pose2D term_3_feature;
      term_3_feature.x = t2_x;
      term_3_feature.y = t2_y;
      features_msg.feature_names.push_back("term_3");
      features_msg.features.push_back(term_3_feature);

      features_msg.header.frame_id = "depth_camera_image";
      features_msg.header.stamp = ros::Time::now();

      features_pub.publish(features_msg);
    }

    // Upload the color_frame frame to OpenGL
    app_state.tex.upload(*color_frame);

    // Draw the pointcloud
    draw_pointcloud(app.width(), app.height(), app_state, points);
  }

  return 0;
}
