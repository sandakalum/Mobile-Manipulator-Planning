//****************************
//顔認識プログラム
// 20回成功で画像取得、100回試行で終了
//キャリブレーションなし
//*****************************
//インクルードファイル
#include <ods_face_detection/face_detection.h>

//*******************************
//**入力座標までの距離を求める
//*******************************
void GetDepth(std::vector< int >& tmp_idx)
{
  std::cout << "Getdepth" << std::endl;

  int m = 0;
  for (int i = 0; i < IMAGE_HEIGHT * IMAGE_WIDTH; i++)
  {
    if (tmp_idx[m] == i)
    {
      idx.push_back(cloud->points[i].z * 1000);
      m++;
    }
    else
    {
      idx.push_back(0);
    }
  }

  return;
}

//*******************************
//**顔認識
//*******************************
int detectAndDraw(cv::Mat& img, cv::Mat& view, int cn, cv::CascadeClassifier& cascade,
                  cv::CascadeClassifier& nestedCascade, double scale)
{
  //変数宣言
  std::vector< cv::Rect > faces;

  cv::Mat gray, smallImg(cvRound(img.rows / scale), cvRound(img.cols / scale), CV_8UC1);
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  cv::resize(gray, smallImg, smallImg.size(), 0, 0, cv::INTER_LINEAR);
  cv::equalizeHist(smallImg, smallImg);
  cascade.detectMultiScale(smallImg, faces, 1.3, 3, 0 | CV_HAAR_FIND_BIGGEST_OBJECT /*|CV_HAAR_SCALE_IMAGE*/,
                           cv::Size(40, 40));

  //顔を検出したときの処理

  for (std::vector< cv::Rect >::const_iterator r = faces.begin(); r != faces.end(); r++)
  {
    //変数宣言
    cv::Point center;
    cv::Point center2;
    double radius, depth;

    //原画像に置ける顔の中心座標計算
    X = cvRound((r->x + r->width * 0.5) * scale);
    Y = cvRound((r->y + r->height * 0.5) * scale);

    //原画像における顔の中心座標計算
    switch (channel)
    {
      //標準モードの場合
      case 0:
        if (cn == 0)
        {
          center.x = X;
          center.y = Y;
        }
        else if (cn == 1)
        {
          center.x = IMAGE_WIDTH - X;
          center.y = Y;
        }
        break;

      //回転モードの場合
      case 1:
        center.x = (int)(320 + (X - 320) * cos(pi * (-angle) / 180) - (240 - Y) * sin(pi * (-angle) / 180));
        center.y = (int)(240 - (X - 320) * sin(pi * (-angle) / 180) - (240 - Y) * cos(pi * (-angle) / 180));
        center2.x = X;
        center2.y = Y;
        // std::cout << "(X, Y) = (" << X << ", " << Y << ")" << std::endl;
        // std::cout << "(center.x, center.y) = (" << center.x << ", " << center.y << ")" << std::endl;
        break;
    }

    //顔の中心座標(3次元)
    // depth = idx[center.y*IMAGE_WIDTH + center.x];

    // std::cout << "depth = " << depth << std::endl;

    // if((P.z < 40) || (1500 < P.z)) return 0;

    //描く円の半径を求める
    radius = cvRound((r->width + r->height) * 0.25 * scale);

    //円を描写
    cv::circle(img, center2, radius, CV_RGB(0, 0, 255), 3, 8, 0);
    // cv::circle( view, center, radius, CV_RGB(0,0,255), 3, 8, 0 );

    // std::cout << "success : " << depth << std::endl;
    sucnum++;

    if (nestedCascade.empty())
      continue;

    // success
    return 1;
  }

  // failure
  return 0;
}

//**************************
//**指定された角度だけ画像を回転
//**************************
void rotation_img(cv::Mat& src, cv::Mat& dst, double angle)
{
  double scale2 = 1.0;
  cv::Point2d center2(src.cols * 0.5, src.rows * 0.5);
  const cv::Mat affine_matrix = cv::getRotationMatrix2D(center2, angle, scale2);

  cv::warpAffine(src, dst, affine_matrix, dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(255));

  return;
}

//*******************************
//**顔認識(事前処理)
//*******************************
void face_detection(cv::Mat& frame)
{
  //変数宣言
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< int > tmp_idx;
  cv::CascadeClassifier cascade, nestedCascade;
  cv::Mat frame2, frame3;
  double scale = 1.0;
  int m = 0;

  //識別器の読み込み
  if (!cascade.load(cascadeName))
  {
    std::cerr << "ERROR: Could not load classifier cascade" << std::endl;
    return;
  }

  //初回のみ距離情報の取得
  /*if(trynum==1){
      pcl::removeNaNFromPointCloud(*cloud, *tmp_cloud, tmp_idx);
      GetDepth(tmp_idx);

      //距離値に基づくマスク画像の作成
      for(int y=0;y<mask.rows;y++){
          for(int x=0;x<mask.cols;x++){
              if((idx[y*mask.rows+x] == 0) ||(idx[y*mask.rows+x] > 1500))
                  mask.data[y*mask.rows+x] = 0;
              else
                  mask.data[y*mask.rows+x] = 255;

          }
      }
  }*/

  //反転画像の作成
  cv::flip(frame, frame2, 1);

  //回転画像作成の準備
  frame3 = frame.clone();

  //顔認識関数の呼び出し
  switch (channel)
  {
    //正面または左向きの顔の場合(使用しない)
    case 0:
      if (detectAndDraw(frame, frame, 0, cascade, nestedCascade, scale))
        std::cout << "success " << sucnum << std::endl;

      else if (detectAndDraw(frame2, frame, 1, cascade, nestedCascade, scale))
        std::cout << "success " << sucnum << std::endl;

      else
        std::cout << "failure" << std::endl;

      break;

    //傾いている顔の場合
    case 1:
      while (m < 36)
      {
        //画像の回転
        rotation_img(frame, frame3, angle);

        if (detectAndDraw(frame3, frame, 0, cascade, nestedCascade, scale))
        {
          std::cout << "success " << sucnum << std::endl;
          break;
        }

        angle += 10.0;
        m++;
        if (angle >= 360.0)
          angle -= 360.0;
      }
      if (m >= 36)
      {
        std::cout << "part1" << std::endl;
        std::cout << "failure" << std::endl;
      }
      break;

    default:
      break;
  }

  return;
}

//*******************************
//**RGB画像の取得
//*******************************
void rgb_capture(const sensor_msgs::Image::ConstPtr& input)
{
  std::cout << "capture rgb image " << trynum << std::endl;
  cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);

  if (!(cv_ptr->image.empty()))
    face_detection(cv_ptr->image);

  trynum++;

  return;
}

//*******************************
//**コールバック関数
//*******************************
bool callback(tms_msg_ss::ods_face_detection::Request& req, tms_msg_ss::ods_face_detection::Response& res)
{
  std::cout << "ods_face_detection" << std::endl;

  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Rate loop_rate(30);

  // capture rgb_image and depth_registered_image
  tms_msg_ss::ods_pcd srv;
  // pcl::PCLPointCloud2 cloud2;

  srv.request.id = 3;
  if (commander_to_kinect_capture.call(srv))
  {
    // pcl_conversions::toPCL(srv.response.cloud, cloud2);
    pcl::fromROSMsg(srv.response.cloud, *cloud);
    // pcl::fromPCLPointCloud2(cloud2, *cloud);
    pcl::io::savePCDFile("src/ods_face_detection/data/face_detection/depth_registered.pcd", *cloud);
  }

  pcl_sub = nh.subscribe("/camera/rgb/image_raw", 1000, rgb_capture);

  if (trynum >= TRY)
  {
    std::cout << "finish face detection" << std::endl;

    if (sucnum >= SUC)
    {
      std::cout << "success for face detection" << std::endl;
      cv::imwrite("src/ods_face_detection/data/face_detection/face_image.png", cv_ptr->image);
      cv_ptr->toImageMsg(res.image);
      std::cout << res.image.encoding << std::endl;
      res.result = 1;
      trynum = 0;
      sucnum = 0;
    }
    else
    {
      std::cout << "failure for face detection" << std::endl;
      res.result = 0;
    }

    return true;
  }

  while (trynum < TRY)
  {
    std::cout << "trynum = " << trynum << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "finish face detection" << std::endl;

  if (sucnum >= SUC)
  {
    std::cout << "success for face detection" << std::endl;
    cv::imwrite("src/ods_face_detection/data/face_detection/face_image.png", cv_ptr->image);
    cv_ptr->toImageMsg(res.image);
    std::cout << res.image.encoding << std::endl;
    res.result = 1;
    trynum = 0;
    sucnum = 0;
  }
  else
  {
    std::cout << "failure for face detection" << std::endl;
    res.result = 0;
  }
  return true;

  return true;
}

int main(int argc, char** argv)
{
  printf("init\n");
  ros::init(argc, argv, "ods_face_detection");
  ros::NodeHandle n;

  service = n.advertiseService("ods_face_detection", callback);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("ods_capture");

  ros::spin();

  return 0;
}
