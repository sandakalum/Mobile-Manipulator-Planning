//*******************************************
//**肌色抽出：顔方向の推定
//*******************************************
#include <ods_skin_color/skin_color.h>

//*******************************
//**入力座標までの距離を求める
//*******************************
void GetDepth(std::vector< int > &tmp_idx)
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
//**距離情報を付加した肌色抽出
//*******************************
void check(int *array1, int x, int y, double z, int c)
{
  int cn, d;

  if (((x < 0) || (IMAGE_WIDTH <= x)) || ((y < 0) || (IMAGE_HEIGHT <= y)))
    return;

  cn = y * IMAGE_WIDTH + x;

  //未チェックのピクセル、肌色条件クリアのピクセル
  if ((array1[cn] == 0) || (array1[cn] == 3))
  {
    //注目ピクセルの距離値でフィルタをかける
    d = idx[y * IMAGE_WIDTH + x];
    if (d < 400 || d > 1500)
    {
      array1[cn] = 1;
      return;
    }

    //注目ピクセルまでの距離と隣接するピクセルまでの距離とが5cm以内の差であれば色情報を残す
    if ((z - 50 < d) && (d < z + 50))
    {
      if (array1[cn] == 0)
        array1[cn] = 4;
      else if (array1[cn] == 3)
      {
        array1[cn] = 2;
      }
      //さらに隣接するピクセルを調べる
      switch (c)
      {
        case 1:
          check(array1, x - 1, y, d, 1);
        case 2:
          check(array1, x + 1, y, d, 2);
        case 3:
          check(array1, x, y - 1, d, 3);
        case 4:
          check(array1, x, y + 1, d, 4);
      }
    }

    // 5cm以上の差であれば色情報をマスク
    else
      array1[cn] = 1;
  }
}

//*******************************
//**肌色抽出
//*******************************
void extraction(IplImage &src, cv::Mat &mask)
{
  std::cout << "abstraction" << std::endl;

  int array1[IMAGE_WIDTH * IMAGE_HEIGHT];
  uchar *p_src;
  CvPixelPosition8u pos_src;

  //配列の初期化
  for (int y = 0; y < IMAGE_HEIGHT; y++)
  {
    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
      array1[y * IMAGE_WIDTH + x] = 0;
    }
  }

  CV_INIT_PIXEL_POS(pos_src, (unsigned char *)src.imageData, src.widthStep, cvGetSize(&src), 0, 0, src.origin);

  for (int y = 0; y < IMAGE_HEIGHT; y++)
  {
    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
      p_src = CV_MOVE_TO(pos_src, x, y, 3);

      int cn2 = y * IMAGE_WIDTH + x;

      // RGB値
      uchar R = p_src[2];
      uchar G = p_src[1];
      uchar B = p_src[0];

      uchar MAX = std::max(R, std::max(G, B));
      uchar MIN = std::min(R, std::min(G, B));

      // H値
      uchar H = 0;
      if ((MAX - MIN) != 0)
      {
        if (MAX == R)
          H = 60 * (G - B) / (MAX - MIN);
        else if (MAX == G)
          H = 60 * (B - R) / (MAX - MIN) + 120;
        else if (MAX == B)
          H = 60 * (R - G) / (MAX - MIN) + 240;
      }

      //閾値処理
      //未チェックのピクセル、距離情報の条件クリアのピクセル
      //肌色の条件確認
      if ((array1[cn2] == 0) || array1[cn2] == 4)
      {
        // double z = idx[y*IMAGE_WIDTH + x];

        // if((400 < z) && (z < 1500)){
        if (MAX == R)
        {
          if (H <= 15)
          {
            //色情報を残すピクセル
            array1[cn2] = 2;
            // check(array1, x-1, y, z, 1);
            // check(array1, x+1, y, z, 2);
            // check(array1, x, y-1, z, 3);
            // check(array1, x, y+1, z, 4);
          }
          else
          {
            // if(array1[cn2] == 0)
            array1[cn2] = 3;
            // else if(array1[cn2] == 4) {
            //    array1[cn2] = 2;
            //}
          }
        }
        // R成分がMAXでないものはマスク
        else
          array1[cn2] = 1;
        //}
        // 3m以上の距離のピクセルはマスク
        // else array1[cn2] = 1;
      }
    }
  }

  //色情報の操作
  for (int j = 0; j < IMAGE_HEIGHT; j++)
  {
    for (int i = 0; i < IMAGE_WIDTH; i++)
    {
      if (array1[j * IMAGE_WIDTH + i] == 2)
      {
        mask.data[j * IMAGE_WIDTH + i] = 255;
        if (i < 320)
          count--;
        else
          count++;
      }
      else
      {
        mask.data[j * IMAGE_WIDTH + i] = 0;
      }
    }
  }

  return;
}

//*******************************
//**コールバック関数
//*******************************
bool skin_extraction(tms_msg_ss::ods_skincolor_extraction::Request &req,
                     tms_msg_ss::ods_skincolor_extraction::Response &res)
{
  std::cout << "skin_color" << std::endl;

  //変数宣言
  cv_bridge::CvImagePtr rgb_ptr(new cv_bridge::CvImage);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
  std::vector< int > tmp_idx;
  cv::Mat frame;
  cv::Mat mask(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

  // RGB画像とキャリブレーション済み点群の取得
  tms_msg_ss::ods_pcd srv, srv2;
  /*srv.request.id = 3;
  if(commander_to_kinect_capture.call(srv)){
      pcl::fromROSMsg(srv.response.cloud, *cloud);
      pcl::io::savePCDFile("src/ods_skin_color/data/skin_color/depth_registered.pcd", *cloud);
  }*/
  srv2.request.id = 4;
  if (commander_to_kinect_capture.call(srv2))
  {
    rgb_ptr = cv_bridge::toCvCopy(srv2.response.image, sensor_msgs::image_encodings::BGR8);
    cv::imwrite("src/ods_skin_color/data/skin_color/rgb_image.png", rgb_ptr->image);
  }

  //画素値と距離値の対応付け
  // pcl::removeNaNFromPointCloud(*cloud, *tmp_cloud, tmp_idx);
  // GetDepth(tmp_idx);

  //肌色抽出
  frame = rgb_ptr->image.clone();
  IplImage Iplimg = frame;
  extraction(Iplimg, mask);
  cv::imwrite("src/ods_skin_color/data/skin_color/frame.png", frame);
  cv::imwrite("src/ods_skin_color/data/skin_color/mask.png", mask);

  std::cout << "bbb" << std::endl;
  //顔方向の決定
  if (count >= 0)
    res.direction = 2;  // right
  else
    res.direction = 2;  // left
  std::cout << res.direction << " " << count << std::endl;

  std::cout << "aaa" << std::endl;
  res.image = srv2.response.image;

  return true;
}

int main(int argc, char **argv)
{
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "ods_skin_color");
  ros::NodeHandle n;

  service = n.advertiseService("ods_skin_color", skin_extraction);

  commander_to_kinect_capture = n.serviceClient< tms_msg_ss::ods_pcd >("ods_capture");

  ros::spin();

  return 0;
}
