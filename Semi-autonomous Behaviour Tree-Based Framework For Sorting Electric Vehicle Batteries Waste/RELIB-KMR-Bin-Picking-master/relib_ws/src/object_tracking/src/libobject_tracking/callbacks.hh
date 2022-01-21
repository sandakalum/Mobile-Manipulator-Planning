#ifndef VISP_TRACKER_CALLBACKS_HH
# define VISP_TRACKER_CALLBACKS_HH
# include <boost/thread/recursive_mutex.hpp>
# include <image_transport/image_transport.h>
# include <sensor_msgs/Image.h>
# include <std_msgs/Header.h>

# include <string>

# include <visp3/core/vpImage.h>
# include <visp3/mbt/vpMbGenericTracker.h>
# include <visp3/me/vpMe.h>
# include <visp3/klt/vpKltOpencv.h>

# include <visp_tracker/ModelBasedSettingsConfig.h>
# include <visp_tracker/ModelBasedSettingsKltConfig.h>
# include <visp_tracker/ModelBasedSettingsEdgeConfig.h>

void
imageCallback(vpImage<unsigned char>& image,
              const sensor_msgs::Image::ConstPtr& msg,
              const sensor_msgs::CameraInfoConstPtr& info);

void
imageCallback(vpImage<unsigned char>& image,
              std_msgs::Header& header,
              sensor_msgs::CameraInfoConstPtr& info,
              const sensor_msgs::Image::ConstPtr& msg,
              const sensor_msgs::CameraInfoConstPtr& infoConst);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image,
                  std_msgs::Header& header,
                  sensor_msgs::CameraInfoConstPtr& info);


void reconfigureCallback(vpMbGenericTracker &tracker,
                         vpImage<unsigned char>& I,
                         vpMe& moving_edge,
                         vpKltOpencv& kltTracker,
                         boost::recursive_mutex& mutex,
                         visp_tracker::ModelBasedSettingsConfig& config,
                         uint32_t level);

void reconfigureEdgeCallback(vpMbGenericTracker &tracker,
                             vpImage<unsigned char>& I,
                             vpMe& moving_edge,
                             boost::recursive_mutex& mutex,
                             visp_tracker::ModelBasedSettingsEdgeConfig& config,
                             uint32_t level);

void reconfigureKltCallback(vpMbGenericTracker &tracker,
                            vpImage<unsigned char>& I,
                            vpKltOpencv& kltTracker,
                            boost::recursive_mutex& mutex,
                            visp_tracker::ModelBasedSettingsKltConfig& config,
                            uint32_t level);

void reInitViewerCommonParameters(ros::NodeHandle& nh,
                                  vpMbGenericTracker &tracker);

void reconfigureCallbackAndInitViewer(ros::NodeHandle& nh,
                                      vpMbGenericTracker &tracker,
                                      vpImage<unsigned char>& I,
                                      vpMe& moving_edge,
                                      vpKltOpencv& kltTracker,
                                      boost::recursive_mutex& mutex,
                                      visp_tracker::ModelBasedSettingsConfig& config,
                                      uint32_t level);

void reconfigureEdgeCallbackAndInitViewer(ros::NodeHandle& nh,
                                          vpMbGenericTracker &tracker,
                                          vpImage<unsigned char>& I,
                                          vpMe& moving_edge,
                                          boost::recursive_mutex& mutex,
                                          visp_tracker::ModelBasedSettingsEdgeConfig& config,
                                          uint32_t level);

void reconfigureKltCallbackAndInitViewer(ros::NodeHandle& nh,
                                         vpMbGenericTracker &tracker,
                                         vpImage<unsigned char>& I,
                                         vpKltOpencv& kltTracker,
                                         boost::recursive_mutex& mutex,
                                         visp_tracker::ModelBasedSettingsKltConfig& config,
                                         uint32_t level);


#endif //! VISP_TRACKER_CALLBACKS_HH
