/*
 * A simple 2D gridmap structure
 *
 * Copyright 2011 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef GRIDMAP2D_GRIDMAP2D_H_
#define GRIDMAP2D_GRIDMAP2D_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/OccupancyGrid.h>





namespace gridmap_2d{
/**
 * @brief Stores a nav_msgs::OccupancyGrid in a convenient opencv cv::Mat
 * as binary map (free: 255, occupied: 0) and as distance map (distance
 * to closest obstacle in meter).
 */
class GridMap2D {
public:
  GridMap2D();
  ///@brief Create from nav_msgs::OccupancyGrid
  GridMap2D(const nav_msgs::OccupancyGridConstPtr& grid_map, bool unknown_as_obstacle = false);
  ///@brief Copy constructor, performs a deep copy of underlying data structures
  GridMap2D(const GridMap2D& other);
  virtual ~GridMap2D();

  void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
  void worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const;

  /// check if a coordinate is covered by the map extent (same as worldToMap)
  bool inMapBounds(double wx, double wy) const;

  /**
   * Inflate occupancy map by inflationRadius
   */
  void inflateMap(double inflationRaduis);

  /// Distance (in m) between two map coordinates (indices)
  inline double worldDist(unsigned x1, unsigned y1, unsigned x2, unsigned y2){
    return worldDist(cv::Point(x1, y1), cv::Point(x2, y2));
  }

  inline double worldDist(const cv::Point& p1, const cv::Point& p2){
    return GridMap2D::pointDist(p1, p2) * m_mapInfo.resolution;
  }

  /// Euclidean distance between two points:
  static inline double pointDist(const cv::Point& p1, const cv::Point& p2){
    return sqrt(pointDist2(p1, p2));
  }

  /// Squared distance between two points:
  static inline double pointDist2(const cv::Point& p1, const cv::Point& p2){
    return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
  }

  /// Returns distance (in m) at world coordinates <wx,wy> in m; -1 if out of bounds!
  float distanceMapAt(double wx, double wy) const;

  /// Returns distance (in m) at map cell <mx, my> in m; -1 if out of bounds!
  float distanceMapAtCell(unsigned int mx, unsigned int my) const;

  /// Returns map value at world coordinates <wx, wy>; out of bounds will be returned as 0!
  uchar binaryMapAt(double wx, double wy) const;

  /// Returns map value at map cell <mx, my>; out of bounds will be returned as 0!
  uchar binaryMapAtCell(unsigned int mx, unsigned int my) const;

  /// Returns map value at map cell <mx, my>; out of bounds will be returned as 0!
  uchar& binaryMapAtCell(unsigned int mx, unsigned int my);

  /// @return true if map is occupied at world coordinate <wx, wy>. Out of bounds
  /// 		will be returned as occupied.
  bool isOccupiedAt(double wx, double wy) const;

  /// @return true if map is occupied at cell <mx, my>
  bool isOccupiedAtCell(unsigned int mx, unsigned int my) const;

  ///@brief Initialize map from a ROS OccupancyGrid message
  void setMap(const nav_msgs::OccupancyGridConstPtr& grid_map, bool unknown_as_obstacle = false);

  ///@brief Converts back into a ROS nav_msgs::OccupancyGrid msg
  nav_msgs::OccupancyGrid toOccupancyGridMsg() const;

  ///@brief Initialize from an existing cv::Map. mapInfo (in particular resolution) remains the same!
  void setMap(const cv::Mat& binary_map);

  ///@brief Recalculate the internal distance map. Required after manual changes to the grid map data.
  void updateDistanceMap();

  inline const nav_msgs::MapMetaData& getInfo() const {return m_mapInfo;}
  inline float getResolution() const {return m_mapInfo.resolution; }
  /// returns the tf frame ID of the map (usually "/map")
  inline const std::string getFrameID() const {return m_frameId;}
  /// @return the cv::Mat distance image.
  const cv::Mat& distanceMap() const {return m_distMap;}
  /// @return the cv::Mat binary image.
  const cv::Mat& binaryMap() const {return m_binaryMap;}
  /// @return the size of the cv::Mat binary image. Note that x/y are swapped wrt. height/width
  inline const CvSize size() const {return m_binaryMap.size();}

  const static uchar FREE = 0;  ///< char value for "free": 255
  const static uchar OCCUPIED = 100; ///< char value for "free": 0

protected:
  cv::Mat m_binaryMap;	///< binary occupancy map. 255: free, 0 occupied.
  cv::Mat m_distMap;		///< distance map (in meter)
  nav_msgs::MapMetaData m_mapInfo;
  std::string m_frameId;	///< "map" frame where ROS OccupancyGrid originated from

};

typedef boost::shared_ptr< GridMap2D> GridMap2DPtr;
typedef boost::shared_ptr<const GridMap2D> GridMap2DConstPtr;
}

#endif /* GRIDMAP2D_H_ */
