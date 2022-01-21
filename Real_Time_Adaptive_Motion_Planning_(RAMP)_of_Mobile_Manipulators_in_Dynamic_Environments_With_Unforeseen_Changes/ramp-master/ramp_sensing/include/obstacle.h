#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/Obstacle.h"
#include <tf/transform_datatypes.h>
#include <vector>
#include "utility.h"
#include "circle_packer.h"

class Obstacle 
{
  public:
    Obstacle();
    Obstacle(const nav_msgs::Odometry p);
    ~Obstacle(); 

    /** Data Members */
    
    ramp_msgs::Obstacle msg_;

    //Hold odometry information for t and t-1
    nav_msgs::Odometry odom_t;

    // Hold Circle information
    Circle cir_;

    //Time of last update
    ros::Time last_updated_;

    tf::Transform T_w_init_;
    

    /** Methods */

    void update(const nav_msgs::Odometry o);

    void update(const Circle c);

    void doTF();

  private:
    Utility utility_;
};

#endif
