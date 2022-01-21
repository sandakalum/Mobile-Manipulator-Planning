#include "collision_detection.h"


CollisionDetection::CollisionDetection() {}

CollisionDetection::~CollisionDetection() 
{}

void CollisionDetection::init() {}


void CollisionDetection::performNum(const ramp_msgs::RampTrajectory& trajectory, const std::vector<ramp_msgs::RampTrajectory>& obstacle_trjs, const double& coll_dist, QueryResult& result)
{
  result.collision_ = false;
  for(uint8_t i=0;i<obstacle_trjs.size() && !result.collision_;i++)
  {
    //////ROS_INFO("Ob traj: %s", utility_.toString(obstacle_trjs[i]).c_str());
    query(trajectory.trajectory.points, obstacle_trjs[i].trajectory.points, trajectory.t_start.toSec(), coll_dist, result);
  }
}



/** Returns true if trajectory_ is in collision with any of the objects */
void CollisionDetection::perform(const ramp_msgs::RampTrajectory& trajectory, const std::vector<ramp_msgs::RampTrajectory>& obstacle_trjs, QueryResult& result)  
{
  ////////ROS_INFO("In CollisionDetection::perform()");
  ros::Duration d_points, d_getMisc, d_linearc;
  ros::Time t_start = ros::Time::now();

  ////////ROS_INFO("trajectory.curves.size(): %i", (int)trajectory.curves.size());
  size_t s = obstacle_trjs.size(); 
  size_t s_segment = trajectory.curves.size() > 0 ? 
    (2 * (trajectory.curves.size()+1) < trajectory.i_knotPoints.size()
    ? 
    2 * (trajectory.curves.size()+1) 
    :
    trajectory.i_knotPoints.size())
    : 2;
  uint8_t segment;
  int8_t ob_i;
  std::vector< std::vector<double> > points_of_collision;
 
  ////////ROS_INFO("s_segment: %i", (int)s_segment);

  double radius = 0.5f;
 
  std::vector<double> t;
  double t_final;
 
  ////////ROS_INFO("obstacle_trjs_.size(): %i", (int)obstacle_trjs.size());

  // Predict the obstacle's trajectory
  for(ob_i=0;ob_i<s;ob_i++)
  {
    ////////ROS_INFO("ob_i: %i", (int)ob_i);

    ros::Time t_points = ros::Time::now();
    const trajectory_msgs::JointTrajectoryPoint& ob_a = obstacle_trjs[ob_i].trajectory.points[0];
    const trajectory_msgs::JointTrajectoryPoint& ob_b = obstacle_trjs[ob_i].trajectory.points[ 
        obstacle_trjs[ob_i].trajectory.points.size()-1 ];
    bool ob_trj_line = fabs(utility_.findDistanceBetweenAngles(ob_a.positions[2], ob_b.positions[2])) < 0.01;
    d_points = ros::Time::now() - t_points;

    double ob_v = sqrt( ob_a.velocities[0]*ob_a.velocities[0] + ob_a.velocities[1]*ob_a.velocities[1] );
    double ob_w = ob_a.velocities[2];
    bool ob_moving = (ob_v > 0.01 || ob_w*ob_w > 0.0001);

    /*//////ROS_INFO("ob_a: %s", utility_.toString(ob_a).c_str());
    //////ROS_INFO("ob_b: %s", utility_.toString(ob_b).c_str());
    //////ROS_INFO("ob_v: %f ob_w: %f ob_moving: %s", ob_v, ob_w, ob_moving ? "True" : "False");*/

    //d_for = ros::Time::now()-t_for;
    //t_inner_for = ros::Time::now();
    for(segment=1;segment<s_segment && points_of_collision.size()==0;segment++)
    {
      ////////ROS_INFO("Segment %i", segment);

      // Check a point in the middle of the segment for angular velocity
      // If there's no angular velocity, then it's a straight-line, otherwise it's a curve
      ros::Time t_getMisc = ros::Time::now();
      uint16_t index_start  = trajectory.i_knotPoints[segment-1];
      uint16_t index_end    = trajectory.i_knotPoints[segment]; 
      ////////ROS_INFO("trajectory.i_knotPoints[%i]: %i trajectory.i_knotPoints[%i]: %i", segment-1, trajectory.i_knotPoints[segment-1], segment, trajectory.i_knotPoints[segment]);

      const trajectory_msgs::JointTrajectoryPoint* a = &trajectory.trajectory.points[index_start];
      const trajectory_msgs::JointTrajectoryPoint* b = &trajectory.trajectory.points[index_end];

      double v_start = sqrt( a->velocities[0]*a->velocities[0] + a->velocities[1]*a->velocities[1] );
      double v_end = sqrt( b->velocities[0]*b->velocities[0] + b->velocities[1]*b->velocities[1] );
      double w_start = a->velocities[2];
      double w_end = b->velocities[2];
      d_getMisc = ros::Time::now()-t_getMisc;
      
      bool segment_bezier = (v_end > 0.01 && w_end*w_end > 0.0001) && trajectory.curves.size() > 0;

      ////////ROS_INFO("a: %s", utility_.toString(*a).c_str());
      ////////ROS_INFO("b: %s", utility_.toString(*b).c_str());
      ////////ROS_INFO("v_start: %f v_end: %f w_start: %f w_end: %f", v_start, v_end, w_start, w_end);
      ////////ROS_INFO("segment_bezier: %s", segment_bezier ? "True" : "False");

      if(obstacle_trjs[ob_i].trajectory.points.size() == 1 && ob_v < 0.01 && ob_w*ob_w<0.0001)
      {

        if(!segment_bezier)
        {
          ////////ROS_INFO("Line No Motion");
          ros::Time t_start = ros::Time::now();
          LineNoMotion(trajectory, segment, ob_a, points_of_collision);
          t_ln.push_back(ros::Time::now() - t_start);

          std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator first = 
            trajectory.trajectory.points.begin() + trajectory.i_knotPoints[segment-1];
          
          std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator last = 
            trajectory.trajectory.points.begin() + trajectory.i_knotPoints[segment];

          std::vector<trajectory_msgs::JointTrajectoryPoint> segment_points(first, last);
          
          std::vector<trajectory_msgs::JointTrajectoryPoint> ob;
          ob.push_back(ob_a);

          t_start = ros::Time::now();
          query(segment_points, ob, points_of_collision);
          t_ln_num.push_back(ros::Time::now() - t_start);

        }
        else
        {
          ////////ROS_INFO("Bezier No Motion");
          ros::Time t_start = ros::Time::now();
          BezierNoMotion(trajectory.curves[0].controlPoints, ob_a, points_of_collision);
          t_bn.push_back(ros::Time::now() - t_start);
        }
      }

      // Straight-line
      else if( !segment_bezier )
      {
        // Line-Line
        if(ob_trj_line)
        {
          ////////ROS_INFO("Line Line");
          ros::Time t_start = ros::Time::now();
          LineLineFull(trajectory, segment, obstacle_trjs[ob_i], points_of_collision);
          t_ll.push_back(ros::Time::now() - t_start);

          std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator first = 
            trajectory.trajectory.points.begin() + trajectory.i_knotPoints[segment-1];
          
          std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator last = 
            trajectory.trajectory.points.begin() + trajectory.i_knotPoints[segment];

          std::vector<trajectory_msgs::JointTrajectoryPoint> segment_points(first, last);
          
          t_start = ros::Time::now();
          query(segment_points, obstacle_trjs[ob_i].trajectory.points, points_of_collision);
          t_ll_num.push_back(ros::Time::now() - t_start);
         
        }
        // Line-Arc
        else
        {
          ////////ROS_INFO("Line-Arc");
          ros::Time t_start = ros::Time::now();
          LineArcFull(trajectory, segment, obstacle_trjs[ob_i], points_of_collision);
          t_la.push_back(ros::Time::now() - t_start);
        }
      }
      // Bezier curve
      else
      {
        // Bezier-Line
        if(ob_trj_line)
        {
          ////////ROS_INFO("Bezier Line");
          ros::Time t_start = ros::Time::now();
          BezierLineFull(trajectory.curves[0].controlPoints, obstacle_trjs[ob_i], points_of_collision);
          t_bl.push_back(ros::Time::now() - t_start);
        }
        // Bezier-Arc
        else
        {
          ////////ROS_INFO("Bezier Arc");
          ros::Time t_start = ros::Time::now();
          BezierArc(trajectory.curves[0].controlPoints, obstacle_trjs[ob_i], points_of_collision); 
          t_ba.push_back(ros::Time::now() - t_start);
        }
      }
      //////////ROS_INFO("Segment %i Collision: %s", segment, result.collision_ ? "True" : "False");
    } // end for segment

    ////////ROS_INFO("points_of_collision.size(): %d", (int)points_of_collision.size());
    
    for(int p=0;p<points_of_collision.size();p++)
    {
      std::vector<double>& p_intersect = points_of_collision[p];
      ////////ROS_INFO("Point p: (%f, %f)", p_intersect[0], p_intersect[1]);

      int index = findIndexOfCollision(trajectory, p_intersect);
      ////////ROS_INFO("index: %d", index);

      trajectory_msgs::JointTrajectoryPoint traj_p_at_coll = trajectory.trajectory.points[index];

      if(index < obstacle_trjs[ob_i].trajectory.points.size() || obstacle_trjs[ob_i].trajectory.points.size() == 1)
      {
        trajectory_msgs::JointTrajectoryPoint ob_traj_p_at_coll;
        if(obstacle_trjs[ob_i].trajectory.points.size() == 1)
        {
          ob_traj_p_at_coll = obstacle_trjs[ob_i].trajectory.points[0];
        }
        else
        {
          ob_traj_p_at_coll = obstacle_trjs[ob_i].trajectory.points[index];
        }
        
        /*//////ROS_INFO("traj_point: %s", utility_.toString(traj_p_at_coll).c_str());
        //////ROS_INFO("ob_traj_point: %s", utility_.toString(ob_traj_p_at_coll).c_str());
        //////ROS_INFO("dist: %f", utility_.positionDistance( traj_p_at_coll.positions, ob_traj_p_at_coll.positions ));*/

        if( utility_.positionDistance( traj_p_at_coll.positions, ob_traj_p_at_coll.positions ) < 2.f*radius )
        {
          ////////ROS_INFO("Pushing on index: %i", index);
          t.push_back(index / 10.f);
        }
      } // end if
    } // end for each point
    //d_inner_for = ros::Time::now()-t_inner_for;
  } // end for obstacle */
    
  ////////ROS_INFO("t.size(): %i", (int)t.size());
  
  // Find the minimum time of collision
  if(t.size()>0)
  {
    t_final = t[0];
    for(uint8_t p=1;p<t.size();p++)
    {
      ////////ROS_INFO("t[%i]: %f", (int)p, t[p]);
      if(t[p] < t_final)
      {
        t_final = t[p];
      }
    }

    result.collision_         = true;
    result.t_firstCollision_  = t_final;
  }
  else
  {
    result.collision_ = false;
  }
    

  ros::Time t_done = ros::Time::now();
  //////////ROS_INFO("t_for: %f", d_for.toSec());
  //////////ROS_INFO("t_inner_for: %f", d_inner_for.toSec());
  /*////////ROS_INFO("t_points: %f", d_points.toSec());
  ////////ROS_INFO("t_getMisc: %f", d_getMisc.toSec());
  ////////ROS_INFO("t_linearc: %f", d_linearc.toSec());
  ////////ROS_INFO("t_perform(CD): %f", (t_done - t_start).toSec());*/
  //////////ROS_INFO("Exiting CollisionDetection::perform()");
} //End perform*/


int CollisionDetection::findIndexOfCollision(const ramp_msgs::RampTrajectory& trajectory, std::vector<double>& collision_point) const
{
  int index = (rand() % trajectory.trajectory.points.size());  
  int count = 0;
  ////////ROS_INFO("Collision points: (%f, %f)", collision_point[0], collision_point[1]);
  ////////ROS_INFO("index: %i", index);

  if(trajectory.trajectory.points.size() == 1)
  {
    return 0;
  }

  double d_left, d_right;
  while(fabs(utility_.positionDistance( trajectory.trajectory.points[index].positions, collision_point)) > 0.02 && count < 30)
  {
    /*//////ROS_INFO("count: %i", count);
    //////ROS_INFO("Point: %s", utility_.toString(trajectory.trajectory.points[index]).c_str());
    //////ROS_INFO("dist: %f", fabs(utility_.positionDistance( trajectory.trajectory.points[index].positions, collision_point)));*/
    
    if(index > 0)
    {
      d_left   = fabs(utility_.positionDistance( trajectory.trajectory.points[index-1].positions, collision_point));
    }
    else
    {
      d_left   = fabs(utility_.positionDistance( trajectory.trajectory.points[index].positions, collision_point));
    }
    
    if(index+1 < trajectory.trajectory.points.size())
    {
      d_right  = fabs(utility_.positionDistance( trajectory.trajectory.points[index+1].positions, collision_point));
    }
    else
    {
      d_right  = fabs(utility_.positionDistance( trajectory.trajectory.points[index].positions, collision_point));
    }
    
    ////////ROS_INFO("d_left: %f d_right: %f", d_left, d_right);

    if(d_left < d_right)
    {
      if(index > ceil(d_left*10))
      {
        index -= ceil(d_left*10);
      }
      else if(index == 0)
      {
        return 0;
      }
      else
      {
        index = 0;
      }
    }
    else
    {
      if( (index+ceil(d_right*10)) < trajectory.trajectory.points.size())
      {
        index += ceil(d_right*10);
      }
      else if(index == trajectory.trajectory.points.size())
      {
        return index;
      }
      else
      {
        index = trajectory.trajectory.points.size()-1;
      }
    }

    count++;
    ////////ROS_INFO("New index: %d", index);
  }

  return index;
}
    


void CollisionDetection::NoMotionNoMotion(const trajectory_msgs::JointTrajectoryPoint& a, const trajectory_msgs::JointTrajectoryPoint b,  std::vector< std::vector<double> >& points_of_collision) const
{

  double dist = sqrt( pow(b.positions[0] - a.positions[0],2) + pow(b.positions[0] - a.positions[0],2) );
  double radius = 0.5f;
  if(dist < radius*2.f)
  {
    points_of_collision.push_back(a.positions);
  }
}



void CollisionDetection::BezierArc(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const
{
  ros::Time t_start = ros::Time::now();

  //////////ROS_INFO("w: %f v: %f r: %f", w, v, r);


  // Start sub-dividing the CP
  bool coll_array[7] = {false, false, false, false, false, false, false};
  
  
  //ros::Time t_tree = ros::Time::now();
  std::vector< std::vector<ramp_msgs::MotionState> > control_poly_tree;
  buildTree(control_points, 2, control_poly_tree);
  //ros::Duration d_tree = ros::Time::now()-t_tree;


  int s = control_poly_tree.size();
  //ros::Time t_for = ros::Time::now();
  //////////ROS_INFO("control_poly_tree size: %i", (int)control_poly_tree.size());
  for(int i=0;i<s;i++)
  {
    ////////ROS_INFO("Testing collision on control polygon %i", i);
    std::vector<ramp_msgs::MotionState> control_poly = control_poly_tree.at(i);
    /*////////ROS_INFO("control_poly:");
    for(int j=0;j<control_poly.size();j++)
    {
      ////////ROS_INFO("Vertex %i: %s", i, utility_.toString(control_poly.at(j)).c_str());
    }*/
    if(i > 2)
    {
      ControlPolyArc(control_poly_tree[i], ob_trajectory, coll_array[i], points_of_collision);
    }
    else
    {
      std::vector< std::vector<double> > temp;
      //ros::Time t_coll = ros::Time::now();
      ControlPolyArc(control_poly_tree[i], ob_trajectory, coll_array[i], temp);
      //ros::Duration d_coll = ros::Time::now() - t_coll;
      ////////ROS_INFO("d_coll: %f", d_coll.toSec());
    }

    ////////ROS_INFO("collision at polygon %i: %s", i, coll_array[i] ? "True" : "False");

    // If no collision with the initial control polygon, return false
    if(!coll_array[i] && i == 0)
    {
      ////////ROS_INFO("In 1st if");
      break;
    }
    // If no collision with depth-1 polygons
    else if(i == 2 && !coll_array[i] && !coll_array[i-1])
    {
      ////////ROS_INFO("In 2nd if");
      break;
    }
    // If collision with any depth-n level polygons, return true
    else if(coll_array[i] && i > 2)
    {
      ////////ROS_INFO("In 3rd if");
      break;
    }
  } // end for
  //ros::Duration d_for = ros::Time::now() - t_for;
  //ros::Duration d_total = ros::Time::now() - t_start;
  
  ////////ROS_INFO("t_tree: %f", d_tree.toSec());
  //////////ROS_INFO("t_for: %f", d_for.toSec());
  //////////ROS_INFO("t_BezierArc: %f", d_total.toSec());
} // End BezierArc


void CollisionDetection::buildTree(const std::vector<ramp_msgs::MotionState>& control_poly, const int& depth, std::vector< std::vector<ramp_msgs::MotionState> >& result ) const
{

  result.push_back(control_poly);

  std::vector< std::vector<ramp_msgs::MotionState> > subdivide;
  for(int i=0;i<pow(2,depth);i++)
  {
    subdivide.clear();
    deCasteljau(result.at(i), subdivide);

    result.push_back(subdivide[0]);
    result.push_back(subdivide[1]);
  }
}



void CollisionDetection::deCasteljau(const std::vector<ramp_msgs::MotionState>& control_poly, std::vector< std::vector<ramp_msgs::MotionState> >& result) const
{
  std::vector<ramp_msgs::MotionState> result_left;
  std::vector<ramp_msgs::MotionState> result_right;

  double t = 0.5f;

  double x_b_1_0 = (1-t)*control_poly[0].positions[0] + t*control_poly[1].positions[0];
  double y_b_1_0 = (1-t)*control_poly[0].positions[1] + t*control_poly[1].positions[1];

  double x_b_1_1 = (1-t)*control_poly[1].positions[0] + t*control_poly.at(2).positions[0];
  double y_b_1_1 = (1-t)*control_poly[1].positions[1] + t*control_poly.at(2).positions[1];

  double x_b_2_0 = (1-t)*x_b_1_0 + t*x_b_1_1;
  double y_b_2_0 = (1-t)*y_b_1_0 + t*y_b_1_1;

  ramp_msgs::MotionState b_1_0;
  b_1_0.positions.push_back(x_b_1_0);
  b_1_0.positions.push_back(y_b_1_0);

  ramp_msgs::MotionState b_1_1;
  b_1_1.positions.push_back(x_b_1_1);
  b_1_1.positions.push_back(y_b_1_1);

  ramp_msgs::MotionState b_2_0;
  b_2_0.positions.push_back(x_b_2_0);
  b_2_0.positions.push_back(y_b_2_0);

  result_left.push_back( control_poly[0] );
  result_left.push_back( b_1_0 );
  result_left.push_back( b_2_0 );

  result_right.push_back( b_2_0 );
  result_right.push_back( b_1_1 );
  result_right.push_back( control_poly[2] );

  result.push_back(result_left);
  result.push_back(result_right);
}


void CollisionDetection::BezierCircle(const std::vector<ramp_msgs::MotionState>& con_poly_vert, const std::vector<double>& cir_cent, const double& r, bool& result, std::vector< std::vector<double> >& points_of_collision) const
{

  int start, end;
  double x_com, y_com;
  double sum_radii = 0.4;
  
  std::vector<double> l_p1, l_p2;

  // Line-Arc for each segment on control polygon
  for(int i=0;i<con_poly_vert.size();i++)
  {
    l_p1.clear();
    l_p2.clear();

    ////////ROS_INFO("Control Polygon Edge %i", i);
    start =  i;
    end   = (i == con_poly_vert.size()-1) ? 0 : i+1;
    

    l_p1.push_back(con_poly_vert[start].positions[0]);
    l_p1.push_back(con_poly_vert[start].positions[1]);
    
    l_p2.push_back(con_poly_vert[end].positions[0]);
    l_p2.push_back(con_poly_vert[end].positions[1]);
    
    ////////ROS_INFO("l_p1: (%f, %f)", l_p1.at(0), l_p1.at(1));
    ////////ROS_INFO("l_p2: (%f, %f)", l_p2.at(0), l_p2.at(1));
    
    LineCircle(l_p1, l_p2, cir_cent, r, result, points_of_collision);
  } // end for
}



void CollisionDetection::ControlPolyArc(const std::vector<ramp_msgs::MotionState>& con_poly_vert, const ramp_msgs::RampTrajectory& ob_trajectory, bool& result, std::vector< std::vector<double> >& points_of_collision) const
{
  
  int start, end;
  double x_com, y_com;
  double sum_radii = 0.4;
  
  std::vector<trajectory_msgs::JointTrajectoryPoint> arc_points, pos_dir, neg_dir;
  arc_points.reserve(3); 
  pos_dir.reserve(3);
  neg_dir.reserve(3);

  std::vector<double> l_p1, l_p2;

  // Line-Arc for each segment on control polygon
  for(int i=0;i<con_poly_vert.size();i++)
  {
    arc_points.clear();
    pos_dir.clear();
    neg_dir.clear();
    l_p1.clear();
    l_p2.clear();

    //////////ROS_INFO("Control Polygon Edge %i", i);
    start =  i;
    end   = (i == con_poly_vert.size()-1) ? 0 : i+1;
    
    l_p1.push_back(con_poly_vert[start].positions[0]);
    l_p1.push_back(con_poly_vert[start].positions[1]);
    
    l_p2.push_back(con_poly_vert[end].positions[0]);
    l_p2.push_back(con_poly_vert[end].positions[1]);
    
    //////////ROS_INFO("l_p1: (%f, %f)", l_p1.at(0), l_p1.at(1));
    //////////ROS_INFO("l_p2: (%f, %f)", l_p2.at(0), l_p2.at(1));
    
    arc_points.push_back(ob_trajectory.trajectory.points[0]);

    arc_points.push_back(ob_trajectory.trajectory.points[
        ob_trajectory.trajectory.points.size()/2] );

    arc_points.push_back(ob_trajectory.trajectory.points[
        ob_trajectory.trajectory.points.size()-1] );
    
    LineArc(l_p1, l_p2, arc_points, points_of_collision, result);

    x_com = cos(arc_points[0].positions[2]);
    y_com = sin(arc_points[0].positions[2]);
    ////////ROS_INFO("x_com: %f y_com: %f norm: %f", x_com, y_com, sqrt( (x_com*x_com) + (y_com*y_com) ));


    pos_dir = arc_points;
    // Positive translation for new arc points
    for(uint8_t i=0;i<arc_points.size();i++)
    {
      pos_dir[i].positions[0] += sum_radii*x_com;
      pos_dir[i].positions[1] += sum_radii*y_com;
      ////////ROS_INFO("New arc point: (%f, %f)", pos_dir[i].positions[0], pos_dir[i].positions[1]);
    }

    LineArc(l_p1, l_p2, pos_dir, points_of_collision, result);
    ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
    
    neg_dir = arc_points;
    // Negative translation for new arc points
    for(uint8_t i=0;i<arc_points.size();i++)
    {
      neg_dir[i].positions[0] -= sum_radii*x_com;
      neg_dir[i].positions[1] -= sum_radii*y_com;
      ////////ROS_INFO("New arc point: (%f, %f)", neg_dir[i].positions[0], neg_dir[i].positions[1]);
     }

    LineArc(l_p1, l_p2, neg_dir, points_of_collision, result);
  
    // Boundary lines creating the fan
    LineLineEndPoints(l_p1, l_p2, pos_dir[0].positions, neg_dir[0].positions, points_of_collision); 
    ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
    
    LineLineEndPoints(l_p1, l_p2, pos_dir[2].positions, neg_dir[2].positions, points_of_collision); 
    ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  } // end for
} // End ControlPolyArc 


void CollisionDetection::LineNoMotion(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const trajectory_msgs::JointTrajectoryPoint& ob_state, std::vector< std::vector<double> >& points_of_collision) const
{
  ////////ROS_INFO("In CollisionDetection::LineNoMotion");

  // Line 1
  //std::vector<double> l_p1 = trajectory.trajectory.points.at( trajectory.i_knotPoints.at(segment-1)).positions;
  double l_p1_x = trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[0];

  double l_p1_y = trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[1];
  
  double l_p2_x = trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[0];

  double l_p2_y = trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[1];
 
  //std::vector<double> l_p2 = trajectory.trajectory.points.at( trajectory.i_knotPoints.at(segment)).positions;
  
  double sum_radii = 0.41;
  
  bool temp;
  LineCircle(trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions, trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions, ob_state.positions, sum_radii, temp, points_of_collision);

  /*double l_slope = (l_p2_y - l_p1_y) / (l_p2_x - l_p1_x);
  double l_b = l_p2_y - (l_slope*l_p2_x);

  double substitution = l_slope*ob_state.positions[0] - ob_state.positions[1] + l_b;

  ////////ROS_INFO("Slope: %f b: %f substitution: %f", l_slope, l_b, substitution);

  if(fabs(substitution) < 0.01)
  {
    ////////ROS_INFO("In collision!");
    points_of_collision.push_back(ob_state.positions);
  }*/
}
 
void CollisionDetection::BezierNoMotion(const std::vector<ramp_msgs::MotionState>& control_points, const trajectory_msgs::JointTrajectoryPoint& ob_state, std::vector< std::vector<double> >& points_of_collision) const
{
  ////////ROS_INFO("In CollisionDetection::BezierNoMotion");
  ////////ROS_INFO("controlPoints.size(): %i", (int)control_points.size());

  bool coll_array[7] = {false, false, false, false, false, false, false};
  std::vector< std::vector<ramp_msgs::MotionState> > control_poly_tree;
  buildTree(control_points, 2, control_poly_tree);

  double sum_radii = 0.4;
  int s = control_poly_tree.size();
  for(int i=0;i<s;i++)
  {
    ////////ROS_INFO("i: %i", i);
    std::vector<ramp_msgs::MotionState> control_poly = control_poly_tree.at(i);
    /*//////ROS_INFO("control_poly:");
    for(int j=0;j<control_poly.size();j++)
    {
      //////ROS_INFO("Vertex %i: %s", i, utility_.toString(control_poly.at(j)).c_str());
    }*/
    
    if(i > 2)
    {
      BezierCircle(control_poly_tree[i], ob_state.positions, sum_radii, coll_array[i], points_of_collision);
    }
    else
    {
      std::vector< std::vector<double> > temp;
      //ros::Time t_coll = ros::Time::now();
      BezierCircle(control_poly_tree[i], ob_state.positions, sum_radii, coll_array[i], temp);
      //ros::Duration d_coll = ros::Time::now() - t_coll;
      ////////ROS_INFO("d_coll: %f", d_coll.toSec());
    }

    ////////ROS_INFO("collision at polygon %i: %s", i, coll_array[i] ? "True" : "False");

    if(!coll_array[i] && i == 0)
    {
      ////////ROS_INFO("In 1st if");
      break;
    }
    // If no collision with depth-1 polygons
    else if(i == 2 && !coll_array[i] && !coll_array[i-1])
    {
      ////////ROS_INFO("In 2nd if");
      break;
    }
    // If collision with any depth-n level polygons, return true
    else if(coll_array[i] && i > 2)
    {
      ////////ROS_INFO("In 3rd if");
      break;
    }
  } // end for


  ////////ROS_INFO("Exiting CollisionDetection::BezierNoMotion");
}

void CollisionDetection::LineLineEndPoints(const std::vector<double>& l1_p1, std::vector<double>& l1_p2, const std::vector<double>& l2_p1, std::vector<double>& l2_p2, std::vector< std::vector<double> >& points_of_collision) const
{
  ////////ROS_INFO("In CollisionDetection::LineLineEndPoints");
  ////////ROS_INFO("Endpoints: Line 1: (%f, %f) (%f, %f) Line 2: (%f, %f) (%f, %f)", l1_p1[0], l1_p1[1], l1_p2[0], l1_p2[1], l2_p1[0], l2_p1[1], l2_p2[0], l2_p2[1]);

  double l1_p1_x = l1_p1[0];
  double l1_p1_y = l1_p1[1];
  double l1_p2_x = l1_p2[0];
  double l1_p2_y = l1_p2[1];

  double l1_slope = (l1_p2_y - l1_p1_y) / (l1_p2_x - l1_p1_x);
  double l1_b = l1_p2_y - (l1_slope*l1_p2_x);
  
  double l2_p1_x = l2_p1[0];
  double l2_p1_y = l2_p1[1];
  double l2_p2_x = l2_p2[0];
  double l2_p2_y = l2_p2[1];

  double l2_slope = (l2_p2_y - l2_p1_y) / (l2_p2_x - l2_p1_x);
  double l2_b = l2_p2_y - (l2_slope*l2_p2_x);
  
  // Parallel lines
  // Check that they are not the same line!
  if( fabs(l2_slope - l1_slope) < 0.01 )
  {
    ////////ROS_INFO("Lines are parallel");
    
    // Same line
    if( fabs(l1_slope - l2_slope) < 0.01 && fabs(l1_b - l2_b) < 0.01 )
    {
      std::vector<double> temp;
      temp.push_back(9999);
      temp.push_back(9999);
      points_of_collision.push_back(temp);
      //points_of_collision.push_back(trajectory.trajectory.points[ trajectory.trajectory.points.size()/2 ].positions);
    }
  }
  else
  {
    ////////ROS_INFO("Lines are not parallel");
    double x_intersect = (-(l1_b - l2_b)) / (l1_slope - l2_slope);
    double l1_x_min = l1_p1_x;
    double l1_x_max = l1_p2_x;
    double l2_x_min = l2_p1_x;
    double l2_x_max = l2_p2_x;

    if(l1_x_min > l1_x_max)
    {
      double temp = l1_x_min;
      l1_x_min = l1_x_max;
      l1_x_max = temp;
    }
    if(l2_x_min > l2_x_max)
    {
      double temp = l2_x_min;
      l2_x_min = l2_x_max;
      l2_x_max = temp;
    }

    ////////ROS_INFO("x_intersect: %f l1_x_min: %f l1_x_max: %f l2_x_min: %f l2_x_max: %f", x_intersect, l1_x_min, l1_x_max, l2_x_min, l2_x_max);

    if( (x_intersect >= l1_x_min && x_intersect <= l1_x_max) && (x_intersect >= l2_x_min && x_intersect <= l2_x_max) )
    {
      ////////ROS_INFO("In if lines intersect");

      std::vector<double> p_intersect;
      double x_at_inter = x_intersect;
      double y_at_inter = l1_slope*x_at_inter + l1_b;
      p_intersect.push_back(x_at_inter);
      p_intersect.push_back(y_at_inter);
      
      points_of_collision.push_back(p_intersect);
      ////////ROS_INFO("x_at_inter: %f y_at_inter: %f", x_at_inter, y_at_inter);
    }
  }
}


void CollisionDetection::LineLineFull(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const
{
  ////////ROS_INFO("In CollisionDetection::LineLine");

  ros::Time t_start = ros::Time::now();
  double t;

  /* Line - Line */

  // Line 1
  //std::vector<double> l1_p1 = trajectory.trajectory.points.at( trajectory.i_knotPoints.at(segment-1)).positions;
  double l1_p1_x = trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[0];

  double l1_p1_y = trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[1];
  
  double l1_p2_x = trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[0];

  double l1_p2_y = trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[1];
  
  //std::vector<double> l1_p2 = trajectory.trajectory.points.at( trajectory.i_knotPoints.at(segment)).positions;

  double l1_slope = (l1_p2_y - l1_p1_y) / (l1_p2_x - l1_p1_x);
  double l1_b = l1_p2_y - (l1_slope*l1_p2_x);

  ////////ROS_INFO("line 1 - slope: %f b: %f", l1_slope, l1_b);

  // Line 2
  //std::vector<double> l2_p1 = ob_trajectory.trajectory.points[0].positions;
  //std::vector<double> l2_p2 = ob_trajectory.trajectory.points[1].positions;
  double l2_p1_x = ob_trajectory.trajectory.points[ 0 ].positions[0];

  double l2_p1_y = ob_trajectory.trajectory.points[ 0 ].positions[1];
  
  double l2_p2_x = ob_trajectory.trajectory.points[ ob_trajectory.trajectory.points.size()-1 ].positions[0];

  double l2_p2_y = ob_trajectory.trajectory.points[ ob_trajectory.trajectory.points.size()-1 ].positions[1];

  double l2_slope = (l2_p2_y - l2_p1_y) / (l2_p2_x - l2_p1_x);
  double l2_b = l2_p2_y - (l2_slope*l2_p2_x);
  
  ////////ROS_INFO("line 2 - slope: %f b: %f", l2_slope, l2_b);
  
  std::vector<double> l1_p1; 
  l1_p1.push_back(l1_p1_x);
  l1_p1.push_back(l1_p1_y);
 
  std::vector<double> l1_p2; 
  l1_p2.push_back(l1_p2_x);
  l1_p2.push_back(l1_p2_y);
 
  std::vector<double> l2_p1; 
  l2_p1.push_back(l2_p1_x);
  l2_p1.push_back(l2_p1_y);
 
  std::vector<double> l2_p2; 
  l2_p2.push_back(l2_p2_x);
  l2_p2.push_back(l2_p2_y);

  //LineLineEndPoints(l1_p1, l1_p2, l2_p1, l2_p2, points_of_collision);

  // Get normal to central line
  
  ////////ROS_INFO("l2_p1: (%f, %f) l2_p2: (%f, %f)", l2_p1_x, l2_p1_x, l2_p2_x, l2_p2_y);
  double x_com = l2_p2_x - l2_p1_x;
  double y_com = l2_p2_y - l2_p1_y;
  double norm = sqrt( (x_com*x_com) + (y_com*y_com) );
  ////////ROS_INFO("x_com: %f y_com: %f norm: %f", x_com, y_com, norm);
  
  std::vector<double> n_k_pos;
  n_k_pos.push_back(y_com);
  n_k_pos.push_back(-x_com);
  
  std::vector<double> n_k_neg;
  n_k_neg.push_back(-y_com);
  n_k_neg.push_back(x_com);

  std::vector<double> unit_k_pos;
  unit_k_pos.push_back(n_k_pos[0]/norm);
  unit_k_pos.push_back(n_k_pos[1]/norm);

  std::vector<double> unit_k_neg;
  unit_k_neg.push_back(n_k_neg[0]/norm);
  unit_k_neg.push_back(n_k_neg[1]/norm);

  ////////ROS_INFO("n_k_pos: (%f, %f) n_k_neg: (%f, %f) unit_k_pos: (%f, %f) unit_k_neg: (%f, %f)", 
      //n_k_pos[0], n_k_pos[1], n_k_neg[0], n_k_neg[1], unit_k_pos[0], unit_k_pos[1], unit_k_neg[0], unit_k_neg[1]);


  double sum_radii = 0.4;
  std::vector<double> p_new, p_new_t;
  p_new.push_back(l2_p1_x + (sum_radii*(unit_k_pos[0])));
  p_new.push_back(l2_p1_y + (sum_radii*(unit_k_pos[1])));
 
  p_new_t.push_back(l2_p1_x + (sum_radii*unit_k_neg[0]));
  p_new_t.push_back(l2_p1_y + (sum_radii*unit_k_neg[1]));

  std::vector< std::vector<double> > rect_points;

  if(p_new[1] > p_new_t[1])
  {
    rect_points.push_back(p_new); 
    rect_points.push_back(l2_p1); 
    rect_points.push_back(p_new_t);
  } 
  else
  {
    rect_points.push_back(p_new_t);
    rect_points.push_back(l2_p1);
    rect_points.push_back(p_new);
  }


  // Get the last three points
  std::vector<double> p4, p5, p6;

  double dist = utility_.positionDistance(l2_p1, l2_p2);
  double theta = utility_.findAngleFromAToB(l2_p1, l2_p2);
  ////////ROS_INFO("dist: %f theta: %f", dist, theta);

  p4.push_back( rect_points[2][0] + dist*cos(theta) );
  p4.push_back( rect_points[2][1] + dist*sin(theta) );

  p6.push_back( rect_points[0][0] + dist*cos(theta) );
  p6.push_back( rect_points[0][1] + dist*sin(theta) );

  rect_points.push_back(p4);
  rect_points.push_back(l2_p2);
  rect_points.push_back(p6);

  ////////ROS_INFO("dist between p2 and p3: %f", utility_.positionDistance(rect_points[1], rect_points[2]));

  
  /*//////ROS_INFO("rect_points size: %i", (int)rect_points.size());
  for(uint8_t i=0;i<rect_points.size();i++)
  {
    //////ROS_INFO("p%d: (%f, %f)", (int)i, rect_points[i][0], rect_points[i][1]);
  }*/
  
  // Central line
  LineLineEndPoints(l1_p1, l1_p2, rect_points[0], rect_points[5], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());

  // Boundaries parallel to central
  LineLineEndPoints(l1_p1, l1_p2, rect_points[1], rect_points[4], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  LineLineEndPoints(l1_p1, l1_p2, rect_points[2], rect_points[3], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());

  // Boundaries orthogonal to central
  LineLineEndPoints(l1_p1, l1_p2, rect_points[3], rect_points[5], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  LineLineEndPoints(l1_p1, l1_p2, rect_points[0], rect_points[2], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());

  // Check if any of the points indicate the same line
  for(uint8_t i=0;i<points_of_collision.size();i++)
  {
    if(points_of_collision[i][0] == 9999)
    {
      ////////ROS_INFO("Found 9999");
      ////////ROS_INFO("Removing point: (%f, %f)", points_of_collision[i][0], points_of_collision[i][1]);
      points_of_collision.erase(points_of_collision.begin()+i);

      std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator first = trajectory.trajectory.points.begin() + trajectory.i_knotPoints[segment-1];
      std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator last = trajectory.trajectory.points.begin() + trajectory.i_knotPoints[segment];

      std::vector<trajectory_msgs::JointTrajectoryPoint> segment(first, last);

      ////////ROS_INFO("Calling query");
      query(segment, ob_trajectory.trajectory.points, points_of_collision);
      ////////ROS_INFO("Now collision point: (%f, %f)", points_of_collision[ points_of_collision.size()-1 ][0], points_of_collision[ points_of_collision.size()-1 ][1]);
    }
  }


  ////////ROS_INFO("Exiting CollisionDetection::LineLine");
  //////////ROS_INFO("Query Elapsed time: %f", (ros::Time::now()-t_start).toSec());
} // End LineLine



void CollisionDetection::BezierLineFull(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const
{

  std::vector<double> l_p1 = ob_trajectory.trajectory.points[0].positions;
  std::vector<double> l_p2 = ob_trajectory.trajectory.points[ ob_trajectory.trajectory.points.size()-1 ].positions;


  double l_p1_x = l_p1[0];
  double l_p1_y = l_p1[1];
  double l_p2_x = l_p2[0];
  double l_p2_y = l_p2[1];

  double x_com = l_p2_x - l_p1_x;
  double y_com = l_p2_y - l_p1_y;
  double norm = sqrt( (x_com*x_com) + (y_com*y_com) );
  
  std::vector<double> n_k_pos;
  n_k_pos.push_back(y_com);
  n_k_pos.push_back(-x_com);
  
  std::vector<double> n_k_neg;
  n_k_neg.push_back(-y_com);
  n_k_neg.push_back(x_com);

  std::vector<double> unit_k_pos;
  unit_k_pos.push_back(n_k_pos[0]/norm);
  unit_k_pos.push_back(n_k_pos[1]/norm);

  std::vector<double> unit_k_neg;
  unit_k_neg.push_back(n_k_neg[0]/norm);
  unit_k_neg.push_back(n_k_neg[1]/norm);


  ////////ROS_INFO("n_k_pos: (%f, %f) n_k_neg: (%f, %f) unit_k_pos: (%f, %f) unit_k_neg: (%f, %f)", 
      //n_k_pos[0], n_k_pos[1], n_k_neg[0], n_k_neg[1], unit_k_pos[0], unit_k_pos[1], unit_k_neg[0], unit_k_neg[1]);


  double sum_radii = 0.4;
  std::vector<double> p_new, p_new_t;
  p_new.push_back(l_p1_x + (sum_radii*(unit_k_pos[0])));
  p_new.push_back(l_p1_y + (sum_radii*(unit_k_pos[1])));
 
  p_new_t.push_back(l_p1_x + (sum_radii*unit_k_neg[0]));
  p_new_t.push_back(l_p1_y + (sum_radii*unit_k_neg[1]));

  std::vector< std::vector<double> > rect_points;

  if(p_new[1] > p_new_t[1])
  {
    rect_points.push_back(p_new); 
    rect_points.push_back(l_p1); 
    rect_points.push_back(p_new_t);
  } 
  else
  {
    rect_points.push_back(p_new_t);
    rect_points.push_back(l_p1);
    rect_points.push_back(p_new);
  }


  // Get the last three points
  std::vector<double> p4, p5, p6;

  double dist = utility_.positionDistance(l_p1, l_p2);
  double theta = utility_.findAngleFromAToB(l_p1, l_p2);
  ////////ROS_INFO("dist: %f theta: %f", dist, theta);

  p4.push_back( rect_points[2][0] + dist*cos(theta) );
  p4.push_back( rect_points[2][1] + dist*sin(theta) );

  p6.push_back( rect_points[0][0] + dist*cos(theta) );
  p6.push_back( rect_points[0][1] + dist*sin(theta) );

  rect_points.push_back(p4);
  rect_points.push_back(l_p2);
  rect_points.push_back(p6);

  ////////ROS_INFO("dist between p2 and p3: %f", utility_.positionDistance(rect_points[1], rect_points[2]));

  
  /*//////ROS_INFO("rect_points size: %i", (int)rect_points.size());
  for(uint8_t i=0;i<rect_points.size();i++)
  {
    //////ROS_INFO("p%d: (%f, %f)", (int)i, rect_points[i][0], rect_points[i][1]);
  }*/
  
  // Central line
  BezierLine(control_points, rect_points[0], rect_points[5], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());

  // Boundaries parallel to central
  BezierLine(control_points, rect_points[1], rect_points[4], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  BezierLine(control_points, rect_points[2], rect_points[3], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());

  // Boundaries orthogonal to central
  BezierLine(control_points, rect_points[3], rect_points[5], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  BezierLine(control_points, rect_points[0], rect_points[2], points_of_collision);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  
}

void CollisionDetection::BezierLine(const std::vector<ramp_msgs::MotionState>& control_points, const std::vector<double> l_p1, const std::vector<double> l_p2, std::vector< std::vector<double> >& points_of_collision) const
{
  ////////ROS_INFO("In CollisionDetection::BezierLine");
  ros::Time t_start = ros::Time::now();

  std::vector<double> u_intersection;
  double u_1=0, u_2=0;

  // Get values for Bezier control points
  double X0 = control_points[0].positions[0];
  double Y0 = control_points[0].positions[1];
  double X1 = control_points[1].positions[0];
  double Y1 = control_points[1].positions[1];
  double X2 = control_points[2].positions[0];
  double Y2 = control_points[2].positions[1];

  ////////ROS_INFO("Control Points (X0, Y0): (%f, %f) (X1, Y1): (%f, %f) (X2, Y2): (%f, %f)", X0, Y0, X1, Y1, X2, Y2);

  // Get values for line segment
  double x1 = l_p1[0];
  double y1 = l_p1[1];
  double x2 = l_p2[0];
  double y2 = l_p2[1];

  double slope = (y2-y1)/(x2-x1);

  ////////ROS_INFO("(x1, y1): (%f, %f) (x2, y2): (%f, %f) slope: %f", x1, y1, x2, y2, slope);


  double Q = slope*( X0 - 2*X1 + X2 );
  double S = -( Y0 - 2*Y1 + Y2 );
  double R = 2*slope*( X1 - X0 );
  double T = -2*( Y1 - Y0 );
  
  //////////ROS_INFO("Q: %f R: %f S: %f T: %f", Q, R, S, T);
  
  double A = S+Q;
  double B = R+T;
  double C = (slope*X0) - Y0 + y1 - (slope*x1);
  
  ////////ROS_INFO("A: %f B: %f C: %f", A, B, C);

  // Find values of u that have intersection
  if( fabs(A) < 0.0001 )
  {
    //////////ROS_INFO("A=0, -C/B: %f", -(C/B));
    double u = -C/B;
    double bezier_x = pow( (1-u), 2 )*X0 + 2*u*(1-u)*X1 + pow(u,2)*X2;
    //////////ROS_INFO("bezier_x: %f", bezier_x);
    u_intersection.push_back(u);
  }

  else if( fabs(C) < 0.0001 )
  {
    u_intersection.push_back(-B/A);
  }

  else if( fabs(B) < 0.0001 )
  {
    u_1 = sqrt(-C/A);
    u_2 = -sqrt(-C/A);
    u_intersection.push_back(u_1);
    u_intersection.push_back(u_2);
  }
  
  else
  {
    double discriminant = B*B - (4*A*C);

    ////////ROS_INFO("Discriminant: %f", discriminant);
    //////////ROS_INFO("sqrt(Discriminant): %f", sqrt(discriminant));

    u_1 = (-B + sqrt( discriminant )) / (2.f*A);
    u_2 = (-B - sqrt( discriminant )) / (2.f*A);
    u_intersection.push_back(u_1);
    u_intersection.push_back(u_2);
    /*////////ROS_INFO("-B - sqrt(discriminant): %f", -B - sqrt(discriminant));
    ////////ROS_INFO("2*A: %f", 2.f*A);
    ////////ROS_INFO("-B + sqrt(discriminant): %f", -B + sqrt(discriminant));
    ////////ROS_INFO("(-B + sqrt(discriminant)) / (2*A): %f", (-B + sqrt(discriminant) / (2.f*A)));
    ////////ROS_INFO("u_1: %f u_2: %f", u_1, u_2);*/
  }

  // Check intersection values against bounds
  double x_min = l_p1[0];
  double x_max = l_p2[0];
  if(x_min > x_max)
  {
    double temp = x_min;
    x_min = x_max;
    x_max = temp;
  }
  //////////ROS_INFO("x_min: %f x_max: %f", x_min, x_max);
  
  for(int i=0;i<u_intersection.size();i++)
  {
    double u = u_intersection[i];
    // If intersection values are within bezier domain
    if( (u > -0.0001f && u <= 1.f) )
    {
      double bezier_x = pow( (1-u), 2 )*X0 + 2*u*(1-u)*X1 + pow(u,2)*X2;
      
      // Check if the intersection value is also on the line segment
      if( bezier_x >= x_min && bezier_x <= x_max )
      {
        double bezier_y = pow( (1-u), 2 )*Y0 + 2*u*(1-u)*Y1 + pow(u,2)*Y2;
        
        std::vector<double> p_intersect;
        p_intersect.push_back(bezier_x);
        p_intersect.push_back(bezier_y);
        
        points_of_collision.push_back(p_intersect);      
      } // end inner if
    } // end outer if
  } // end for

  ////////ROS_INFO("Exiting CollisionDetection::BezierLine");
} // End BezierLine



void CollisionDetection::LineCircle(const std::vector<double>& l_p1, const std::vector<double>& l_p2, const std::vector<double>& cir_center, const double& r, bool& result, std::vector< std::vector<double> >& points_of_collision) const
{
  std::vector<double> x_intersection;

  double slope  = (l_p2.at(1) - l_p1.at(1)) / (l_p2.at(0) - l_p1.at(0));
  double b      = l_p2.at(1) - (slope*l_p2.at(0));

  ////////ROS_INFO("slope: %f b: %f", slope, b);

  double h = cir_center[0];
  double k = cir_center[1];
  
  ////////ROS_INFO("h: %f k: %f r: %f", h, k, r);

  // Solve to find values of intersection for x
  double A = pow(slope,2) + 1;
  double B = 2*(slope*b - slope*k - h);
  double C = pow(h,2) + pow(b,2) + pow(k,2) - pow(r,2) - 2*b*k;
  
  ////////ROS_INFO("A: %f B: %f C: %f", A, B, C);
  
  if( fabs(A) < 0.0001 )
  {
    x_intersection.push_back(-C/B);
  }
  else if( fabs(C) < 0.0001 )
  {
    x_intersection.push_back(-B/A);
  }
  else if( fabs(B) < 0.0001 )
  {
    x_intersection.push_back(sqrt(-C/A));
    x_intersection.push_back(-sqrt(-C/A));
  }
  else
  {
    double discriminant = pow(B,2) - 4*A*C;
    double x_intersect_1 = (-B + sqrt(discriminant)) / (2*A);
    double x_intersect_2 = (-B - sqrt(discriminant)) / (2*A);
    x_intersection.push_back(x_intersect_1);
    x_intersection.push_back(x_intersect_2);
   
    ////////ROS_INFO("discriminant: %f x_intersect_1: %f x_intersect_2: %f y: %f y_2: %f", discriminant, x_intersect_1, x_intersect_2, (x_intersect_1)*slope+b, (x_intersect_2)*slope+b);
  }

  // Get Min/Max values of line segment
  double x_min = l_p1.at(0);
  double x_max = l_p2.at(0);

  if(x_min > x_max)
  {
    double temp = x_min;
    x_min = x_max;
    x_max = temp;
  }
  
  ////////ROS_INFO("x_min: %f x_max: %f", x_min, x_max);
  
  result = false;

  for(int i=0;i<x_intersection.size();i++)
  {
    double x = x_intersection[i];
    double y = slope*x + b;
    ////////ROS_INFO("x_intersection[%d]: %f y: %f", i, x, y);
    //////////ROS_INFO("isnan(x): %d", isnan(x));
    
    if(!std::isnan(x) && (x >= x_min && x <= x_max) )  
    {
      //////////ROS_INFO("Collision on segment %i", i);
      result = true;
        
      std::vector<double> p_intersect;
      p_intersect.push_back(x);
      p_intersect.push_back(y);

      points_of_collision.push_back(p_intersect);
    } // end if
  } // end for
}



/**
 * arc_points should contain 1st, middle, and last points on the arc
 */
void CollisionDetection::LineArc(const std::vector<double>& l_p1, const std::vector<double>& l_p2, const std::vector<trajectory_msgs::JointTrajectoryPoint>& arc_points, std::vector< std::vector<double> >& points_of_collision, bool& result) const
{
  ////////ROS_INFO("In CollisionDetection::LineArcPoints");
  std::vector<double> x_intersection;

  double slope  = (l_p2.at(1) - l_p1.at(1)) / (l_p2.at(0) - l_p1.at(0));
  double b      = l_p2.at(1) - (slope*l_p2.at(0));

  double r, h, k;
  getCircleInfoPoints(arc_points, r, h, k);
  ////////ROS_INFO("r: %f h: %f k: %f", r, h, k);
  
  // Solve to find values of intersection for x
  double A = pow(slope,2) + 1;
  double B = 2*(slope*b - slope*k - h);
  double C = pow(h,2) + pow(b,2) + pow(k,2) - pow(r,2) - 2*b*k;

  if( fabs(A) < 0.0001 )
  {
    //////////ROS_INFO("A=0, -C/B: %f", -(C/B));
    x_intersection.push_back(-C/B);
  }
  else if( fabs(C) < 0.0001 )
  {
    x_intersection.push_back(-B/A);
  }
  else if( fabs(B) < 0.0001 )
  {
    x_intersection.push_back(sqrt(-C/A));
    x_intersection.push_back(-sqrt(-C/A));
  }
  else
  {
    double discriminant = pow(B,2) - 4*A*C;
    double x_intersect_1 = (-B + sqrt(discriminant)) / (2*A);
    double x_intersect_2 = (-B - sqrt(discriminant)) / (2*A);
    x_intersection.push_back(x_intersect_1);
    x_intersection.push_back(x_intersect_2);
   
    //////////ROS_INFO("discriminant: %f x_intersect_1: %f x_intersect_2: %f y: %f y_2: %f", discriminant, x_intersect_1, x_intersect_2, (x_intersect_1)*slope+b, (x_intersect_2)*slope+b);
  }
  
  // Get Min/Max values of line segment
  double x_min = l_p1.at(0);
  double x_max = l_p2.at(0);

  if(x_min > x_max)
  {
    double temp = x_min;
    x_min = x_max;
    x_max = temp;
  }
    
  // Get range of x for the arc
  //uint16_t i_min = 0;
  //uint16_t i_max = ob_trajectory.trajectory.points.size()-1;
    
  // Angles to starting and end points on arc
  double a = utility_.findAngleFromAToB(0, 0, arc_points[0].positions[0], arc_points[0].positions[1]);
  double b_angle = utility_.findAngleFromAToB(0, 0, arc_points[2].positions[0], arc_points[2].positions[1]);

  // Angle along circle that the arc begins at
  double starting_angle = utility_.findAngleFromAToB(h, k, arc_points[0].positions[0], arc_points[0].positions[1]);

  // Find the closest angle divisible by 90deg
  std::vector<double> angles;
  angles.push_back(0);
  angles.push_back(PI/2.f);
  angles.push_back(PI);
  angles.push_back(-PI/2.f);
  double next_axis_angle = angles[0];
  if( fabs(fmodf(starting_angle, (PI/2.f))) < 0.0001)
  {
    ////////ROS_INFO("In if");
    if(arc_points[0].velocities[2] > 0)
    {
      next_axis_angle = utility_.displaceAngle(starting_angle, (PI/2.f));
    }
    else
    {
      next_axis_angle = utility_.displaceAngle(starting_angle, -(PI/2.f));
    }
  }
  else
  {
    //////////ROS_INFO("In else");
    double min_dist = fabs(utility_.findDistanceBetweenAngles(starting_angle, next_axis_angle));
    //////////ROS_INFO("next_axis_angle: %f min_dist: %f", next_axis_angle, min_dist);
    
    for(int i=1;i<angles.size();i++)
    {
      //////////ROS_INFO("angles[%i]: %f", i, angles[i]);
      //////////ROS_INFO("fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i))): %f", fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i))));
      if(fabs( fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i))) - min_dist ) < 0.0001)
      {
        /*////////ROS_INFO("In if equal!");
        ////////ROS_INFO("utility_.findDistanceBetweenAngles(starting_angle, angles[i]): %f", utility_.findDistanceBetweenAngles(starting_angle, angles[i]));
        ////////ROS_INFO("ob_trajectory w: %f", ob_trajectory.trajectory.points[0].velocities[2]);*/

        // If they share signs
        if(utility_.findDistanceBetweenAngles(starting_angle, angles[i]) * 
            arc_points[0].velocities[2] > 0.f)
        {
          next_axis_angle = angles[i];
        }
      }
      if(fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i))) < (min_dist-0.01))
      {
        min_dist = fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i)));
        next_axis_angle = angles.at(i);
      }
    } // end for
  } // end else
  //////////ROS_INFO("starting_angle: %f next_axis_angle: %f", starting_angle, next_axis_angle);

  std::vector<double> p;
  p.push_back(h + r*cos(next_axis_angle));
  p.push_back(k + r*sin(next_axis_angle));


  std::vector< std::vector<double> > minmax_points;
  minmax_points.push_back(p);
  minmax_points.push_back(arc_points[0].positions);
  minmax_points.push_back(arc_points[2].positions);

  double ob_x_min = minmax_points[0][0];
  double ob_x_max = minmax_points[0][0];
  double ob_y_min = minmax_points[0][1];
  double ob_y_max = minmax_points[0][1];
  for(int i=1;i<minmax_points.size();i++)
  {
    if(minmax_points[i][0] < ob_x_min)
    {
      ob_x_min = minmax_points[i][0];
    }
    if(minmax_points[i][0] > ob_x_max)
    {
      ob_x_max = minmax_points[i][0];
    }
    if(minmax_points[i][1] < ob_y_min)
    {
      ob_y_min = minmax_points[i][1];
    }
    if(minmax_points[i][1] > ob_y_max)
    {
      ob_y_max = minmax_points[i][1];
    }
  }
  
  //////////ROS_INFO("Info: x_min: %f x_max: %f ob_x_min: %f ob_x_max: %f ob_y_min: %f ob_y_max: %f", x_min, x_max, ob_x_min, ob_x_max, ob_y_min, ob_y_max);
    
  result = false;

  for(int i=0;i<x_intersection.size();i++)
  {
    double x = x_intersection[i];
    double y = slope*x + b;
    //////////ROS_INFO("x_intersection[%d]: %f y: %f", i, x, y);
    //////////ROS_INFO("isnan(x): %d", isnan(x));
    
    if(!std::isnan(x) && (x >= x_min && x <= x_max) && ((x >= ob_x_min && x <= ob_x_max) && (y >= ob_y_min && y <= ob_y_max)))  
    {
      //////////ROS_INFO("Collision on segment %i", i);
      result = true;
        
      std::vector<double> p_intersect;
      p_intersect.push_back(x);
      p_intersect.push_back(y);

      points_of_collision.push_back(p_intersect);
    } // end if
  } // end for

}



void CollisionDetection::LineArc(const std::vector<double>& l_p1, const std::vector<double>& l_p2, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision, bool& result) const
{
  ////////ROS_INFO("In CollisionDetection::LineArcTraj");

  std::vector<trajectory_msgs::JointTrajectoryPoint> arc_points;
  arc_points.push_back(ob_trajectory.trajectory.points[0]);

  arc_points.push_back(ob_trajectory.trajectory.points[
      ob_trajectory.trajectory.points.size()/2] );

  arc_points.push_back(ob_trajectory.trajectory.points[
      ob_trajectory.trajectory.points.size()-1] );

  /*for(uint8_t i=0;i<arc_points.size();i++)
  {
    //////ROS_INFO("arc_points[%i]: %s", (int)i, utility_.toString(arc_points[i]).c_str());
  }*/
  LineArc(l_p1, l_p2, arc_points, points_of_collision, result);
  
  // Line info
  ////////ROS_INFO("Exiting CollisionDetection::LineArc");
} // End LineArc




void CollisionDetection::getCircleInfoPoints(const std::vector<trajectory_msgs::JointTrajectoryPoint>& arc_points, double& r, double& h, double& k) const
{
  const trajectory_msgs::JointTrajectoryPoint* p1 = &arc_points[0];
  const trajectory_msgs::JointTrajectoryPoint* p2 = &arc_points[1];
  const trajectory_msgs::JointTrajectoryPoint* p3 = &arc_points[2];
  
  double w = p1->velocities[2];
  double v = sqrt((p1->velocities[0] * p1->velocities[0]) + (p1->velocities[1]*p1->velocities[1]) );
  r = fabs(v / w);
  
  ////////ROS_INFO("p1: (%f,%f) p2: (%f,%f) p3: (%f,%f)", p1->positions[0], p1->positions[1], p2->positions[0], p2->positions[1], p3->positions[0], p3->positions[1]);

  double q = utility_.positionDistance(p1->positions, p2->positions);

  double x_mid = (p1->positions[0] + p2->positions[0]) / 2.f;
  double y_mid = (p1->positions[1] + p2->positions[1]) / 2.f;

  double x_dir = p2->positions[0] - p1->positions[0];
  double y_dir = p2->positions[1] - p1->positions[1];

  double x_dir_per = y_dir / q;
  double y_dir_per = -x_dir / q;
  
  double p0_x = x_mid;      double p0_y = y_mid;
  double r0_x = x_dir_per;  double r0_y = y_dir_per;

  // Get second mid line
  x_mid = (p2->positions.at(0) + p3->positions.at(0)) / 2.f;
  y_mid = (p2->positions.at(1) + p3->positions.at(1)) / 2.f;

  x_dir = p3->positions.at(0) - p2->positions.at(0);
  y_dir = p3->positions.at(1) - p2->positions.at(1);

  x_dir_per = y_dir / q;
  y_dir_per = -x_dir / q;
  
  double p1_x = x_mid;      double p1_y = y_mid;
  double r1_x = x_dir_per;  double r1_y = y_dir_per;
  
  // Find intersection of the two mid lines that are in vector form
  double s = (r0_y*(p1_x-p0_x) + r0_x*(p0_y-p1_y)) / (r1_y*r0_x - r0_y*r1_x);
  double t = ( p1_x - p0_x +s*r1_x) / r0_x;
  
  ////////ROS_INFO("s: %f t: %f", s, t);
  ////////ROS_INFO("q: %f mid: (%f, %f) dir: (%f, %f) dir_per: (%f, %f)", q, x_mid, y_mid, x_dir, y_dir, x_dir_per, y_dir_per);

  // Compute h and k
  h = p0_x + r0_x*t;
  k = p0_y + r0_y*t;
  
  //////////ROS_INFO("h: %f k: %f r: %f", h, k, r);
}




void CollisionDetection::getCircleInfoTraj(const ramp_msgs::RampTrajectory& traj, double& r, double& h, double& k) const
{
  
  // Get circle info
  double w = traj.trajectory.points[0].velocities[2];
  double v = sqrt((traj.trajectory.points[0].velocities[0] * traj.trajectory.points[0].velocities[0]) + (traj.trajectory.points[0].velocities[1]*traj.trajectory.points[0].velocities[1]) );
  r = fabs(v / w);

  //////////ROS_INFO("v: %f w: %f r: %f", v, w, r);

  // Compute circle center!
  const trajectory_msgs::JointTrajectoryPoint* p1 = &traj.trajectory.points[0];
  const trajectory_msgs::JointTrajectoryPoint* p2 = &traj.trajectory.points[ traj.trajectory.points.size() / 2.f];
  const trajectory_msgs::JointTrajectoryPoint* p3 = &traj.trajectory.points[
      traj.trajectory.points.size()-1];
  //////////ROS_INFO("p1: (%f,%f) p2: (%f,%f) p3: (%f,%f)", p1->positions[0], p1->positions[1], p2->positions[0], p2->positions[1], p3->positions[0], p3->positions[1]);

  std::vector<trajectory_msgs::JointTrajectoryPoint> arc_points;
  arc_points.push_back(*p1);
  arc_points.push_back(*p2);
  arc_points.push_back(*p3);

  getCircleInfoPoints(arc_points, r, h, k);
}


void CollisionDetection::LineArcFull(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const
{
  ////////ROS_INFO("In CollisionDetection::LineArcFull");
  ros::Time t_start = ros::Time::now();

  // Get Line info
  std::vector<double> l_p1;
  l_p1.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[0]);
  l_p1.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[1]);
  
  std::vector<double> l_p2;
  l_p2.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[0]);
  l_p2.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[1]);
  ////////ROS_INFO("l_p1: (%f, %f) l_p2: (%f, %f)", l_p1[0], l_p1[1], l_p2[0], l_p2[1]);



  std::vector<trajectory_msgs::JointTrajectoryPoint> arc_points, pos_dir, neg_dir;
  arc_points.push_back(ob_trajectory.trajectory.points[0]);

  arc_points.push_back(ob_trajectory.trajectory.points[
      ob_trajectory.trajectory.points.size()/2] );

  arc_points.push_back(ob_trajectory.trajectory.points[
      ob_trajectory.trajectory.points.size()-1] );

  bool temp;
  LineArc(l_p1, l_p2, arc_points, points_of_collision, temp);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());


  // Find the points for two other arcs
  // Translate the arc points
  double x_com = cos(arc_points[0].positions[2]);
  double y_com = sin(arc_points[0].positions[2]);
  ////////ROS_INFO("x_com: %f y_com: %f norm: %f", x_com, y_com, sqrt( (x_com*x_com) + (y_com*y_com) ));

  double sum_radii = 0.4;

  pos_dir = arc_points;
  // Positive translation for new arc points
  for(uint8_t i=0;i<arc_points.size();i++)
  {
    pos_dir[i].positions[0] += sum_radii*x_com;
    pos_dir[i].positions[1] += sum_radii*y_com;
    ////////ROS_INFO("New arc point: (%f, %f)", pos_dir[i].positions[0], pos_dir[i].positions[1]);
  }

  LineArc(l_p1, l_p2, pos_dir, points_of_collision, temp);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  
  neg_dir = arc_points;
  // Negative translation for new arc points
  for(uint8_t i=0;i<arc_points.size();i++)
  {
    neg_dir[i].positions[0] -= sum_radii*x_com;
    neg_dir[i].positions[1] -= sum_radii*y_com;
    ////////ROS_INFO("New arc point: (%f, %f)", neg_dir[i].positions[0], neg_dir[i].positions[1]);
  }
  
  LineArc(l_p1, l_p2, neg_dir, points_of_collision, temp);
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());

  
  // Boundary lines creating the fan
  LineLineEndPoints(l_p1, l_p2, pos_dir[0].positions, neg_dir[0].positions, points_of_collision); 
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());
  
  LineLineEndPoints(l_p1, l_p2, pos_dir[2].positions, neg_dir[2].positions, points_of_collision); 
  ////////ROS_INFO("points_of_collision.size(): %i", (int)points_of_collision.size());

  
  //ros::Duration d_linearc = ros::Time::now() - t_start;
  //////////ROS_INFO("LineArc time: %f", d_linearc.toSec());

  ////////ROS_INFO("Exiting CollisionDetection::LineArc");
} // End LineArc




void CollisionDetection::query(const std::vector<trajectory_msgs::JointTrajectoryPoint>& segment, const std::vector<trajectory_msgs::JointTrajectoryPoint>& ob_trajectory, const double& traj_start, const double& coll_dist, QueryResult& result) const
{
  ros::Time time_start = ros::Time::now();

  ////ROS_INFO("In CollisionDetection::query"); 
  //////ROS_INFO("trajectory.points.size(): %i", (int)segment.size());
  //////ROS_INFO("ob_trajectory.points.size(): %i", (int)ob_trajectory.size());

  /*if(ob_trajectory.trajectory.points.size() > 2)
  {
    ////////ROS_INFO("ob_trajectory: %s", utility_.toString(ob_trajectory).c_str());
  }*/
  
  // For every point, check circle detection on a subset of the obstacle's trajectory
  float dist_threshold = coll_dist > 0.4 ? coll_dist : 0.225;
  dist_threshold = 0.225;

  // Trajectories start in the future, obstacle trajectories start at the present time, 
  // set an offset for obstacle indices to account for this 
  double  t_start   = traj_start;
  int     j_offset  = t_start * 10.f;
  
  //////ROS_INFO("t_start: %f j_offset: %i", t_start, j_offset);

  int i=0, j=0;

  for(i=0;i<segment.size();i++) 
  //for(i=0;i<49;i++) 
  {

    // Set obstacle index. If i+offset > trajectory size, set j to the last point on the obstacle trajectory
    j = (i+j_offset) >= ob_trajectory.size() ? ob_trajectory.size()-1 : i+j_offset;

    ////////ROS_INFO("i: %i j: %i", i, j);
    
    // Get the points
    const trajectory_msgs::JointTrajectoryPoint* p_i    = &segment[i];
    const trajectory_msgs::JointTrajectoryPoint* p_ob   = &ob_trajectory[j];
    
    ////////ROS_INFO("p_i: %s", utility_.toString(*p_i).c_str());
    ////////ROS_INFO("p_j: %s", utility_.toString(*p_ob).c_str());

    // Get the distance between the centers
    float dist = sqrt( pow(p_i->positions.at(0) - p_ob->positions.at(0),2) + pow(p_i->positions.at(1) - p_ob->positions.at(1),2) );

    // If the distance between the two centers is less than the sum of the two radii, 
    // there is collision
    if( dist <= dist_threshold) 
    {
      /*//ROS_INFO("Points in collision: (%f,%f), and (%f,%f), dist: %f i: %i j: %i",
          p_i->positions.at(0),
          p_i->positions.at(1),
          p_ob->positions.at(0),
          p_ob->positions.at(1),
          dist,
          (int)i,
          (int)j);*/
      
      result.collision_         = true;
      result.t_firstCollision_  = p_i->time_from_start.toSec();
      i                         = segment.size();
    } // end if
  } // end for

  ////////ROS_INFO("Exiting CollisionDetection::query");
} // End query


void CollisionDetection::query(const std::vector<trajectory_msgs::JointTrajectoryPoint>& segment, const std::vector<trajectory_msgs::JointTrajectoryPoint>& ob_trajectory, std::vector< std::vector<double> >& points_of_collision) const
{
  ros::Time time_start = ros::Time::now();
  //////ROS_INFO("In CollisionDetection::query"); 
  //////////ROS_INFO("trajectory.points.size(): %i", (int)trajectory.trajectory.points.size());
  //////////ROS_INFO("ob_trajectory.points.size(): %i", (int)ob_trajectory.trajectory.points.size());
  /*if(ob_trajectory.trajectory.points.size() > 2)
  {
    ////////ROS_INFO("ob_trajectory: %s", utility_.toString(ob_trajectory).c_str());
  }*/

  double  t_start   = segment[0].time_from_start.toSec();
  int     j_offset  = t_start * 10.f;

  // If only 1 point, then set stopping point to end of segment
  int     j_stop = ob_trajectory.size() > segment.size() ? ob_trajectory.size() : segment.size();
  //////ROS_INFO("t_start: %f j_offset: %i", t_start, j_offset);

  int i=0, j=0;
  
  // For every point, check circle detection on a subset of the obstacle's trajectory
  float radius = 0.5f;

  while(i<segment.size())
  //for(i=0;i<segment.size() && j<j_stop;i++) 
  {
    
    // Get the ith point on the trajectory
    const trajectory_msgs::JointTrajectoryPoint* p_i = &segment[i];

    //////////ROS_INFO("p_i: %s", utility_.toString(p_i).c_str());

    int j = (i+j_offset) >= ob_trajectory.size() ? ob_trajectory.size()-1 : i+j_offset;
    
    ////////ROS_INFO("i: %i j: %i", i, j);

    // Get the jth point of the obstacle's trajectory
    const trajectory_msgs::JointTrajectoryPoint* p_ob  = &ob_trajectory[j];

    // Get the distance between the centers
    float dist = sqrt( pow(p_i->positions.at(0) - p_ob->positions.at(0),2) + pow(p_i->positions.at(1) - p_ob->positions.at(1),2) );

    // If the distance between the two centers is less than the sum of the two radii, 
    // there is collision
    if( dist <= radius*2 ) 
    {
      /*////ROS_INFO("Points in collision: (%f,%f), and (%f,%f), dist: %f i: %i j: %i",
          p_i->positions[0],
          p_i->positions[1],
          p_ob->positions[0],
          p_ob->positions[1],
          dist,
          (int)i,
          (int)j);*/
      points_of_collision.push_back(p_i->positions);
      i = segment.size();
    } // end if

    i++; 
  } // end for
}


/** 
 * This method returns true if there is collision between trajectory_ and the obstacle's trajectory, false otherwise 
 * The robots are treated as circles for simple collision detection
 */
void CollisionDetection::query(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& result) const 
{
  ros::Time time_start = ros::Time::now();
  //////////ROS_INFO("In CollisionDetection::query"); 
  //////////ROS_INFO("trajectory.points.size(): %i", (int)trajectory.trajectory.points.size());
  //////////ROS_INFO("ob_trajectory.points.size(): %i", (int)ob_trajectory.trajectory.points.size());
  /*if(ob_trajectory.trajectory.points.size() > 2)
  {
    ////////ROS_INFO("ob_trajectory: %s", utility_.toString(ob_trajectory).c_str());
  }*/

  double t_start = trajectory.t_start.toSec();
  int j_offset = t_start * 10.f;
  //////////ROS_INFO("t_start: %f j_offset: %i", t_start, j_offset);

  uint8_t t_checkColl = 3;

 
  // Find the point that ends the trajectory's non-holonomic section
  uint16_t i_stop = 0;

  // If there are no curves
  // If there is a curve and only two knot points (curve ends traj)
  if(t_start < 0.01)
  {
    //////////ROS_INFO("In 1st if");
    i_stop = trajectory.i_knotPoints.at(trajectory.i_knotPoints.size()-1);
  }
  else if(   trajectory.curves.size() == 0 ||
      ( trajectory.curves.size() == 1 && trajectory.i_knotPoints.size() == 2) )
  {
    //////////ROS_INFO("In 2nd if");
    i_stop = trajectory.i_knotPoints.at(1);
  }
  
  // If there's only one curve 
  //  (If no transition traj, then two segments)
  //  (If transition traj, then only one segment)
  else if(trajectory.curves.size() == 1)
  {
    //////////ROS_INFO("In 3rd if");
    i_stop = trajectory.i_knotPoints.at(2);
  }

  // If there's two curves
  else
  {
    //////////ROS_INFO("In 4th if");
    i_stop = trajectory.i_knotPoints.at(3);
  }
 
  int j_start;
 
  //////////ROS_INFO("i_stop: %i", i_stop);
  
  // For every point, check circle detection on a subset of the obstacle's trajectory
  float radius = 0.5f;
  for(uint16_t i=0;i<i_stop;i++) 
  {
    
    // Get the ith point on the trajectory
    const trajectory_msgs::JointTrajectoryPoint* p_i = &trajectory.trajectory.points[i];

    //////////ROS_INFO("p_i: %s", utility_.toString(p_i).c_str());

    
    // Compute which point on the obstacle trajectory to start doing collision checking
    if(ob_trajectory.trajectory.points.size() == 1)
    {
      j_start = 0;
      t_checkColl = 0;
    }
    else if(i <= t_checkColl)
    {
      j_start = 0+j_offset;
    }
    else
    {
      j_start = (i-t_checkColl)+j_offset;
    }

    //////////ROS_INFO("j_start: %i", j_start);

    // *** Test position i for collision against some points on obstacle's trajectory ***
    // Obstacle trajectory should already be in world coordinates!
    for(int j = j_start;
        j<=(i+t_checkColl+j_offset) && j<ob_trajectory.trajectory.points.size();
        j++)
    {
    /*if(ob_trajectory.trajectory.points.size() > 2)
    {
      ////////ROS_INFO("i: %i j: %i", i, j);
    }*/

      // Get the jth point of the obstacle's trajectory
      const trajectory_msgs::JointTrajectoryPoint* p_ob  = &ob_trajectory.trajectory.points[j];

      // Get the distance between the centers
      float dist = sqrt( pow(p_i->positions.at(0) - p_ob->positions.at(0),2) + pow(p_i->positions.at(1) - p_ob->positions.at(1),2) );

    /*if(ob_trajectory.trajectory.points.size() > 2)
    {
      ////////ROS_INFO("Comparing trajectory point (%f,%f) and obstacle point (%f,%f): dist = %f", 
          p_i.positions.at(0), p_i.positions.at(1), 
          p_ob.positions.at(0), p_ob.positions.at(1), 
          dist);
    }*/
        

      // If the distance between the two centers is less than the sum of the two radii, 
      // there is collision
      if( dist <= radius*2 ) 
      {
        /*////ROS_INFO("Points in collision: (%f,%f), and (%f,%f), dist: %f i: %i j: %i",
            p_i.positions.at(0),
            p_i.positions.at(1),
            p_ob.positions.at(0),
            p_ob.positions.at(1),
            dist,
            (int)i,
            (int)j);*/
        result.collision_ = true;
        result.t_firstCollision_ = p_i->time_from_start.toSec();
        i = i_stop;
        break;
      } // end if
    } // end for
  } // end for

  //////////ROS_INFO("result: %s", result.collision_ ? "True" : "False");
  //////////ROS_INFO("Exiting CollisionDetection::query");
  //////////ROS_INFO("Query Elapsed time: %f", (ros::Time::now()-time_start).toSec());
} //End query








