#include "circle_packer.h"



CirclePacker::CirclePacker(nav_msgs::OccupancyGridConstPtr g)
{
  //ROS_INFO("In CirclePacker::CirclePacker()");
  grid_ = *g;
  convertOGtoMat(g);
}

CirclePacker::~CirclePacker() {}

void CirclePacker::convertOGtoMat(nav_msgs::OccupancyGridConstPtr g)
{
  //ROS_INFO("In CirclePacker::convertOGtoMat");

  
  // Use the GridMap2D library to convert from nav_msgs::OccupancyGrid to cv::Mat
  gridmap_2d::GridMap2D gmap(g, false);

  // Create a window
  //cv::namedWindow("testing", CV_WINDOW_AUTOSIZE);

  src = gmap.binaryMap();

  // Show the image
  //cv::imshow("testing", src);

  // PRESS ESC TO BEFORE CLOSING WINDOW, OTHERWISE THE PROGRAM WILL HANG
  //cv::waitKey(0);
}

void CirclePacker::CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src, detected_edges, cv::Size(3,3) );

  //std::cout<<"\nDetected Edges: "<<detected_edges;
  //ROS_INFO("detected_edges type: %i", detected_edges.type());
  //ROS_INFO("lowThreshold: %i", lowThreshold);

  // Somehow, lowThreshold is being converted to unsigned int before this point
  // its value is 32767 (-1 for unsigned 4-byte int)
  // Set the value back to 0 for edge detection to work
  lowThreshold = 0;

  /// Canny detector
  cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = cv::Scalar::all(0);

  //std::cout<<"\nDetected Edges: "<<detected_edges;

  src.copyTo( dst, detected_edges);
  //cv::imshow("detected_edges", dst);
  //cv::waitKey(0);
  //imshow( window_name, dst );
}

double CirclePacker::getMinDistToPoly(const Polygon& poly, const Cell& cell)
{
  double result = 100000;

  for(int n=0;n<poly.normals.size();n++)
  {
    //std::cout<<"\npoly.normals[n]: "<<poly.normals[n].a<<", "<<poly.normals[n].b;
    // Get unit normal
    double l = sqrt( pow(poly.normals[n].a,2) + pow(poly.normals[n].b,2) );
    Normal v_hat;
    v_hat.a = poly.normals[n].a / l;
    v_hat.b = poly.normals[n].b / l;
    
    std::vector<double> r;
    r.push_back(poly.edges[n].start.x - cell.p.x);
    r.push_back(poly.edges[n].start.y - cell.p.y);

    double d = fabs((v_hat.a*r[0]) + (v_hat.b*r[1]));
    //std::cout<<"\nl: "<<l<<" v_hat: <"<<v_hat.a<<","<<v_hat.b<<"> r: <"<<r[0]<<","<<r[1]<<">";
    //std::cout<<"\nd: "<<d;
    if(d < result)
    {
      result = d;
    }
  }

  return result;
}

double CirclePacker::getMinDistToCirs(const std::vector<Circle>& cirs, const Cell& cell)
{
  if(cirs.size() == 0)
  {
    return -1;
  }

  double result=10000;

  for(int i=0;i<cirs.size();i++)
  {
    double dist = sqrt( pow( cell.p.x - cirs[i].center.x, 2) + pow( cell.p.y - cirs[i].center.y, 2) );
    
    // Then, subtract the radius to get the dist to the outside of the circle
    dist -= cirs[i].radius;

    if( dist < result)
    {
      result = dist;
    }
  }

  return result;
}


void CirclePacker::deleteCellsInCir(const std::vector<Cell>& cells, const Circle cir, std::vector<Cell>& result)
{
  //std::cout<<"\nIn deleteCellsInCir\n";
  for(int i=0;i<cells.size();i++)
  {
    //std::cout<<"\nTesting cell "<<cells[i].p.x<<", "<<cells[i].p.y;
    //std::cout<<"\nDist: "<<(sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ));
    // Get distance between circle of cell and new cell, check if > its radius
    if( sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ) > cir.radius )
    {
      result.push_back(cells[i]);
    }
  }
  //std::cout<<"\nExiting deleteCellsInCir\n";
}


Normal CirclePacker::computeNormal(Edge e)
{
  //std::cout<<"\ne.start: "<<e.start.x<<" "<<e.start.y;
  //std::cout<<"\ne.end: "<<e.end.x<<" "<<e.end.y;
  Normal result;
  result.a = e.end.y - e.start.y;
  result.b = -(e.end.x - e.start.x);

  result.c = -((result.a*e.start.x) + (result.b*e.start.y));

  //std::cout<<"\na: "<<result.a<<" b: "<<result.b<<" c: "<<result.c;
  return result;
}


bool CirclePacker::cellInPoly(Polygon poly, cv::Point cell)
{
  for(int i=0;i<poly.normals.size();i++)
  {
    //std::cout<<"\nnormal a: "<<poly.normals[i].a<<" b: "<<poly.normals[i].b<<" c: "<<poly.normals[i].c;
    double d = poly.normals[i].a*cell.x + poly.normals[i].b*cell.y + poly.normals[i].c;
    //std::cout<<"\ncell center: "<<cell.x<<", "<<cell.y<<" d: "<<d;
    if(d > -0.000001)
    {
      //std::cout<<"\nNot in polygon";
      return false;
    }
  }

  return true;
}


std::vector<Circle> CirclePacker::getCirclesFromPoly(Polygon poly)
{
  //std::cout<<"\n# of edges: "<<poly.edges.size();
  std::vector<Circle> result;
  std::vector<cv::Point> vertices;
  
  // Push on all vertices
  for(int i=0;i<poly.edges.size();i++)
  {
    vertices.push_back(poly.edges[i].start);
    vertices.push_back(poly.edges[i].end);
  }
  
  double MAX_LENGTH= vertices[0].y;
  double MAX_WIDTH = vertices[0].x;
  double MIN_LENGTH= vertices[0].y;
  double MIN_WIDTH = vertices[0].x;
  for(int i=0;i<vertices.size();i++)
  {
    if(vertices[i].y > MAX_LENGTH)
    {
      MAX_LENGTH = vertices[i].y;
    }
    if(vertices[i].y < MIN_LENGTH)
    {
      MIN_LENGTH = vertices[i].y;
    }
    
    if(vertices[i].x > MAX_WIDTH)
    {
      MAX_WIDTH = vertices[i].x;
    }
    if(vertices[i].x < MIN_WIDTH)
    {
      MIN_WIDTH = vertices[i].x;
    }
  }

  double round = 1;
  int width_count = (MAX_WIDTH - MIN_WIDTH) / round;
  int length_count = (MAX_LENGTH - MIN_LENGTH) / round;
  double start_x = MIN_WIDTH + round/2.f;
  double start_y = MIN_LENGTH + round/2.f;

  //std::cout<<"\nMAX_WIDTH: "<<MAX_WIDTH<<" MAX_LENGTH: "<<MAX_LENGTH<<" width_count: "<<width_count<<" length_count: "<<length_count;

  std::vector<Cell> cells;
 
  for(int i=0;i<width_count;i++)
  {
    for(int j=0;j<length_count;j++)
    {
      //std::cout<<"\ni: "<<i<<" j: "<<j<<" round: "<<round;
      double x = start_x + (round * (i)); 
      double y = start_y + (round * (j));
      Cell temp;
      temp.p.x = x;
      temp.p.y = y;
    
      //std::cout<<"\n("<<temp.p.x<<", "<<temp.p.y<<")";

      if(cellInPoly(poly, temp.p))
      {
        cells.push_back(temp);
      }
    }
  }
  

  std::vector<Cell> reduced_cells = cells;


  while(cells.size() > 0)
  {
    //std::cout<<"\nIn while cells.size(): "<<cells.size()<<" result.size(): "<<result.size();
    cells = reduced_cells;

    std::priority_queue<Cell, std::vector<Cell>, CompareDist> updated_pq;

    // Delete all cells whose centers lie in the largest circle
    if(result.size() > 0)
    {
      reduced_cells.clear();
      deleteCellsInCir(cells, result[result.size()-1], reduced_cells);
    }

    // Recalculate the distance, include existing circles!
    // For each cell, compute distance to the closest polygon edge
    for(int i=0;i<reduced_cells.size();i++)
    {
      Cell& cell = reduced_cells[i];

      double min_d=getMinDistToPoly(poly, cell);
      double min_cir=getMinDistToCirs(result,cell);

      if(min_d < min_cir || min_cir < 0)
      {
        cell.dist = min_d;
      }
      else
      {
        cell.dist = min_cir;
      }

      updated_pq.push(cell);
    } // end for each cell

    if(!updated_pq.empty())
    {
      Cell c = updated_pq.top();
      Circle temp;

      temp.center.x = c.p.x;
      temp.center.y = c.p.y;
      temp.radius = c.dist;

      result.push_back(temp);
    }
  }
  
  /*std::cout<<"\nFinal number of circles: "<<result.size();
  for(int i=0;i<result.size();i++)
  {
    std::cout<<"\nCircle "<<i<<" ("<<result[i].center.x<<", "<<result[i].center.y<<") radius: "<<result[i].radius;
  }*/

  return result;
}


std::vector<Triangle> CirclePacker::triangulatePolygon(const Polygon& poly)
{
  std::vector<Triangle> result;

  // Get all vertices of polygon
  std::vector<cv::Point> vertices;
  for(int i=0;i<poly.edges.size();i++)
  {
    vertices.push_back(poly.edges[i].start);
  }
  
  std::vector<int> i_reflex;

  // For each vertex, get its two neighbors
  // Check if line connecting them is in polygon
  for(int i=0;i<vertices.size();i++)
  {
    std::cout<<"\nVertex "<<i<<": ("<<vertices[i].x<<", "<<vertices[i].y<<")";

    // Get neighbors
    cv::Point v0 = i == 0 ? vertices[vertices.size()-2] : vertices[i-1];
    cv::Point v1 = vertices[i];
    cv::Point v2 = i == vertices.size() - 1 ? vertices[0] : vertices[i+1];

    std::cout<<"\nNeighbors: ("<<v0.x<<", "<<v0.y<<") ("<<v2.x<<", "<<v2.y<<")";
   
    // Get direction angle of each segment
    // y component negated because grid y increases as it goes down
    double ax = v0.x - v1.x;
    double bx = v2.x - v1.x;
    double ay = -( v0.y - v1.y );
    double by = -( v2.y - v1.y );
    std::cout<<"\nax: "<<ax<<" ay: "<<ay<<" bx: "<<bx<<" by: "<<by;

    double ta = atan2(ay , ax);
    double tb = atan2(by , bx);
    std::cout<<"\nta: "<<ta<<" tb: "<<tb;
   
    double t_final = fmodf(ta - tb, 6.28);
    std::cout<<"\nt_final: "<<t_final;

    if(t_final > 3.14159)
    {
      i_reflex.push_back(i);
    }
  } // end for each vertex


  return result;
}


visualization_msgs::Marker CirclePacker::getMarkerForCircle(const Circle c) const
{
  visualization_msgs::Marker result;

  result.header.stamp = ros::Time::now();
  result.header.frame_id = "/base_footprint";
  result.ns = "basic_shapes";
  result.id = 0;
  
  result.type = visualization_msgs::Marker::SPHERE;
  result.action = visualization_msgs::Marker::ADD;
  result.pose.position.x = c.center.x;
  result.pose.position.y = c.center.y;
  result.pose.position.z = 0;
  result.pose.orientation.x = 0.0;
  result.pose.orientation.y = 0.0;
  result.pose.orientation.z = 0.0;
  result.pose.orientation.w = 1.0;
  result.scale.x = c.radius*2.f;
  result.scale.y = c.radius*2.f;
  result.scale.z = 0.01;
  result.color.r = 0;
  result.color.g = 1;
  result.color.b = 0;
  result.color.a = 1;
  result.lifetime = ros::Duration();

  return result;
}



std::vector<visualization_msgs::Marker> CirclePacker::getMarkers(const std::vector<Circle> cirs) const
{
  std::vector<visualization_msgs::Marker> result;

  for(int i=0;i<cirs.size();i++)
  {
    result.push_back(getMarkerForCircle(cirs[i]));
  }

  return result;
}


std::vector<Circle> CirclePacker::getCirclesFromEdgeSets(const std::vector< std::vector<Edge> > edge_sets)
{
  std::vector<Circle> result;

  //ROS_INFO("In CirclePacker::getCirclesFromEdgeSets");

  for(int i=0;i<edge_sets.size();i++)
  {
    //ROS_INFO("Edge set %i", i);
    // For each set of edges, find the minimum and maximum values for x and y
    int x_min = edge_sets[i][0].start.x, 
        y_min = edge_sets[i][0].start.y, 
        x_max = x_min, 
        y_max = y_min;
    for(int j=1;j<edge_sets[i].size();j++)
    {
      //ROS_INFO("\tEdge %i - start: (%i,%i) end: (%i,%i)", j, edge_sets[i][j].start.y, edge_sets[i][j].start.x, edge_sets[i][j].end.y, edge_sets[i][j].end.x);
      if( edge_sets[i][j].start.x < x_min )
      {
        x_min = edge_sets[i][j].start.x;
      } 
      if( edge_sets[i][j].start.x > x_max )
      {
        x_max = edge_sets[i][j].start.x;
      } 
      if( edge_sets[i][j].start.y < y_min )
      {
        y_min = edge_sets[i][j].start.y;
      } 
      if( edge_sets[i][j].start.y > y_max )
      {
        y_max = edge_sets[i][j].start.y;
      } 
    } // end inner for

    //ROS_INFO("\tx_min: %i x_max: %i y_min: %i y_max: %i", x_min, x_max, y_min, y_max);

    // Get difference between min+max for both x and y
    double x_diff = fabs(x_max - x_min);
    double y_diff = fabs(y_max - y_min);

    //ROS_INFO("\tx_diff: %f y_diff: %f", x_diff, y_diff);

    // Set radius to half of the largest difference (half because difference would be diameter)
    double r = x_diff > y_diff ? x_diff/2.f : y_diff/2.f;

    // Find approximate center 
    // FLIP X AND Y!!
    Point cen; 
    cen.x = (y_max + y_min) / 2.f;
    cen.y = (x_max + x_min) / 2.f;

    // Translate the center by 0.075cm in both directions
    cen.x+=1.5;
    cen.y+=1.5;

    //ROS_INFO("\tCenter: (%f,%f) Radius: %f", cen.x, cen.y, r);

    Circle temp;

    // Inflate radius by 20cm
    temp.radius = r+4;
    temp.center = cen;

    result.push_back(temp);
  } // end outter for
  
  return result;
}

std::vector<Circle> CirclePacker::getCirclesFromEdges(const std::vector<Edge> edges, const cv::Point robot_cen)
{
  std::vector<Circle> result;

  for(int i=0;i<edges.size();i++)
  {
    Circle temp;

    //ROS_INFO("Edge endpoints: (%i,%i) (%i,%i)", edges[i].start.x, edges[i].start.y, edges[i].end.x, edges[i].end.y);

    // Get length of edge to use as diameter of circle
    double dist = sqrt( pow(edges[i].end.x - edges[i].start.x, 2) + pow(edges[i].end.y - edges[i].start.y, 2) );
    temp.radius = dist/2.f;
    
    // Get the midpoint of the edge
    double x_mid = (edges[i].end.x + edges[i].start.x) / 2.f;
    double y_mid = (edges[i].end.y + edges[i].start.y) / 2.f;
    
    // Get angle between robot center and edge midpoint
    std::vector<double> rob_cen; 
    rob_cen.push_back(robot_cen.x); 
    rob_cen.push_back(robot_cen.y);
    std::vector<double> edge_mid;
    edge_mid.push_back(x_mid);
    edge_mid.push_back(y_mid);
    
    double theta = utility_.findAngleFromAToB(rob_cen, edge_mid); 
    double phi = utility_.displaceAngle(PI, theta);

    // Get circle center with phi
    double psi = utility_.displaceAngle(phi, PI);
    double delta_x = temp.radius*cos(psi);
    double delta_y = temp.radius*sin(psi);

    double x_cen = x_mid + delta_x;
    double y_cen = y_mid + delta_y;

    //ROS_INFO("Edge midpoint: (%f, %f) theta: %f phi: %f psi: %f Circle center: (%f, %f)", x_mid, y_mid, theta, phi, psi, x_cen, y_cen);

    temp.center.x = x_cen;
    temp.center.y = y_cen;

    result.push_back(temp);
  }

  return result;
}


std::vector<Circle> CirclePacker::go()
{
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);

  
  // Get the contour points
  ros::Time t_start_contour = ros::Time::now();

  std::vector< std::vector<cv::Point> > detected_contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(detected_edges, detected_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  ros::Duration d_contour(ros::Time::now() - t_start_contour);

  //ROS_INFO("detected_contours size: %i", (int)detected_contours.size());

  /*
   * Get every edge in 1 vector, and make a circle for each edge
   */

  // Make Edges from detected contour points (endpoints of edges)
  std::vector< std::vector<Edge> > edge_sets;
  std::vector<Edge> edges;
  for(int i=0;i<detected_contours.size();i++)
  {
    std::vector<Edge> set;
    //ROS_INFO("detected_contours[%i].size(): %i", i, (int)detected_contours[i].size());
    for(int j=0;j<detected_contours[i].size()-1;j++)
    {
      //ROS_INFO("detected_contours[%i][%i]: (%i, %i)", i, j, detected_contours[i][j].x, detected_contours[i][j].y);
      Edge temp;
      temp.start.x = detected_contours[i][j].x;
      temp.start.y = detected_contours[i][j].y;

      temp.end.x = detected_contours[i][j+1].x;
      temp.end.y = detected_contours[i][j+1].y;

      edges.push_back(temp);
      set.push_back(temp);
    }
    
    Edge temp;
    temp.start.x = detected_contours[i][detected_contours[i].size()-1].x;
    temp.start.y = detected_contours[i][detected_contours[i].size()-1].y;

    temp.end.x = detected_contours[i][0].x;
    temp.end.y = detected_contours[i][0].y;

    edges.push_back(temp);
    set.push_back(temp);

    edge_sets.push_back(set);
  }
  //ROS_INFO("Edges.size(): %i", (int)edges.size());





  /*ros::Time t_start_cirs_from_edges = ros::Time::now();
  
  cv::Point robo_cen;
  robo_cen.x = 0;
  robo_cen.y = 0;

  std::vector<Circle> cirs_from_edges = getCirclesFromEdges(edges, robo_cen);
  ros::Duration d_cirs_from_edges(ros::Time::now() - t_start_cirs_from_edges);

  //ROS_INFO("cirs_from_edges size: %i", (int)cirs_from_edges.size());
  for(int i=0;i<cirs_from_edges.size();i++)
  {
    ROS_INFO("Circle %i - Center: (%i, %i) Radius: %f", i, cirs_from_edges[i].center.x, cirs_from_edges[i].center.y, cirs_from_edges[i].radius);
  }*/

  ros::Time t_start_cirs_from_sets = ros::Time::now();
  std::vector<Circle> cirs_from_sets = getCirclesFromEdgeSets(edge_sets);
  ros::Duration d_cirs_from_sets(ros::Time::now() - t_start_cirs_from_sets);

  for(int i=0;i<cirs_from_sets.size();i++)
  {
    //ROS_INFO("Circle %i - Center: (%f, %f) Radius: %f", i, cirs_from_sets[i].center.x, cirs_from_sets[i].center.y, cirs_from_sets[i].radius);
    result.push_back(cirs_from_sets[i]);
  }


  /*
   * Get every set of edges, and make a circle for each set
   */


  // Find convex hull for each set of contour points
  /*std::vector< std::vector<cv::Point> > hull(detected_contours.size());
  for(int i=0;i<detected_contours.size();i++)
  {
    cv::convexHull( detected_contours[i], hull[i], false );
  }
  
  ROS_INFO("hull.size(): %i", (int)hull.size());
  
  // Build a polygon for each convex hull
  for(int h=0;h<hull.size();h++)
  {
    Polygon p;
    for(int i=0;i<hull[h].size()-1;i++)
    {
      std::cout<<"\nPoint "<<i<<" ("<<hull[h][i].x<<", "<<hull[h][i].y<<")";
      Edge temp;
      temp.start.x = hull[h][i].x;
      temp.start.y = hull[h][i].y;

      temp.end.x = hull[h][i+1].x;
      temp.end.y = hull[h][i+1].y;

      p.edges.push_back(temp);
    }
    
    // Last edge
    Edge temp;
    temp.start.x = hull[h][hull[h].size()-1].x;
    temp.start.y = hull[h][hull[h].size()-1].y;

    temp.end.x = hull[h][0].x;
    temp.end.y = hull[h][0].y;

    p.edges.push_back(temp);
   
    // Build the set of normals for the polygon 
    for(int i=0;i<p.edges.size();i++)
    {
      p.normals.push_back(computeNormal(p.edges[i]));
    }
    
    // Pack the polygon and return the set of circles 
    std::vector<Circle> cirs = getCirclesFromPoly(p);
    ROS_INFO("cirs.size(): %i", (int)cirs.size());

    result.push_back(cirs);
  }
  
  ROS_INFO("result size: %i", (int)result.size());
  for(int i=0;i<result.size();i++)
  {
    ROS_INFO("result[%i].size(): %i", i, (int)result[i].size());
    for(int j=0;j<result[i].size();j++)
    {
      ROS_INFO("Circle %i, Center: (%i, %i) Radius: %f", j, result[i][j].center.x, result[i][j].center.y, result[i][j].radius);
    }
  }*/

  ROS_INFO("d_edges_detect: %f", d_edges_detect.toSec());
  ROS_INFO("d_contour: %f", d_contour.toSec());
  //ROS_INFO("d_cirs_from_edges: %f", d_cirs_from_edges.toSec());
  ROS_INFO("d_cirs_from_sets: %f", d_cirs_from_sets.toSec());

  //ROS_INFO("Leaving go()");

  return result;
}
