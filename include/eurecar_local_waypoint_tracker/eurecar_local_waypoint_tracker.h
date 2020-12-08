#ifndef LOCAL_WPT_TRACKER_H
#define LOCAL_WPT_TRACKER_H


// headers in ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Lane.h>

// headers in local directory
#include "autoware_msgs/DetectedObjectArray.h"

// headers in STL
#include <memory>
#include <cmath>

// OpenCV Libraries
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class LocalWaypointTracker
{
public:
  LocalWaypointTracker();
  ~LocalWaypointTracker();

  void init();
  void run();
  void CallbackGlobalWaypoints(const autoware_msgs::LaneArrayConstPtr& msg);
  void CallbackCostMapBasedPath(const autoware_msgs::LaneConstPtr& msg);
  void CallbackCurrentOdometry(const nav_msgs::OdometryConstPtr &msg);

private:

  void CalculateClosestWaypoint();
  void PublishClosestIndex(const int& minIndex);
  void PublishLocalWaypoints(int minIndex,const autoware_msgs::Lane& lane);

  double CalculateDistance(const double &x1, const double &x2, const double &y1, const double &y2);
  
  void LocalPathPolynomialFitting(const nav_msgs::Path& local_path);
  cv::Mat polyfit(std::vector<cv::Point2f>& in_point, int n);
  double first_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X);
  double second_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pubClosestIndex;
  ros::Publisher pubGlobalTarget;
  ros::Publisher pubClosestIndexMarker;
  ros::Publisher pubBaseWaypoint;
  ros::Publisher pubLocalPathOnGlobal;
  ros::Publisher pubLocalPathOnBody;
  ros::Publisher pubLocalPathFitting;
  ros::Publisher pubCrossTrackError;
  ros::Publisher pubHeadingError;

  ros::Subscriber subCurrentOdom;
  ros::Subscriber subGlobalWaypoints;
  ros::Subscriber subCostMapBasedPath;
  ros::Subscriber sub_occupancy_grid_;

  std::shared_ptr<autoware_msgs::LaneArray> m_LaneArray_ptr;
  std::shared_ptr<autoware_msgs::Lane> m_CostMapBasedPath_ptr;
  std::shared_ptr<nav_msgs::Odometry> m_Odometry_ptr;
  std::vector<geometry_msgs::Pose> m_LocalWaypoint;
  autoware_msgs::Lane m_FinalLane;

  std::string m_BaseFrameID;
  std::string m_OdomTopic;
  int m_SetLocalWaypointSize;
  int m_SetLaneID;

  bool bLoadWaypoint, bCurrentOdometry;
  bool m_UseCostMapBasedPlanner;
  bool bCostMapBasedPath;
  int m_ClosestIndex;
  
};

#endif  // LOCAL_WPT_TRACKER_H
