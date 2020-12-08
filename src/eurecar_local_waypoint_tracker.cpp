#include "eurecar_local_waypoint_tracker/eurecar_local_waypoint_tracker.h"
#include <string>
#include <assert.h>


// Constructor
LocalWaypointTracker::LocalWaypointTracker() : nh_(""), private_nh_("~"),
                                       bLoadWaypoint(false),bCurrentOdometry(false), m_ClosestIndex(-1), bCostMapBasedPath(false)
{
  private_nh_.param("base_frame_id", m_BaseFrameID, std::string("base_link"));
  private_nh_.param("use_costmap_based_local_planner", m_UseCostMapBasedPlanner, bool(false));
  private_nh_.param("set_local_waypoint_size", m_SetLocalWaypointSize, int(50));
  private_nh_.param("set_lane_id", m_SetLaneID, int(0));
  private_nh_.param("odometry_topic", m_OdomTopic, std::string("/Odometry/final"));
  

  pubClosestIndex = nh_.advertise<std_msgs::Int32>("/closest_waypoint", 1);
  pubGlobalTarget = nh_.advertise<geometry_msgs::Pose>("/global_goal", 1);
  pubClosestIndexMarker = nh_.advertise<visualization_msgs::Marker>("/Marker/closest_waypoint", 1);
  pubBaseWaypoint = nh_.advertise<autoware_msgs::Lane>("/Lane/LocalWaypoint/OnGlobal", 1);
  pubLocalPathOnGlobal = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/OnGlobal", 1);
  pubLocalPathOnBody = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/OnBody", 1);
  pubLocalPathFitting = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/Fitting", 1);
  pubCrossTrackError = nh_.advertise<std_msgs::Float64>("/Float64/tracker/cross_track_error", 1);
  pubHeadingError    = nh_.advertise<std_msgs::Float64>("/Float64/tracker/heading_error", 1);


  subCurrentOdom = nh_.subscribe(m_OdomTopic, 1, &LocalWaypointTracker::CallbackCurrentOdometry, this);
  subGlobalWaypoints = nh_.subscribe("/lane_waypoints_array", 1, &LocalWaypointTracker::CallbackGlobalWaypoints, this);
  subCostMapBasedPath = nh_.subscribe("/final_waypoints", 1, &LocalWaypointTracker::CallbackCostMapBasedPath, this);
  
  init();

}

LocalWaypointTracker::~LocalWaypointTracker(){}

void LocalWaypointTracker::init()
{
  ROS_INFO("init eurecar_local_waypoint_tracker");
}

void LocalWaypointTracker::run()
{
  if(bCurrentOdometry && bLoadWaypoint)
  {
    CalculateClosestWaypoint();
    PublishClosestIndex(m_ClosestIndex);
    PublishLocalWaypoints(m_ClosestIndex, m_FinalLane);
  }
}

void LocalWaypointTracker::CallbackCurrentOdometry(const nav_msgs::OdometryConstPtr &msg)
{
  m_Odometry_ptr = std::make_shared<nav_msgs::Odometry>(*msg);
  bCurrentOdometry = true;  
}

void LocalWaypointTracker::CallbackGlobalWaypoints(const autoware_msgs::LaneArrayConstPtr& msg)
{

  m_LaneArray_ptr = std::make_shared<autoware_msgs::LaneArray>(*msg);
  bLoadWaypoint = true;
}

void LocalWaypointTracker::CallbackCostMapBasedPath(const autoware_msgs::LaneConstPtr& msg)
{
  m_CostMapBasedPath_ptr = std::make_shared<autoware_msgs::Lane>(*msg);
  bCostMapBasedPath = true;
}

void LocalWaypointTracker::CalculateClosestWaypoint()
{
  if(!m_LaneArray_ptr)
  {
    ROS_WARN("No received Waypoints");
    return;
  }
  if(!m_Odometry_ptr)
  {
    ROS_WARN("No received Vehciel Odometry");
    return;
  }
  size_t TotalLanes = m_LaneArray_ptr->lanes.size();

  double veh_x = m_Odometry_ptr->pose.pose.position.x;
  double veh_y = m_Odometry_ptr->pose.pose.position.y;
  tf::Quaternion q_(m_Odometry_ptr->pose.pose.orientation.x, 
                    m_Odometry_ptr->pose.pose.orientation.y,
                    m_Odometry_ptr->pose.pose.orientation.z, 
                    m_Odometry_ptr->pose.pose.orientation.w);
  tf::Matrix3x3 m_veh(q_);
  double veh_roll, veh_pitch, veh_yaw;
  m_veh.getRPY(veh_roll, veh_pitch, veh_yaw);
  
  for(int LaneID = 0; LaneID < TotalLanes; LaneID++)
  {
    if(LaneID == m_SetLaneID)
    {
      size_t WaypointSize = m_LaneArray_ptr->lanes[LaneID].waypoints.size();
      autoware_msgs::Lane lane;
      std::cout << "m_UseCostMapBasedPlanner: " << m_UseCostMapBasedPlanner << std::endl;
      if(m_UseCostMapBasedPlanner && bCostMapBasedPath && !m_CostMapBasedPath_ptr->waypoints.empty())
      {
        lane = *m_CostMapBasedPath_ptr;
        ROS_INFO("Cost map based planner");
      }
      else
      {
        lane = m_LaneArray_ptr->lanes[LaneID];
        ROS_INFO("waypoint tracker");
      }
      double minDistance = DBL_MAX;
      m_ClosestIndex = -1;
      for(int i = 0; i < lane.waypoints.size(); i++)
      {
        double wpt_x = lane.waypoints[i].pose.pose.position.x;
        double wpt_y = lane.waypoints[i].pose.pose.position.y;
        tf::Quaternion q(lane.waypoints[i].pose.pose.orientation.x, 
                         lane.waypoints[i].pose.pose.orientation.y,
                         lane.waypoints[i].pose.pose.orientation.z, 
                         lane.waypoints[i].pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double wpt_roll, wpt_pitch, wpt_yaw;
        m.getRPY(wpt_roll, wpt_pitch, wpt_yaw);
        double distance = CalculateDistance(veh_x, wpt_x, veh_y, wpt_y);
        double heading_error = fabs(veh_yaw - wpt_yaw);
        if(heading_error > 2* M_PI)
        {
          heading_error = heading_error - 2* M_PI;
        }
        heading_error = heading_error * heading_error;
        
        if(distance + heading_error < minDistance)
        { 
          minDistance = distance  + heading_error;
          m_ClosestIndex = i;
        }
      }
      m_FinalLane = lane;
      std::cout << "Total num of WPT: " << m_FinalLane.waypoints.size() << std::endl;
      std::cout << "m_ClosestIndex : " << m_ClosestIndex << std::endl;
      std::cout << "minDistance : " << minDistance << std::endl;
    }
  }

}

void LocalWaypointTracker::PublishClosestIndex(const int& minIndex)
{
  if(minIndex == -1)
    ROS_WARN("Closest index value is -1.");

  std_msgs::Int32 minIndex_msg;
  minIndex_msg.data = minIndex;
  pubClosestIndex.publish(minIndex_msg);

  visualization_msgs::Marker minIndexMarker;
  minIndexMarker.header.stamp = ros::Time::now();
  minIndexMarker.header.frame_id = "map";
  minIndexMarker.id = minIndex;
  minIndexMarker.type = visualization_msgs::Marker::SPHERE;
  minIndexMarker.action = visualization_msgs::Marker::MODIFY;
  minIndexMarker.pose.position.x = m_FinalLane.waypoints[minIndex].pose.pose.position.x;
  minIndexMarker.pose.position.y = m_FinalLane.waypoints[minIndex].pose.pose.position.y;
  minIndexMarker.pose.orientation.w = 1;
  minIndexMarker.scale.x = 1, minIndexMarker.scale.y = 1, minIndexMarker.scale.z = 1;
  minIndexMarker.color.g = 1, minIndexMarker.color.a = 1;
  minIndexMarker.lifetime = ros::Duration(0.2);
  pubClosestIndexMarker.publish(minIndexMarker);

  geometry_msgs::Pose GoalPose;
  if(m_FinalLane.waypoints.size() -  minIndex > 6)
  {
    GoalPose.position.x = m_FinalLane.waypoints[minIndex + 5].pose.pose.position.x;
    GoalPose.position.y = m_FinalLane.waypoints[minIndex + 5].pose.pose.position.y;
  }
  else
  {
    GoalPose.position.x = m_FinalLane.waypoints[m_FinalLane.waypoints.size()-1].pose.pose.position.x;
    GoalPose.position.y = m_FinalLane.waypoints[m_FinalLane.waypoints.size()-1].pose.pose.position.y;
  }
  pubGlobalTarget.publish(GoalPose);    

}


void LocalWaypointTracker::PublishLocalWaypoints(int minIndex, const autoware_msgs::Lane& lane)
{
  if(lane.waypoints.size() == 0)
    ROS_WARN("Lane is Empty.");
  auto remained_size = lane.waypoints.size() - minIndex;
  int CutWptSize;
  nav_msgs::Path LocalWaypoint_OnGlobalCoord;
  LocalWaypoint_OnGlobalCoord.header.stamp = m_Odometry_ptr->header.stamp;
  LocalWaypoint_OnGlobalCoord.header.frame_id = "map";
  
  nav_msgs::Path LocalWaypoint_OnBodyCoord;
  LocalWaypoint_OnBodyCoord.header.stamp = m_Odometry_ptr->header.stamp;
  LocalWaypoint_OnBodyCoord.header.frame_id = m_BaseFrameID;

  if(remained_size > m_SetLocalWaypointSize)
  {
    CutWptSize = m_SetLocalWaypointSize;
  }
  else
  {
    CutWptSize = remained_size;
  }

  double veh_x = m_Odometry_ptr->pose.pose.position.x;
  double veh_y = m_Odometry_ptr->pose.pose.position.y;
  
  double veh_roll, veh_pitch, veh_yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(m_Odometry_ptr->pose.pose.orientation ,q);
  tf::Matrix3x3 m(q);
  m.getRPY(veh_roll, veh_pitch, veh_yaw);

  for(int i = 0; i < CutWptSize; i++)
  {
    geometry_msgs::PoseStamped wpt;
    wpt.pose.position.x = lane.waypoints[i + minIndex].pose.pose.position.x;
    wpt.pose.position.y = lane.waypoints[i + minIndex].pose.pose.position.y;
    wpt.pose.position.z = lane.waypoints[i + minIndex].pose.pose.position.z;
    wpt.pose.orientation = lane.waypoints[i + minIndex].pose.pose.orientation;

    LocalWaypoint_OnGlobalCoord.poses.push_back(wpt);

    geometry_msgs::PoseStamped OnBodyCoord;
    OnBodyCoord.pose.position.x = (lane.waypoints[i + minIndex].pose.pose.position.x - veh_x) * cos(veh_yaw) +
                                  (lane.waypoints[i + minIndex].pose.pose.position.y - veh_y) * sin(veh_yaw);
    OnBodyCoord.pose.position.y = (lane.waypoints[i + minIndex].pose.pose.position.x - veh_x) * -sin(veh_yaw) +
                                  (lane.waypoints[i + minIndex].pose.pose.position.y - veh_y) * cos(veh_yaw);

    // OnBodyCoord.pose.position.z = lane.waypoints[i + minIndex].twist.twist.linear.x * 0.0001;
    LocalWaypoint_OnBodyCoord.poses.push_back(OnBodyCoord);

  }
  pubLocalPathOnGlobal.publish(LocalWaypoint_OnGlobalCoord);
  pubLocalPathOnBody.publish(LocalWaypoint_OnBodyCoord);
  pubBaseWaypoint.publish(lane);
  
  LocalPathPolynomialFitting(LocalWaypoint_OnBodyCoord);
}

double LocalWaypointTracker::CalculateDistance(const double &x1, const double &x2, const double &y1, const double &y2)
{
  double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  return distance;
}

//For polynomial fitting
void LocalWaypointTracker::LocalPathPolynomialFitting(const nav_msgs::Path& local_path){
    
  if(local_path.poses.size() < 5)
    return;

  std::vector<cv::Point2f> ToBeFit;
  ToBeFit.clear();
  
  for (auto pose : local_path.poses)
  {
    cv::Point2f pointTmp;
    pointTmp.x = pose.pose.position.x;
    pointTmp.y = pose.pose.position.y;
    ToBeFit.push_back(pointTmp);

    if(ToBeFit.size() > 15)
      break;
  }

  //For polynomial fitting
  int poly_order = 3;
  cv::Mat PolyCoefficient = polyfit(ToBeFit, poly_order);

  nav_msgs::Path LocalWaypointFitting;

  LocalWaypointFitting.header = local_path.header;
  for (uint32_t i = 0; i < ToBeFit.size(); i++)
  {
    geometry_msgs::PoseStamped poseTmp;
    poseTmp.pose.position.x = (double)ToBeFit[i].x;
    poseTmp.pose.position.y = PolyCoefficient.at<double>(poly_order  , 0)* pow(ToBeFit[i].x,poly_order) + 
                              PolyCoefficient.at<double>(poly_order-1, 0)* pow(ToBeFit[i].x,poly_order-1) + 
                              PolyCoefficient.at<double>(poly_order-2, 0)* pow(ToBeFit[i].x,poly_order-2) + 
                              PolyCoefficient.at<double>(poly_order-3, 0); //Cubic

    LocalWaypointFitting.poses.push_back(poseTmp);
  }
  pubLocalPathFitting.publish(LocalWaypointFitting);

  //Calculate Cross-track error
  std_msgs::Float64 CrossTrackErrorMsg;
  double LookAheadPoint = 1;
  double CrossTrackError = PolyCoefficient.at<double>(poly_order  , 0)* pow(LookAheadPoint, poly_order) + 
                           PolyCoefficient.at<double>(poly_order-1, 0)* pow(LookAheadPoint, poly_order-1) + 
                           PolyCoefficient.at<double>(poly_order-2, 0)* pow(LookAheadPoint, poly_order-2) + 
                           PolyCoefficient.at<double>(poly_order-3, 0); //Cubic
  CrossTrackError = fabs(CrossTrackError); //[m]
  if(CrossTrackError > 5)
    CrossTrackError = 5; //Saturation value;
  CrossTrackErrorMsg.data = CrossTrackError;

  //Calculate Heading error
  std_msgs::Float64 HeadingErrorMsg;
  double slope = first_derivative_of_cubic_poly(PolyCoefficient, LookAheadPoint);
  double HeadingError = atan2(slope , 1) * 180 / M_PI; //[deg]
  HeadingErrorMsg.data = HeadingError;
  
  std::cout << "CrossTrackError: " << CrossTrackError << "\n" << "Heading Error: " << HeadingError << std::endl;

  pubCrossTrackError.publish(CrossTrackErrorMsg);
  pubHeadingError.publish(HeadingErrorMsg);
}

double LocalWaypointTracker::first_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X){
  double Y_dot = 3 * input_poly_coeffi.at<double>(3, 0)* X* X + 
                 2 * input_poly_coeffi.at<double>(2, 0)* X + 
                 input_poly_coeffi.at<double>(1, 0);
  return Y_dot;
}

double LocalWaypointTracker::second_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X){
  double Y_dot = 6 * input_poly_coeffi.at<double>(3, 0)* X + 
                 2 * input_poly_coeffi.at<double>(2, 0);
  return Y_dot;
}

cv::Mat LocalWaypointTracker::polyfit(std::vector<cv::Point2f>& in_point, int n)
{
  int size = in_point.size();

  int x_num = n + 1;

  cv::Mat mat_u(size, x_num, CV_64F);
  cv::Mat mat_y(size, 1, CV_64F);
  cv::Mat mat_k(x_num, 1, CV_64F);

  if (size == 0){
      for (int i = 0; i < mat_k.rows; ++i){
          mat_k.at<double>(i,0) = 0;
      }
      return mat_k;
  }
  else{
      for (int i = 0; i < mat_u.rows; ++i)
              for (int j = 0; j < mat_u.cols; ++j)
              {
                      mat_u.at<double>(i, j) = pow(in_point[i].x, j);
              }

      for (int i = 0; i < mat_y.rows; ++i)
      {
              mat_y.at<double>(i, 0) = in_point[i].y;
      }

      mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
      
      return mat_k;
  }
}
