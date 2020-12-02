#include "eurecar_local_waypoint_tracker/eurecar_local_waypoint_tracker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_check_with_path");
  LocalWaypointTracker local_waypoint_tracker;
  
  ros::Rate loop_rate(20);
  while(ros::ok())
  {
    ros::spinOnce();
    local_waypoint_tracker.run();
    loop_rate.sleep();
  }


  return 0;
}
