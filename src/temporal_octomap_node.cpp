#include <ros/ros.h>
#include <temporal_octomap/TemporalOctomap.h>

using namespace temporal_octomap;

int main(int argc, char** argv){

  ros::init(argc, argv, "temporal_octomap");
  const ros::NodeHandle nh;

  TemporalOctomap server(nh);

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();

  return 0;
}
