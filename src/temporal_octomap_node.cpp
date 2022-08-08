#include <ros/ros.h>
#include <temporal_octomap/TemporalOctomap.h>

using namespace temporal_octomap;

int main(int argc, char** argv){

  ros::init(argc, argv, "temporal_octomap");
  const ros::NodeHandle nh;


  TemporalOctomap server(nh);
  ros::spinOnce();

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}