#include <temporal_octomap/TemporalOctomap.h>

using namespace octomap;
using octomap_msgs::Octomap;

namespace temporal_octomap{

TemporalOctomap::TemporalOctomap(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  PCLSub(NULL),
  tfPCLSub(NULL),
  octree(NULL),
  maxRange(15.0),
  minRange(2.0),
  worldFrameId("map"), baseFrameId("base_footprint"),
  useHeightMap(false),
  colorFactor(0.8),
  latchedTopics(true),
  publishFreeSpace(false),
  res(0.8),
  treeDepth(0),
  maxTreeDepth(0),
  pointcloudMinX(-std::numeric_limits<double>::max()),
  pointcloudMaxX(std::numeric_limits<double>::max()),
  pointcloudMinY(-std::numeric_limits<double>::max()),
  pointcloudMaxY(std::numeric_limits<double>::max()),
  pointcloudMinZ(-std::numeric_limits<double>::max()),
  pointcloudMaxZ(std::numeric_limits<double>::max()),
  occupancyMinZ(-std::numeric_limits<double>::max()),
  occupancyMaxZ(std::numeric_limits<double>::max()),
  minSizeX(0.0), minSizeY(0.0),
  incrementalUpdate(false)
  {
    double probHit, probMiss, thresMin, thresMax;

    nodeHandle.param("frame_id", worldFrameId, worldFrameId);
    nodeHandle.param("base_frame_id", baseFrameId, baseFrameId);
    nodeHandle.param("height_map", useHeightMap, useHeightMap);
    nodeHandle.param("color_factor", colorFactor, colorFactor);
    nodeHandle.param("pointcloud_min_x", pointcloudMinX,pointcloudMinX);
    nodeHandle.param("pointcloud_max_x", pointcloudMaxX,pointcloudMaxX);
    nodeHandle.param("pointcloud_min_y", pointcloudMinY,pointcloudMinY);
    nodeHandle.param("pointcloud_max_y", pointcloudMaxY,pointcloudMaxY);
    nodeHandle.param("pointcloud_min_z", pointcloudMinZ,pointcloudMinZ);
    nodeHandle.param("pointcloud_max_z", pointcloudMaxZ,pointcloudMaxZ);
    nodeHandle.param("occupancy_min_z", occupancyMinZ,0.0);
    nodeHandle.param("occupancy_max_z", occupancyMaxZ,30.0);
    nodeHandle.param("min_x_size", minSizeX,minSizeX);
    nodeHandle.param("min_y_size", minSizeY,minSizeY);
    nodeHandle.param("min_range", maxRange,maxRange);
    nodeHandle.param("max_range", minRange,minRange);
    nodeHandle.param("resolution", res,res);
    nodeHandle.param("publish_free_space", publishFreeSpace, publishFreeSpace);
    nodeHandle.param("sensor_model/hit", probHit, 0.7);
    nodeHandle.param("sensor_model/miss", probMiss, 0.4);
    nodeHandle.param("sensor_model/min", thresMin, 0.12);
    nodeHandle.param("sensor_model/max", thresMax, 0.97);
    nodeHandle.param("incremental_2D_projection", incrementalUpdate, incrementalUpdate);

    nodeHandle.param("latch", latchedTopics, latchedTopics);

    double sec, nsec;
    nodeHandle.param("decaytime/sec", sec, 15.0);
    nodeHandle.param("decaytime/nsec", nsec, 0.0);
    decaytime.sec = sec;
    decaytime.nsec = nsec;

    octree = new OcTreeT(res);
    octree->setProbHit(probHit);
    octree->setProbMiss(probMiss);
    octree->setClampingThresMin(thresMin);  
    octree->setClampingThresMax(thresMax);
    
    treeDepth = octree->getTreeDepth();
    maxTreeDepth = treeDepth;
    res = octree->getResolution();
    gridmap.info.resolution = res;

    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree->getMetricMin(minX, minY, minZ);
    octree->getMetricMax(maxX, maxY, maxZ);



    markerPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, latchedTopics);
    fmarkerPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, latchedTopics);
    mapPub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, latchedTopics);

    PCLSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nodeHandle, "/PointCloud", 5);
    tfPCLSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*PCLSub, tfListener, worldFrameId, 5);
    tfPCLSub->registerCallback(boost::bind(&TemporalOctomap::insertCloudCallback, this, boost::placeholders::_1));

    updateInterval = nodeHandle.createTimer(ros::Duration(0.5), &TemporalOctomap::checkNodes, this);

    

    color.a = 1.0;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;

    colorFree.a = 1.0;
    colorFree.r = 0.0;
    colorFree.g = 1.0;
    colorFree.b = 0.0;

  }

  TemporalOctomap::~TemporalOctomap(){
    if (tfPCLSub){
      delete tfPCLSub;
      tfPCLSub = NULL;
    }

    if (PCLSub){
      delete PCLSub;
      PCLSub = NULL;
    }

    if (octree){
      delete octree;
      octree = NULL;
    }
  }





//USING PointCloud2
void TemporalOctomap::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){

  tf::StampedTransform sensorToWorldTf;
  tfListener.lookupTransform(worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  tf::Point sensorOriginTf = sensorToWorldTf.getOrigin();
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf); //Get sensor origin

  sensor_msgs::PointCloud2 cloudOut;
  pcl_ros::transformPointCloud(worldFrameId, *cloud, cloudOut, tfListener); //Transform PointCloud2 to world frame

  Pointcloud OctCloud;
  pointCloud2ToOctomap(cloudOut, OctCloud); //Convert PointCloud2 to octomap::PointCloud

  octree->insertPointCloud(OctCloud, sensorOrigin, maxRange, false, true); //Insert octomap::PointCloud to octree

  octomap::point3d minPt, maxPt;
  minPt = octree->keyToCoord(updateBBXMin);
  maxPt = octree->keyToCoord(updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << updateBBXMin[0] << " " <<updateBBXMin[1] << " " << updateBBXMin[2] << " / " <<updateBBXMax[0] << " "<<updateBBXMax[1] << " "<< updateBBXMax[2]);
  

  publishAll(cloud->header.stamp);

}


//CONVERTING TO pcl::PointCloud<pcl::PointXYZ> PCLPointCloud INSTEAD OF PointCloud2
// void TemporalOctomap::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){

//   tf::StampedTransform sensorToWorldTf;
//   tfListener.lookupTransform(worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
//   tf::Point sensorOriginTf = sensorToWorldTf.getOrigin();
//   point3d sensorOrigin = pointTfToOctomap(sensorOriginTf); //Get sensor origin

//   PCLPointCloud cloudIn;
//   pcl::fromROSMsg(*cloud, cloudIn);

//   PCLPointCloud cloudOut;
//   pcl_ros::transformPointCloud(worldFrameId, cloudIn, cloudOut, tfListener); //Transform PointCloud to world frame

//   Pointcloud OctCloud;
//   pointcloudPCLToOctomap(cloudOut, OctCloud); //Convert PointCloud to octomap::PointCloud

//   octree->insertPointCloud(OctCloud, sensorOrigin); //Insert octomap::PointCloud to octree

//   octomap::point3d minPt, maxPt;
//   minPt = octree->keyToCoord(updateBBXMin);
//   maxPt = octree->keyToCoord(updateBBXMax);
//   ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
//   ROS_DEBUG_STREAM("Bounding box keys (after): " << updateBBXMin[0] << " " <<updateBBXMin[1] << " " << updateBBXMin[2] << " / " <<updateBBXMax[0] << " "<<updateBBXMax[1] << " "<< updateBBXMax[2]);

// }

void TemporalOctomap::publishAll(const ros::Time& rostime){

  size_t octomapSize = octree->size();
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishFreeMarkerArray = publishFreeSpace && (latchedTopics || fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (latchedTopics || markerPub.getNumSubscribers() > 0);

  visualization_msgs::MarkerArray occupiedNodesVis;
  visualization_msgs::MarkerArray freeNodesVis;
  
  freeNodesVis.markers.resize(treeDepth+1);
  occupiedNodesVis.markers.resize(treeDepth+1);
  
  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  for (OcTreeT::iterator it = octree->begin(), end = octree->end(); it != end; ++it){

    double size = it.getSize();
    double x = it.getX();
    double y = it.getY();
    double z = it.getZ();

    unsigned idx = it.getDepth();
    assert(idx < occupiedNodesVis.markers.size());

    geometry_msgs::Point cubeCenter;
    cubeCenter.x = x;
    cubeCenter.y = y;
    cubeCenter.z = z;

    if (octree->isNodeOccupied(*it)){
      occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

    }else if(!octree->isNodeOccupied(*it)){
      freeNodesVis.markers[idx].points.push_back(cubeCenter);
    }
  }

  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

    double size = octree->getNodeSize(i);
    occupiedNodesVis.markers[i].header.frame_id = worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].pose = pose;
    occupiedNodesVis.markers[i].color = color;

    if (occupiedNodesVis.markers[i].points.size() > 0){
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    }else{
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  markerPub.publish(occupiedNodesVis);

  for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){

    double size = octree->getNodeSize(i);
    freeNodesVis.markers[i].header.frame_id = worldFrameId;
    freeNodesVis.markers[i].header.stamp = rostime;
    freeNodesVis.markers[i].ns = "map";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].scale.x = size;
    freeNodesVis.markers[i].scale.y = size;
    freeNodesVis.markers[i].scale.z = size;
    freeNodesVis.markers[i].pose = pose;
    freeNodesVis.markers[i].color = colorFree;

    if (freeNodesVis.markers[i].points.size() > 0){
      freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    }else{
      freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  fmarkerPub.publish(freeNodesVis);


}






//Delete unseen Nodes;
void TemporalOctomap::checkNodes(const ros::TimerEvent& event){
  ROS_WARN("checkNodes called");
  OcTreeKey nodeKey;
  for (OcTreeT::iterator it = octree->begin(), end = octree->end(); it != end; ++it){
    int timeleft = getTimeLeft(it, ros::Time::now());
    if (timeleft = 0){
      nodeKey = it.getKey();
    }
    octree->deleteNode(nodeKey);
  }
}






int TemporalOctomap::getTimeLeft(const OcTreeT::iterator& it, const ros::Time& rostime){
  int time_left;
  unsigned int stamp = it->getTimestamp();
  int nsecs = (stamp % 1000) * 1000 * 1000;
  int secs = stamp / 1000;
  ros::Time state_timestamp(secs, nsecs);
  time_left = decaytime.toSec() - (rostime - state_timestamp).toSec();
  if (time_left<0){time_left = 0;}
  return time_left;
}






std_msgs::ColorRGBA TemporalOctomap::getColor(int time){
  std_msgs::ColorRGBA newColor;
  newColor.a = 1.0;
  newColor.b = 0.0;
  double timeleft = time/decaytime.toSec();
  if (timeleft>=0.5){
    double blue = -((timeleft-0.5)/0.001961);
    newColor.r = 1.0;
    newColor.b = blue;
  }else{
    double red = (510*timeleft);
    newColor.b = 1.0;
    newColor.r = red;
  }
  return newColor;
}






}