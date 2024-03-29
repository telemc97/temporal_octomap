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
  worldFrameId("map"),
  colorFactor(0.8),
  sec(3600),
  nsec(0),
  latchedTopics(true),
  publishFreeSpace(false),
  publishMarkersTopic(true),
  res(1.0),
  treeDepth(0),
  maxTreeDepth(0),
  minSizeX(0.0), minSizeY(0.0),
  incrementalUpdate(false),
  debug(true)
  {
    double probHit, probMiss, thresMin, thresMax, occupancyThres;

    nodeHandle.param("frame_id", worldFrameId, worldFrameId);
    nodeHandle.param("color_factor", colorFactor, colorFactor);
    nodeHandle.param("publish_Markers_Topic", publishMarkersTopic,publishMarkersTopic);
    nodeHandle.param("min_x_size", minSizeX,minSizeX);
    nodeHandle.param("min_y_size", minSizeY,minSizeY);
    nodeHandle.param("max_range", maxRange,maxRange);
    nodeHandle.param("resolution", res,res);
    nodeHandle.param("publish_free_space", publishFreeSpace, publishFreeSpace);
    nodeHandle.param("sensor_model/hit", probHit, 0.85);
    nodeHandle.param("sensor_model/miss", probMiss, 0.15);
    nodeHandle.param("sensor_model/min", thresMin, 0.3);
    nodeHandle.param("sensor_model/max", thresMax, 0.7);
    nodeHandle.param("Occupancy_Thres",occupancyThres, 0.45);
    nodeHandle.param("incremental_2D_projection", incrementalUpdate, incrementalUpdate);
    nodeHandle.param("debug", debug, debug);

    nodeHandle.param("latch", latchedTopics, latchedTopics);

    nodeHandle.param("decaytime/sec", sec, sec);
    nodeHandle.param("decaytime/nsec", nsec, nsec);
    decaytime.sec = sec;
    decaytime.nsec = nsec;

    octree = new OcTreeT(res);
    octree->setProbHit(probHit);
    octree->setProbMiss(probMiss);
    octree->setClampingThresMin(thresMin);  
    octree->setClampingThresMax(thresMax);
    octree->setOccupancyThres(occupancyThres);
    
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
    
    if (debug){
      debugger = nodeHandle.advertise<temporal_octomap::TemporalOctomapDebug>("Temporal_Octomap_Debug", 1, latchedTopics);
    }

    PCLSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nodeHandle, "/PointCloud", 5);
    tfPCLSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*PCLSub, tfListener, worldFrameId, 5);
    tfPCLSub->registerCallback(boost::bind(&TemporalOctomap::insertCloudCallback, this, boost::placeholders::_1));

    //Timer-Based Callbacks' timers
    checkNodesUpdateInterval = nodeHandle.createTimer(ros::Rate(1), &TemporalOctomap::checkNodes, this);
    PublishOccupancy = nodeHandle.createTimer(ros::Rate(1), &TemporalOctomap::PublishOccupancyGrid, this);

    if (publishMarkersTopic){
      PublishMarkers = nodeHandle.createTimer(ros::Rate(1), &TemporalOctomap::publishMarkers, this);
    }

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
  auto start = std::chrono::high_resolution_clock::now();
  tf::StampedTransform sensorToWorldTf;
  tfListener.lookupTransform(worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  tf::Point sensorOriginTf = sensorToWorldTf.getOrigin();
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf); //Get sensor origin

  sensor_msgs::PointCloud2 cloudOut;
  pcl_ros::transformPointCloud(worldFrameId, *cloud, cloudOut, tfListener); //Transform PointCloud2 to world frame

  Pointcloud OctCloud;
  pointCloud2ToOctomap(cloudOut, OctCloud); //Convert PointCloud2 to octomap::PointCloud

  octree->insertPointCloud(OctCloud, sensorOrigin, maxRange, true, true); //Insert octomap::PointCloud to octree

  octomap::point3d minPt, maxPt;
  minPt = octree->keyToCoord(updateBBXMin);
  maxPt = octree->keyToCoord(updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << updateBBXMin[0] << " " <<updateBBXMin[1] << " " << updateBBXMin[2] << " / " <<updateBBXMax[0] << " "<<updateBBXMax[1] << " "<< updateBBXMax[2]);
  if (debug){
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    debug_msg.header.stamp = ros::Time::now();
    debug_msg.total_time = duration.count()*0.000001;
    debugger.publish(debug_msg);
    }

}


void TemporalOctomap::PublishOccupancyGrid(const ros::TimerEvent& event){
  gridmap.header.frame_id = worldFrameId;
  gridmap.header.stamp = ros::Time::now();
  nav_msgs::MapMetaData oldMapInfo = gridmap.info;
  double minX, minY, minZ, maxX, maxY, maxZ;
  octree->getMetricMax(maxX, maxY, maxZ);
  octree->getMetricMin(minX, minY, minZ);
  octomap::point3d minPt(minX, minY, minZ);
  octomap::point3d maxPt(maxX, maxY, maxZ);
  octomap::OcTreeKey minKey = octree->coordToKey(minPt, maxTreeDepth);
  octomap::OcTreeKey maxKey = octree->coordToKey(maxPt, maxTreeDepth);
  ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);
  double halfPaddedX = 0.5*minSizeX;
  double halfPaddedY = 0.5*minSizeY;
  minX = std::min(minX, -halfPaddedX);
  maxX = std::max(maxX, halfPaddedX);
  minY = std::min(minY, -halfPaddedY);
  maxY = std::max(maxY, halfPaddedY);
  minPt = octomap::point3d(minX, minY, minZ);
  maxPt = octomap::point3d(maxX, maxY, maxZ);
  OcTreeKey paddedMaxKey;
  if (!octree->coordToKeyChecked(minPt, maxTreeDepth, paddedMinKey)){
    ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
    return;
  }
  if (!octree->coordToKeyChecked(maxPt, maxTreeDepth, paddedMaxKey)){
    ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
    return;
  }
  ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", paddedMinKey[0], paddedMinKey[1], paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
  assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);
  multires2DScale = 1 <<(treeDepth - maxTreeDepth);
  uint32_t width = (paddedMaxKey[0] - paddedMinKey[0])/multires2DScale + 1;
  uint32_t height = (paddedMaxKey[1] - paddedMinKey[1])/multires2DScale + 1;
  if (height%2!=0){height = height+1;}
  if (width%2!=0){width = width+1;}
  gridmap.info.width = width;
  gridmap.info.height = height;
  int mapOriginX = minKey[0] - paddedMinKey[0];
  int mapOriginY = minKey[1] - paddedMinKey[1];
  assert(mapOriginX >= 0 && mapOriginY >= 0);
  octomap::point3d origin = octree->keyToCoord(paddedMinKey, treeDepth);
  double gridRes = octree->getNodeSize(maxTreeDepth);
  projectCompleteMap = (!incrementalUpdate || (std::abs(gridRes-gridmap.info.resolution) > 1e-6));
  gridmap.info.resolution = gridRes;
  gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
  gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
  if (maxTreeDepth != treeDepth){
    gridmap.info.origin.position.x -= res/2.0;
    gridmap.info.origin.position.y -= res/2.0;
  }
  if (maxTreeDepth < treeDepth){
    projectCompleteMap = true;
  }
  if(projectCompleteMap){
    ROS_DEBUG("Rebuilding complete 2D map");
    gridmap.data.clear();
    // init to unknown:
    gridmap.data.resize(gridmap.info.width * gridmap.info.height, 0);
  }else{
    if (mapChanged(oldMapInfo, gridmap.info)){
      ROS_DEBUG("2D grid map size changed to %dx%d", gridmap.info.width, gridmap.info.height);
      // adjustMapData(gridmap, oldMapInfo);
      if (gridmap.info.resolution != oldMapInfo.resolution){
        ROS_ERROR("Resolution of map changed, cannot be adjusted");
        return;
      }
      int i_off = int((oldMapInfo.origin.position.x - gridmap.info.origin.position.x)/gridmap.info.resolution +0.5);
      int j_off = int((oldMapInfo.origin.position.y - gridmap.info.origin.position.y)/gridmap.info.resolution +0.5);
      if (i_off < 0 || j_off < 0
          || oldMapInfo.width  + i_off > gridmap.info.width
          || oldMapInfo.height + j_off > gridmap.info.height)
      {
        ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
        return;
      }
      nav_msgs::OccupancyGrid::_data_type oldMapData = gridmap.data;
      gridmap.data.clear();
      // init to unknown:
      gridmap.data.resize(gridmap.info.width * gridmap.info.height, 0);
      nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;
      for (int j =0; j < int(oldMapInfo.height); ++j ){
        // copy chunks, row by row:
        fromStart = oldMapData.begin() + j*oldMapInfo.width;
        fromEnd = fromStart + oldMapInfo.width;
        toStart = gridmap.data.begin() + ((j+j_off)*gridmap.info.width + i_off);
        copy(fromStart, fromEnd, toStart);
      }
    }
    nav_msgs::OccupancyGrid::_data_type::iterator startIt;
    size_t mapUpdateBBXMinX = std::max(0, (int(updateBBXMin[0]) - int(paddedMinKey[0]))/int(multires2DScale));
    size_t mapUpdateBBXMinY = std::max(0, (int(updateBBXMin[1]) - int(paddedMinKey[1]))/int(multires2DScale));
    size_t mapUpdateBBXMaxX = std::min(int(gridmap.info.width-1), (int(updateBBXMax[0]) - int(paddedMinKey[0]))/int(multires2DScale));
    size_t mapUpdateBBXMaxY = std::min(int(gridmap.info.height-1), (int(updateBBXMax[1]) - int(paddedMinKey[1]))/int(multires2DScale));
    assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
    assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);
    size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;
    uint max_idx = gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
    if (max_idx  >= gridmap.data.size()){
      ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, gridmap.data.size(), gridmap.info.width, gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);
    }
    for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
    std::fill_n(gridmap.data.begin() + gridmap.info.width*j+mapUpdateBBXMinX,
                numCols, -1);
    }
  }
  for (OcTreeT::iterator it = octree->begin(), end = octree->end(); it != end; ++it){

    bool inUpdateBBX = isInUpdateBBX(it);
    bool occupied;

    if (octree->isNodeOccupied(*it)){
      occupied = true;
      if (projectCompleteMap){
        update2DMap(it, occupied);
      }
      if (inUpdateBBX){
        if (!projectCompleteMap){
          update2DMap(it, occupied);
        }
      }
    }else{
      occupied = false;
      if (projectCompleteMap){
        update2DMap(it, occupied);
      }
      if (inUpdateBBX){
        if (!projectCompleteMap){
          update2DMap(it, occupied);
        }
      }
    }
  }
  mapPub.publish(gridmap);
}


void TemporalOctomap::update2DMap(const OcTreeT::iterator& it, bool occupied){
  if (it.getDepth() == maxTreeDepth){
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
      gridmap.data[mapIdx(it.getKey())] = 100;
    else if (gridmap.data[idx] == -1){
      gridmap.data[idx] = 0;
    }

  } else{
    int intSize = 1 << (maxTreeDepth - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = (minKey[0]+dx - paddedMinKey[0])/multires2DScale;
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, (minKey[1]+dy - paddedMinKey[1])/multires2DScale);
        if (occupied)
          gridmap.data[idx] = 100;
        else if (gridmap.data[idx] == -1){
          gridmap.data[idx] = 0;
        }
      }
    }
  }
}


void TemporalOctomap::publishMarkers(const ros::TimerEvent& event){

  size_t octomapSize = octree->size();
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  // bool publishFreeMarkerArray = publishFreeSpace && (latchedTopics || fmarkerPub.getNumSubscribers() > 0);
  // bool publishMarkerArray = (latchedTopics || markerPub.getNumSubscribers() > 0);

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

    if (filterSpeckles && (it.getDepth() == treeDepth +1) && isSpeckleNode(it.getKey())){
      ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
      continue;
    } // else: current octree node is no speckle, send it out

    unsigned idx = it.getDepth();
    assert(idx < occupiedNodesVis.markers.size());

    geometry_msgs::Point cubeCenter;
    cubeCenter.x = x;
    cubeCenter.y = y;
    cubeCenter.z = z;

    if (octree->isNodeOccupied(*it)){
      occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
      occupiedNodesVis.markers[idx].color = getColor(getTimeLeft(it, ros::WallTime::now()));

    }else if(!octree->isNodeOccupied(*it)){
      freeNodesVis.markers[idx].points.push_back(cubeCenter);
    }
  }

  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

    double size = octree->getNodeSize(i);
    occupiedNodesVis.markers[i].header.frame_id = worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].pose = pose;


    if (occupiedNodesVis.markers[i].points.size() > 0){
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    }else{
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  markerPub.publish(occupiedNodesVis);

  if (publishFreeSpace){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){

      double size = octree->getNodeSize(i);
      freeNodesVis.markers[i].header.frame_id = worldFrameId;
      freeNodesVis.markers[i].header.stamp = ros::Time::now();
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
}


//Delete unseen Nodes and other stuff;
void TemporalOctomap::checkNodes(const ros::TimerEvent& event){
  OcTreeKey nodeKey;
  for (OcTreeT::iterator it = octree->begin(), end = octree->end(); it != end; ++it){
    int timeleft = getTimeLeft(it, ros::WallTime::now());
    if (octree->isNodeOccupied(*it) && timeleft == 0){
      nodeKey = it.getKey();
      octree->deleteNode(nodeKey);
    }
  }
}


int TemporalOctomap::getTimeLeft(const OcTreeT::iterator& it, const ros::WallTime& roswalltime){
  int time_left;
  unsigned int stamp = it->getTimestamp();
  time_left = decaytime.toSec() - (roswalltime.toSec() - stamp);
  if (time_left<0){time_left = 0;}
  return time_left;
}


std_msgs::ColorRGBA TemporalOctomap::getColor(const int timeLeft){
  std_msgs::ColorRGBA newColor;
  newColor.a = 1.0;
  newColor.g = 0.0;
  double timeleft = 1.0 - (decaytime.toSec()-timeLeft)/decaytime.toSec();
  if (timeleft>0.5 && timeleft<=1.0){
    newColor.r = 1.0;
    newColor.b = (-2*timeleft + 2);
  }else if(timeleft>=0 && timeleft <=0.5){
    newColor.b = 1.0;
    newColor.r = (2*timeleft);
  }

  return newColor;
}


bool TemporalOctomap::isSpeckleNode(const OcTreeKey&nKey) const {
  OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          OcTreeNode* node = octree->search(key);
          if (node && octree->isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }
  return neighborFound;
}


}