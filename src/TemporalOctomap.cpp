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

    octree = new OcTree(res);
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

    clearBBXService = nodeHandle.advertiseService("clear_bbx", &TemporalOctomap::clearBBXSrv, this);
    resetService = nodeHandle.advertiseService("reset", &TemporalOctomap::resetSrv, this);

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






void TemporalOctomap::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){

  tf::StampedTransform sensorToWorldTf;
  tfListener.lookupTransform(worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  tf::Point sensorOriginTf = sensorToWorldTf.getOrigin();
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf); //Get sensor origin

  sensor_msgs::PointCloud2 cloudOut;
  pcl_ros::transformPointCloud(worldFrameId, *cloud, cloudOut, tfListener); //Transform PointCloud2 to world frame

  octomap::Pointcloud OctCloud;
  pointCloud2ToOctomap(cloudOut, OctCloud); //Convert PointCloud2 to octomap::PointCloud

  octree->insertPointCloud(OctCloud, sensorOrigin); //Insert octomap::PointCloud to octree

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
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = octree->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
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

  handlePreNodeTraversal(rostime);

  for (OcTreeT::iterator it = octree->begin(maxTreeDepth),
      end = octree->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (octree->isNodeOccupied(*it)){ // node occupied
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > occupancyMinZ && z - half_size < occupancyMaxZ){

        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();

        handleOccupiedNode(it);

        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);

        if (publishMarkerArray){

          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;
          if (occupiedNodesVis.markers[idx].lifetime.isZero()){
            occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            double size = octree->getNodeSize(idx);
            occupiedNodesVis.markers[idx].header.frame_id = worldFrameId;
            occupiedNodesVis.markers[idx].header.stamp = rostime;
            occupiedNodesVis.markers[idx].ns = "map";
            occupiedNodesVis.markers[idx].id = idx;
            occupiedNodesVis.markers[idx].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVis.markers[idx].scale.x = size;
            occupiedNodesVis.markers[idx].scale.y = size;
            occupiedNodesVis.markers[idx].scale.z = size;
            occupiedNodesVis.markers[idx].lifetime = decaytime;
            occupiedNodesVis.markers[idx].pose = pose;
            occupiedNodesVis.markers[idx].color = getColor(occupiedNodesVis.markers[idx].lifetime.toSec());
          }
          if (useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            octree->getMetricMin(minX, minY, minZ);
            octree->getMetricMax(maxX, maxY, maxZ);
          }
        }
      }
    }else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if(z + half_size > occupancyMinZ && z - half_size < occupancyMaxZ){
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);

        if (publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());
            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
            double size = octree->getNodeSize(idx);
            freeNodesVis.markers[idx].header.frame_id = worldFrameId;
            freeNodesVis.markers[idx].header.stamp = rostime;
            freeNodesVis.markers[idx].ns = "map";
            freeNodesVis.markers[idx].id = idx;
            freeNodesVis.markers[idx].type = visualization_msgs::Marker::CUBE_LIST;
            freeNodesVis.markers[idx].scale.x = size;
            freeNodesVis.markers[idx].scale.y = size;
            freeNodesVis.markers[idx].scale.z = size;
            freeNodesVis.markers[idx].lifetime = decaytime;
            freeNodesVis.markers[idx].pose = pose;
            freeNodesVis.markers[idx].color = colorFree;
          }
        }
      }
    }
  }
  handlePostNodeTraversal(rostime);

  // finish occupied marker array
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      if (occupiedNodesVis.markers[i].points.size() > 0){
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      }else{
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }
    markerPub.publish(occupiedNodesVis);
  }

  // finish free marker array
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      if (freeNodesVis.markers[i].points.size() > 0){
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      }else{
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }
    fmarkerPub.publish(freeNodesVis);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

}






void TemporalOctomap::update2DMap(const OcTreeT::iterator& it, bool occupied){
  if (it.getDepth() == maxTreeDepth){
    unsigned idx = mapIdx(it.getKey());
    if(occupied){
      gridmap.data[idx] = 100;
    }else{
      gridmap.data[idx] = 0;
    }
  }else{

    int intSize = 1 << (maxTreeDepth - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = (minKey[0]+dx - paddedMinKey[0])/multires2DScale;
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, (minKey[1]+dy - paddedMinKey[1])/multires2DScale);
        if (occupied){
          gridmap.data[idx] = 100;
        }else if(gridmap.data[idx] == -1){
          gridmap.data[idx] = 0;
        }
      }
    }
  }
}






void TemporalOctomap::handlePreNodeTraversal(const ros::Time& rostime){
  if (publish2DMap){
    gridmap.header.frame_id = worldFrameId;
    gridmap.header.stamp = rostime;
    nav_msgs::MapMetaData oldMapInfo = gridmap.info;

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree->getMetricMin(minX, minY, minZ);
    octree->getMetricMax(maxX, maxY, maxZ);

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

    OcTreeKey paddedMaxKey, paddedMinKey;
    if (!octree->coordToKeyChecked(minPt, maxTreeDepth, paddedMinKey)){
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!octree->coordToKeyChecked(maxPt, maxTreeDepth, paddedMaxKey)){
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }

    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    multires2DScale = 1 << (treeDepth - maxTreeDepth);
    gridmap.info.width = (paddedMaxKey[0] - paddedMinKey[0])/multires2DScale +1;
    gridmap.info.height = (paddedMaxKey[1] - paddedMinKey[1])/multires2DScale +1;

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

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (maxTreeDepth < treeDepth)
      projectCompleteMap = true;

    if(projectCompleteMap){
      ROS_DEBUG("Rebuilding complete 2D map");
      gridmap.data.clear();
      // init to unknown:
      gridmap.data.resize(gridmap.info.width * gridmap.info.height, -1);
    }else{
      if (mapChanged(oldMapInfo, gridmap.info)){
          ROS_DEBUG("2D grid map size changed to %dx%d", gridmap.info.width, gridmap.info.height);
          adjustMapData(gridmap, oldMapInfo);
       }
       nav_msgs::OccupancyGrid::_data_type::iterator startIt;
       size_t mapUpdateBBXMinX = std::max(0, (int(updateBBXMin[0]) - int(paddedMinKey[0]))/int(multires2DScale));
       size_t mapUpdateBBXMinY = std::max(0, (int(updateBBXMin[1]) - int(paddedMinKey[1]))/int(multires2DScale));
       size_t mapUpdateBBXMaxX = std::min(int(gridmap.info.width-1), (int(updateBBXMax[0]) - int(paddedMinKey[0]))/int(multires2DScale));
       size_t mapUpdateBBXMaxY = std::min(int(gridmap.info.height-1), (int(updateBBXMax[1]) - int(paddedMinKey[1]))/int(multires2DScale));

       assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
       assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

       size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

       // test for max idx:
       uint max_idx = gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
       if (max_idx  >= gridmap.data.size())
         ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, gridmap.data.size(), gridmap.info.width, gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

       // reset proj. 2D map in bounding box:
       for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
          std::fill_n(gridmap.data.begin() + gridmap.info.width*j+mapUpdateBBXMinX,
                      numCols, -1);
        }
    }
  }
}






void TemporalOctomap::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const{
  if (map.info.resolution != oldMapInfo.resolution){
    ROS_ERROR("Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
  int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);

  if (i_off < 0 || j_off < 0
      || oldMapInfo.width  + i_off > map.info.width
      || oldMapInfo.height + j_off > map.info.height)
  {
    ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

  for (int j =0; j < int(oldMapInfo.height); ++j ){
    // copy chunks, row by row:
    fromStart = oldMapData.begin() + j*oldMapInfo.width;
    fromEnd = fromStart + oldMapInfo.width;
    toStart = map.data.begin() + ((j+j_off)*gridmap.info.width + i_off);
    copy(fromStart, fromEnd, toStart);

//    for (int i =0; i < int(oldMapInfo.width); ++i){
//      map.data[m_gridmap.info.width*(j+j_off) +i+i_off] = oldMapData[oldMapInfo.width*j +i];
//    }

  }
}






void TemporalOctomap::handlePostNodeTraversal(const ros::Time& rostime){
  publishProjected2DMap(rostime);
}






void TemporalOctomap::publishProjected2DMap(const ros::Time& rostime) {
  publish2DMap = (latchedTopics || mapPub.getNumSubscribers() > 0);
  if (publish2DMap) {
    gridmap.header.stamp = rostime;
    mapPub.publish(gridmap);
  }
}






void TemporalOctomap::handleOccupiedNode(const OcTreeT::iterator& it){
  if (publish2DMap && projectCompleteMap){
    update2DMap(it, true);
  }
}

void TemporalOctomap::handleFreeNode(const OcTreeT::iterator& it){
  if (publish2DMap && projectCompleteMap){
    update2DMap(it, false);
  }
}

void TemporalOctomap::handleOccupiedNodeInBBX(const OcTreeT::iterator& it){
  if (publish2DMap && !projectCompleteMap){
    update2DMap(it, true);
  }
}

void TemporalOctomap::handleFreeNodeInBBX(const OcTreeT::iterator& it){
  if (publish2DMap && !projectCompleteMap){
    update2DMap(it, false);
  }
}





std_msgs::ColorRGBA TemporalOctomap::getColor(double time){
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






std_msgs::ColorRGBA TemporalOctomap::heightMapColor(double h) {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}






bool TemporalOctomap::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(treeDepth +1);
  ros::Time rostime = ros::Time::now();
  octree->clear();
  // clear 2D map:
  gridmap.data.clear();
  gridmap.info.height = 0.0;
  gridmap.info.width = 0.0;
  gridmap.info.resolution = 0.0;
  gridmap.info.origin.position.x = 0.0;
  gridmap.info.origin.position.y = 0.0;

  ROS_INFO("Cleared octomap");
  publishAll(rostime);  // Note: This will return as the octree is empty

  publishProjected2DMap(rostime);

  for (std::size_t i = 0; i < occupiedNodesVis.markers.size(); ++i){
    occupiedNodesVis.markers[i].header.frame_id = worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  markerPub.publish(occupiedNodesVis);

  visualization_msgs::MarkerArray freeNodesVis;
  freeNodesVis.markers.resize(treeDepth +1);
  for (std::size_t i = 0; i < freeNodesVis.markers.size(); ++i){
    freeNodesVis.markers[i].header.frame_id = worldFrameId;
    freeNodesVis.markers[i].header.stamp = rostime;
    freeNodesVis.markers[i].ns = "map";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  fmarkerPub.publish(freeNodesVis);

  return true;
}






bool TemporalOctomap::clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp){
  point3d min = pointMsgToOctomap(req.min);
  point3d max = pointMsgToOctomap(req.max);

  double thresMin = octree->getClampingThresMin();
  for(OcTreeT::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max),
      end=octree->end_leafs_bbx(); it!= end; ++it){

    it->setLogOdds(octomap::logodds(thresMin));
    //			octree->updateNode(it.getKey(), -6.0f);
  }
  // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
  octree->updateInnerOccupancy();

  publishAll(ros::Time::now());

  return true;
}







}