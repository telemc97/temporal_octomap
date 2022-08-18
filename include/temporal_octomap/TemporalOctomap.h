//ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/PointCloud2.h>

//PCL includes
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

//Octomap includes
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeKey.h>

namespace temporal_octomap {

class TemporalOctomap{

public:
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTree OcTreeT;

  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  TemporalOctomap(const ros::NodeHandle &nh_ = ros::NodeHandle()); //constructor
  virtual ~TemporalOctomap(); //deconstructor
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
  bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);


  void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

protected:

  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;

  inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
    return (    oldMapInfo.height != newMapInfo.height
                || oldMapInfo.width != newMapInfo.width
                || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }

  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };
    
  void insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& pcl);

  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  void publishProjected2DMap(const ros::Time& rostime);

  void publishAll(const ros::Time& rostime);
  
  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode(const OcTreeT::iterator& it) {};

  /// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX(const OcTreeT::iterator& it) {};

  /// hook that is called when traversing occupied nodes of the updated Octree
  virtual void handleOccupiedNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing occupied nodes in the updated area (updates 2D map projection here)
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes of the updated Octree
  virtual void handleFreeNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes in the updated area (updates 2D map projection here)
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);
  
  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  std_msgs::ColorRGBA getColor(double time);

  inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (maxTreeDepth - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0] + voxelWidth >= updateBBXMin[0]
            && key[1] + voxelWidth >= updateBBXMin[1]
            && key[0] <= updateBBXMax[0]
            && key[1] <= updateBBXMax[1]);
  }


  inline unsigned mapIdx(int i, int j) const {
    return gridmap.info.width * j + i;
    
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
    return mapIdx((key[0] - paddedMinKey[0]) / multires2DScale,
                  (key[1] - paddedMinKey[1]) / multires2DScale);

  }


  OcTreeT* octree;
  octomap::KeyRay keyray;
  octomap::OcTreeKey updateBBXMin;
  octomap::OcTreeKey updateBBXMax;
  
  ros::NodeHandle nodeHandle;
  ros::Publisher mapPub, markerPub, fmarkerPub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* PCLSub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* tfPCLSub;
  ros::ServiceServer clearBBXService, resetService;

  tf::TransformListener tfListener;
  std::string worldFrameId;
  std::string baseFrameId;
  nav_msgs::OccupancyGrid gridmap;
  octomap::OcTreeKey paddedMinKey;

  static std_msgs::ColorRGBA heightMapColor(double h);


  unsigned treeDepth;
  unsigned maxTreeDepth;
  unsigned multires2DScale;

  double colorFactor;
  double res;
  double pointcloudMinX;
  double pointcloudMaxX;
  double pointcloudMinY;
  double pointcloudMaxY;
  double pointcloudMinZ;
  double pointcloudMaxZ;
  double occupancyMinZ;
  double occupancyMaxZ;
  double minSizeX;
  double minSizeY;
  double maxRange;
  double minRange;
  ros::Duration decaytime;
  bool publish2DMap;

  bool incrementalUpdate;
  bool projectCompleteMap;
  bool latchedTopics;
  bool filterSpeckles;
  bool useHeightMap;
  bool publishFreeSpace;

  std_msgs::ColorRGBA colorFree;
  std_msgs::ColorRGBA color;


};
}