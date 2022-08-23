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
#include <pcl/point_cloud.h>
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
#include <octomap/OcTreeStamped.h>


namespace temporal_octomap {

class TemporalOctomap{

public:
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTreeStamped OcTreeT;

  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  TemporalOctomap(const ros::NodeHandle &nh_ = ros::NodeHandle()); //constructor
  virtual ~TemporalOctomap(); //deconstructor
  // bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
  // bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);


  virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  void checkNodes(const ros::TimerEvent& event);

  // ros::Time getDecayTime() {return decaytime;}

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

  static inline void pointcloudPCLToOctomap(const PCLPointCloud& pclCloud, octomap::Pointcloud& octomapCloud){
    octomapCloud.reserve(pclCloud.points.size());

    PCLPointCloud::const_iterator it;
    for (it = pclCloud.begin(); it != pclCloud.end(); ++it){
      // Check if the point is invalid
      if (!isnan (it->x) && !isnan (it->y) && !isnan (it->z))
        octomapCloud.push_back(it->x, it->y, it->z);
    }
  }
  
  void insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& pcl);

  void publishAll(const ros::Time& rostime);

  std_msgs::ColorRGBA getColor(int time);

  int getTimeLeft(const OcTreeT::iterator& it, const ros::Time& rostime);



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
  ros::Timer updateInterval;
  ros::ServiceServer clearBBXService, resetService;

  tf::TransformListener tfListener;
  std::string worldFrameId;
  std::string baseFrameId;
  nav_msgs::OccupancyGrid gridmap;
  octomap::OcTreeKey paddedMinKey;

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
  ros::Time decaytime;
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