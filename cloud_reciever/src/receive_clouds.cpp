#include <sensor_msgs/Imu.h>

#include<std_msgs/Float32.h>
#include<std_msgs/String.h>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

//ledger imports
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <map_database/map_database.h>

#include<ros/ros.h>

double accumulation_time;
std::string target_frame;
int point_r, point_g, point_b;



// variables
pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
ros::Time accumulation_start_time;
md::MapDatabase* map_database;

// publishers
ros::Publisher accumulated_cloud_pub;
ros::Publisher map_pub;

// subscribers
ros::Subscriber cloud_sub;
tf::TransformListener* listener;

// callbacks
void cloudCallback(sensor_msgs::PointCloud2 cloud);
void submapCallback(md::MapID map_id);


bool cloud_receiver_initialize(ros::NodeHandle &nh){
  // init params
  ROS_WARN(" L 53");
//   ros::NodeHandle* nh ;
//   ros::NodeHandle* pnh ;

  accumulation_time = 2.0;
  target_frame = std::string("map");
  
  point_r =0;
  point_g =  0;
  point_b = 0;

  // init variables
  accumulation_start_time = ros::Time::now();
  
  // ----------------------------------- MAP DATABASE EXAMPLE ------------------------------------------
  map_database = md::MapDatabase::getMapDatabase();
  map_database->registerCallback(&submapCallback);
  map_database->addRequestPolicy(new md::GreedyRequestPolicy(md::MapDataType::RawPointCloud));
  map_database->addRequestPolicy(new md::GreedyRequestPolicy(md::MapDataType::TrueDetections));
  
  // init publishers
  accumulated_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("accumulated_cloud", 1);
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("map", 1);

  ROS_WARN(" L 77");
  return true;
}

void publishMap(){
    ROS_WARN("L 82");
  sensor_msgs::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(*map_cloud, map_cloud_msg);
  map_cloud_msg.header.stamp = ros::Time::now();
  map_cloud_msg.header.frame_id = target_frame;
  map_pub.publish(map_cloud_msg);
}

void submapCallback(md::MapID map_id){
    ROS_WARN(" L 91");
  // ----------------------------------- MAP DATABASE EXAMPLE ------------------------------------------
  // Check that the submap isn't from this robot. It would have already been added to the map in that case.
//   ROS_WARN(map_id.getCreator());
//   ROS_WARN(map_id.getMapDataType());
  if(map_id.getMapDataType() == md::MapDataType::RawPointCloud)
  {
    ROS_WARN("data type correct");
  }

  if(map_id.getCreator() == md::EntityInfo::getEntityID())
  {
    ROS_WARN("Creator Id correct");
  }



    ROS_WARN("%d",map_id.getCreator() == md::EntityInfo::getEntityID());
    ROS_WARN("%d",map_id.getMapDataType() == md::MapDataType::RawPointCloud);

  if((map_id.getCreator() == md::EntityInfo::getEntityID()) && (map_id.getMapDataType() == md::MapDataType::RawPointCloud)){
    ROS_WARN(" L 95");
    // Look up the submap based on the map_id.
    md::SubMap submap;
    bool submap_valid = map_database->getSubMap(map_id, submap);
    // Make sure the submap look up was successful.
    if(submap_valid){
        ROS_WARN("submap is valid");
      // Extract the data from the submap.
      std::vector<uint8_t> data = submap.getData();
      ROS_WARN("L 121");
      // Use the data by adding it to the map.
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      ROS_WARN("L 124");
      submap_cloud->points.resize(data.size()/sizeof(pcl::PointXYZI));
      ROS_WARN("L 126");
      memcpy(&submap_cloud->points[0], &data[0], data.size());
      ROS_WARN(" L 128");
      *map_cloud += *submap_cloud;
      ROS_WARN("L 130");
      publishMap();
      ROS_WARN(" L 131");

    }
    else{
      ROS_ERROR_STREAM("INVALID SUBMAP in submapCallback");
    }
  }
  else if(map_id.getCreator() != md::EntityInfo::getEntityID() && map_id.getMapDataType() == md::MapDataType::TrueDetections){
    ROS_WARN("got artifacts");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

//   ros::NodeHandle n;

  ros::NodeHandle nh ;
//   ros::NodeHandle* pnh ;

//   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  
  cloud_receiver_initialize(nh);

  

  ros::spin();

  return 0;
}
