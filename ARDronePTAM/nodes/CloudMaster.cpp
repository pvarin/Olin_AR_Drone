#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing

  sensor_msgs::PointCloud2 output;

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "CloudMaster");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("vslam/pc2", 1, cloud_cb);

  // Spin
  ros::spin ();
}