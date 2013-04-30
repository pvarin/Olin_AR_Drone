#include <stdio.h>
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "std_msgs/Empty.h"
#include <Eigen/Core>
// PCL Header Files
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


std::vector<pcl::ModelCoefficients::Ptr> planes;
unsigned int text_id = 0;
bool firstCloud = true;
std::vector<double> distances;

pcl::PointCloud<pcl::PointXYZ>::Ptr outliercloud (new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

void addPlanetoViewer(pcl::ModelCoefficients::Ptr coefficients)
{
  char buff[7];
  sprintf(buff, "plane%d", planes.size());
  viewer->addPlane(*coefficients, buff);
  viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, buff); 
}

void addCloudtoViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer->initCameraParameters ();
}

void updateCloudViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // std::cerr << "Update cloud size: " << cloud->points.size() << endl;
  viewer->updatePointCloud<pcl::PointXYZ> (cloud, "cloud");
}

void findPlaneModels()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  //////////////////Set plane segmentation parameters//////////////////
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.2);

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  while (outliercloud->points.size () >= 20)
  { 
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (outliercloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () < 20)
    {//throw away junk planes
      PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
      std::cerr << "The plane that it found contained " << inliers->indices.size() << " points" << std::endl;
      break;
    }

    if (planes.size() != 0)
    {//if there are pre-existing planes
      int temp = planes.size();
      for (int i = 0; i < temp; i++)
      {
        if ( (fabs(planes[i]->values[0]*coefficients->values[0] + planes[i]->values[1]*coefficients->values[1] + planes[i]->values[2]*coefficients->values[2]) < 0.05) && (fabs(planes[i]->values[3] - coefficients->values[3]) > 0.05))
        {
          std::cerr << "Numplanes: " << planes.size() << " DotProduct: " << fabs(planes[i]->values[0]*coefficients->values[0] + planes[i]->values[1]*coefficients->values[1] + planes[i]->values[2]*coefficients->values[2]) << " Plane Distance: "<< fabs(planes[i]->values[3] - coefficients->values[3]) <<std::endl;
          planes.push_back(coefficients);
          addPlanetoViewer(coefficients);
        }
      }
    }
    else
    {
      planes.push_back(coefficients);
      addPlanetoViewer(coefficients);
    }

    //extract the outliers and put them in outliercloud
    extract.setInputCloud (outliercloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_p);
    outliercloud.swap (cloud_p);
  }
  // std::cerr << "Done with findPlaneModels" << std::endl;
  return;
}
  


// void findPlaneCallback(const std_msgs::Empty::ConstPtr& msg)
void findPlaneCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (input->data.size() == 0)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);
  std::cerr << "Recieved " << cloud->size() << " point(s) from the ROS message." << std::endl;
  if (planes.size() == 0)
  {
    *outliercloud += *cloud;
    std::cerr << "Size of outliercloud before findPlaneModels(): " << outliercloud->points.size() << std::endl;
    findPlaneModels();
    std::cerr << "Size of outliercloud after findPlaneModels(): " << outliercloud->points.size() << std::endl;
  }
  else
  {
    float distance; 
    pcl::PointIndices::Ptr outlierindices (new pcl::PointIndices());

    //Check each point to see if it fits a plane
    for (int point = 0; (point < cloud->points.size()) && (planes.size() > 0) ; point++)
    {
      bool isOutlier = true;
      for (int i = 0; i < planes.size(); i++)
      {
        distance = fabs(( cloud->points[point].x)*( (planes[i]->values[0])) + ( cloud->points[point].y)*( (planes[i]->values[1])) + ( cloud->points[point].z)*( (planes[i]->values[2])) + ( (planes[i]->values[3])));
        if (distance < 0.01)
        {
          isOutlier = false;
          break;//the point fits a plane so discard it
        }
      }
      if (isOutlier){
        outlierindices->indices.push_back(point);
      }
        
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (outlierindices);
    extract.setNegative (false);
    extract.filter (*outliers);

    *outliercloud += *outliers;
    std::cerr << "OutlierIndicesSize: " << outlierindices->indices.size() << " OutlierCloudSize: " << outliercloud->points.size() << " OutliersSize: " << outliers->points.size() << std::endl;
    findPlaneModels();
    std::cerr << "Size of outliercloud after findPlaneModels(): " << outliercloud->points.size() << std::endl;
  }

  if (firstCloud)
  {
    firstCloud = false;
    addCloudtoViewer(outliercloud);
  }
  else
  {
    updateCloudViewer(outliercloud);
  }

}

void timerCallback(const ros::TimerEvent& e)
{
  viewer->spinOnce (49);
  //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}


// ----------------------------------------------------------------------
// ---------------------------------Main---------------------------------
// ----------------------------------------------------------------------
int
main (int argc, char** argv)
{
  ros::init(argc, argv, "point_processor");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("vslam/pc2", 1, findPlaneCallback);

  ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
  
  ros::spin();

}
