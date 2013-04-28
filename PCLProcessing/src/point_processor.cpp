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


pcl::ModelCoefficients::Ptr planes[10];
unsigned int text_id = 0;
int num_planes = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
std::vector<double> distances;


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}



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

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}


void makeVisualizer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::ModelCoefficients::Ptr coefficients)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(cloud);
  viewer->addPlane (*coefficients, "plane");
  viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "plane");
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}


void findPlaneCallback(const std_msgs::Empty::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eric/groovy_workspace/Olin_AR_Drone/ARDronePTAM/data/master.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file master_pcd.pcd \n");
    return;
  }

  float sum;
  for (int i = 0; i < 10; i++)
  {
    if (i <= num_planes)
    {
      break;
    }
    for (int point = 0; point < cloud->points.size(); point++)
    {
      sum = ((float) cloud->points[0])*((float) (planes[i]->values[0])) + ((float) cloud->points[1])*((float) (planes[i]->values[1])) + ((float) cloud->points[2])*((float) (planes[i]->values[2])) + ((float) (planes[i]->values[3]));
    }
  }


  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eric/groovy_workspace/Olin_AR_Drone/ARDronePTAM/data/outliers.pcd", *outliers) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file outliers.pcd \n");
  }

  *cloud += *outliers;

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
  seg.setDistanceThreshold (0.1);


  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  while (cloud->points.size () > 100)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud->makeShared());
    seg.segment (*inliers, *coefficients);
  
    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      break;
    }

    num_planes++;
    if (num_planes < 10)
    {
      planes[num_planes-1] = coefficients;
    }
    
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);
  }

  pcl::PCDWriter writer;
  
  if ( writer.write("/home/eric/groovy_workspace/Olin_AR_Drone/ARDronePTAM/data/outliers.pcd", *cloud) == -1)
  {
    PCL_ERROR("Could not write to file.");
    return;
  }

  makeVisualizer(cloud, coefficients);

}


// ----------------------------------------------------------------------
// ---------------------------------Main---------------------------------
// ----------------------------------------------------------------------
int
main (int argc, char** argv)
{
  ros::init(argc, argv, "point_processor");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("findPlane", 1, findPlaneCallback);

  ros::spin();

}
