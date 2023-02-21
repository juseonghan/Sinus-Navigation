#ifndef ROS_IGTL_POLARIS_H
#define ROS_IGTL_POLARIS_H
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

// Messages Includes
#include "ros_igtl_bridge/igtltransform.h"

// C++ Includes
#include <cstdlib>
#include <cstring>
#include <string>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdint.h>

// VTK header files
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class ROS_IGTL_Polaris
{
public:
  ROS_IGTL_Polaris(int argc, char *argv[], const char* node_name);
  ~ROS_IGTL_Polaris();
  void Run();
  
private:
  ros::NodeHandle *nh;
  ros::Publisher transform_pub;
  ros::Subscriber sub_polaris; 
  
  virtual void polarisCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
        
};
#endif 
