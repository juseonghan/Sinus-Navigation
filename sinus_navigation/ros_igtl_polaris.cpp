#include "ros_igtl_polaris.h"
#include "igtlMath.h"

// VTK Includes
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkCubeSource.h>
#include <vtkVertex.h>
#include <vtkPolyLine.h>
#include <vtkTriangleStrip.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
#include <vtkPolyDataReader.h>

#include <igtlTypes.h>

//----------------------------------------------------------------------
ROS_IGTL_Polaris::ROS_IGTL_Polaris(int argc, char *argv[], const char* node_name)
{
  ros::init(argc, argv, node_name);
  nh = new ros::NodeHandle;	
  	
  // declare publisher 
  transform_pub = nh->advertise<ros_igtl_bridge::igtltransform>("IGTL_TRANSFORM_OUT", 10);

  // declare subscriber
  sub_polaris = nh->subscribe("/NDI/Endoscope/measured_cp", 10, &ROS_IGTL_Polaris::polarisCallback, this); 
}

//----------------------------------------------------------------------
ROS_IGTL_Polaris::~ROS_IGTL_Polaris()
{
	 
}

//----------------------------------------------------------------------
void ROS_IGTL_Polaris::Run()
{
  ros::spin();
}

//---callbackcs for subscriber------------------------------------------
//----------------------------------------------------------------------

void ROS_IGTL_Polaris::polarisCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

//   ROS_INFO("[ROS_IGTL_Test] Transform %s received: \n", msg->name.c_str()); 

  // read from the transformation that came and publish on transformpub
  ros_igtl_bridge::igtltransform transform_msg;

  // the values that we get from polaris are in meters, but the CT scan is in millimeters so we should multiply by
  // 1000 in order to scale them properly 
  float tx, ty, tz, qx, qy, qz, qw; 
  tx = 1000. * msg->pose.position.x; 
  ty = 1000. * msg->pose.position.y; 
  tz = 1000. * msg->pose.position.z; 
  qx = 1000. * msg->pose.orientation.x; 
  qy = 1000. * msg->pose.orientation.y; 
  qz = 1000. * msg->pose.orientation.z; 
  qw = 1000. * msg->pose.orientation.w; 
  transform_msg.transform.translation.x = tx;
  transform_msg.transform.translation.y = ty;
  transform_msg.transform.translation.z = tz;
  transform_msg.transform.rotation.x = qx;
  transform_msg.transform.rotation.y = qy;
  transform_msg.transform.rotation.z = qz;
  transform_msg.transform.rotation.w = qw;
  transform_msg.name = "ROS_IGTL_POSE"; 

//   std::cout << "Header #" << msg->header.frame_id << std::endl;
    ROS_INFO("[ROS_IGTL_Polaris] Pose Translation: %d %d %d \n", tx, ty, tz); 
    ROS_INFO("[ROS_IGTL_Polaris] Pose Quaternion: %d %d %d %d\n", qx, qy, qz, qw); 
//   std::cout << "translation vec: " << tx << " " << ty << " " << tz << std::endl;
//   std::cout << "quaternion vec: " << qx << " " << qy << " " << qz << " " << qw << std::endl;

  transform_pub.publish(transform_msg); 

  // ros::Duration(0.1).sleep();

}
