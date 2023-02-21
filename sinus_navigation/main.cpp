#include <ros_igtl_polaris.h>

int main (int argc, char *argv[])
{
  ROS_IGTL_Polaris polaris_node(argc, argv, "ros_igtl_polaris");
  polaris_node.Run();

  return 0;
}
