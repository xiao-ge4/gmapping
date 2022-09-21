#include <ros/ros.h>

#include "slam_gmapping.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_gmapping"); //初始化 ros 节点

  SlamGMapping gn; //声明一个 SlamGMapping 类
  gn.startLiveSlam(); //调用方法
  ros::spin(); //

  return(0);
}

