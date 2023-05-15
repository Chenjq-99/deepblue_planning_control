/**
 * @Author: YunKai Xia
 * @Date:   2022-06-15 16:18:15
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-06-17 22:39:32
 */

#include <iostream>

#include "path_planning_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planning");
  PathPlanningNode planning_node;
  ROS_INFO("Start Demo!");
  if (!planning_node.init()) {
    std::cout << "fail to init planning node" << std::endl;
    return -1;
  }
  ros::spin();
  return 0;
}
