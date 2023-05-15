/**
 * @Author: YunKai Xia
 * @Date:   2022-06-15 16:18:15
 * @Last Modified by:   Runqi Qiu
 * @Last Modified time: 2022-10-08 22:42:02
 */

#include <iostream>

#include "lqr_controller_node.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lqr_control");
  LQRControllerNode control_node;
  if (!control_node.init()) {
    std::cout << "fail to init lqr_control_node" << std::endl;
    return -1;
  }
  ros::spin();
  return 0;
}
