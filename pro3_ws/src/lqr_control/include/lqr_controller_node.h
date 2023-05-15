/**
 * @Author: YunKai Xia
 * @Date:   2022-06-17 22:12:09
 * @Last Modified by:   Runqi Qiu
 * @Last Modified time: 2022-10-8 23:43:25
 */
#include <stdint.h>

#ifndef __LQR_CONTROLLER_NODE_H__
#define __LQR_CONTROLLER_NODE_H__

#include <memory>

#include "lqr_controller.h"
#include "pid_controller.h"
#include "ros_viz_tools/ros_viz_tools.h"
using namespace shenlan::control;
using namespace ros_viz_tools;
using namespace std;
class LQRControllerNode {
 public:
  LQRControllerNode();
  ~LQRControllerNode();
  bool init();

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  //控制线程
  void controlTimerLoop(const ros::TimerEvent &);
  double pid_control();
  // 可视化路径的线程函数
  void visTimerLoop(const ros::TimerEvent &);
  //加载路网地图，并设置轨迹的速度信息
  bool loadRoadmap(const std::string &roadmap_path, const double target_speed);
  //将路网转换成可视化的marker数据
  void addRoadmapMarker(const std::vector<TrajectoryPoint> &path,
                        const std::string &frame_id);
  //计算两点之间的距离
  double pointDistance(const TrajectoryPoint &point, const double x,
                       const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return sqrt(dx * dx + dy * dy);
  }
  double pointDistance(const double x1, const double y1, const double x,
                       const double y) {
    double dx = x1 - x;
    double dy = y1 - y;
    return sqrt(dx * dx + dy * dy);
  }

 private:
  ros::NodeHandle nh_;                             //句柄
  ros::NodeHandle pnh_;                            //读取配置参数句柄
  ros::Timer visTimer_;                            //可视化的线程
  ros::Timer controlTimer_;                        //控制的线程
  ros::Subscriber VehiclePoseSub_;                 //订阅车辆的定位信息
  ros::Publisher controlPub_;                      //发布控制指令
  std::shared_ptr<RosVizTools> roadmapMarkerPtr_;  //发布可视化路网
  VehicleState vehicleState_;

  double targetSpeed_ = 5;
  std::shared_ptr<PIDController> speedPidControllerPtr_;
  std::shared_ptr<LqrController> lqrController_;
  double controlFrequency_ = 100;               //控制频率
  TrajectoryData planningPublishedTrajectory_;  //跟踪的轨迹
  TrajectoryPoint goalPoint_;                   //终点
  double goalTolerance_ = 0.5;                  //到终点的容忍距离
  bool isReachGoal_ = false;
  bool firstRecord_ = true;
};
#endif /* __LQR_CONTROLLER_NODE_H__ */
