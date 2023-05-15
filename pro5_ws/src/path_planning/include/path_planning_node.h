/**
 * @Author: YunKai Xia
 * @Date:   2022-06-17 22:12:09
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-07-18 21:41:04
 */

#ifndef __LQR_CONTROLLER_NODE_H__
#define __LQR_CONTROLLER_NODE_H__

#include <sensor_msgs/Imu.h>

#include <memory>

#include "frenet_optimal_trajectory.h"
#include "frenet_path.h"
#include "lqr_controller.h"
#include "map.h"
#include "pid_controller.h"
#include "ros_viz_tools/ros_viz_tools.h"
#include "visualization.h"

using namespace shenlan::control;
using namespace shenlan;
using namespace ros_viz_tools;
using namespace std;
using namespace zjlmap;

namespace shenlan {

template <typename U, typename V>
double DistanceXY(const U &u, const V &v) {
  return std::hypot(u.x - v.x, u.y - v.y);
}

class PathPlanningNode {
 public:
  PathPlanningNode();
  ~PathPlanningNode();
  bool init();

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg);
  //规划线程
  void plannerTimerLoop(const ros::TimerEvent &);
  //控制线程
  void controlTimerLoop(const ros::TimerEvent &);
  /**
   * @brief 使用PID对车辆速度进行控制
   *
   * @return double
   */
  double pid_control();
  // 可视化路径的线程函数
  void visTimerLoop(const ros::TimerEvent &);
  //加载路网地图，并设置轨迹的速度信息
  bool loadRoadmap(const std::string &roadmap_path, const double target_speed);
  //将路网转换成可视化的marker数据
  void addRoadmapMarker(const std::vector<TrajectoryPoint> &path,
                        const std::string &frame_id);

  void addLocalTrajectoryMarker(const std::vector<TrajectoryPoint> &path,
                                const std::string &frame_id);
  //计算两点之间的距离
  double pointDistance(const TrajectoryPoint &point, const double x,
                       const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return sqrt(dx * dx + dy * dy);
  }
  double pointDistance(const Poi_f &point, const double x, const double y) {
    double dx = point[0] - x;
    double dy = point[1] - y;
    return sqrt(dx * dx + dy * dy);
  }
  double pointDistance(const double x1, const double y1, const double x,
                       const double y) {
    double dx = x1 - x;
    double dy = y1 - y;
    return sqrt(dx * dx + dy * dy);
  }
  /**
   * @brief Get the Trajectory Form Frenet Path object
   *
   * @param path
   * @return TrajectoryData
   */
  TrajectoryData GetTrajectoryFormFrenetPath(const FrenetPath &path);

  void publishLocalPlan(
      const std::vector<geometry_msgs::PoseStamped> &local_plan) const;

  Spline2D CreateRefrenceSpiline();

  void UpdateStaticObstacle();
  /**
   * @brief Get the Way Points object:根据参考轨迹进行踩点
   *
   */
  void GetWayPoints();

  void PlotGlobalPath();
  /**
   * @brief Get the Nearest Reference Index
   * object：根据车辆的当前位置，获取与参考路劲最近点的id
   *
   * @param ego_state ：当前时刻车辆的位置
   * @return int
   */
  int GetNearestReferenceIndex(const VehicleState &ego_state);

  double GetNearestReferenceLength(const VehicleState &ego_state);
  double GetNearestReferenceLatDist(const VehicleState &ego_state);
  bool LeftOfLine(const VehicleState &p, const geometry_msgs::PoseStamped &p1,
                  const geometry_msgs::PoseStamped &p2);

  void LoadReferenceLine();

 private:
  ros::NodeHandle nh_;                             //句柄
  ros::NodeHandle pnh_;                            //读取配置参数句柄
  ros::Timer visTimer_;                            //可视化的线程
  ros::Timer controlTimer_;                        //控制的线程
  ros::Timer plannerTimer_;                        //规划的线程
  ros::Subscriber VehiclePoseSub_;                 //订阅车辆的定位信息
  ros::Subscriber ImuSub_;                         // 订阅imu信息
  ros::Publisher controlPub_;                      //发布控制指令
  ros::Publisher localPlanPub_;                    // 局部路径
  ros::Publisher localTrajectoryPub_;              // 局部轨迹
  std::shared_ptr<RosVizTools> roadmapMarkerPtr_;  //发布可视化路网
  VehicleState vehicleState_;

  double targetSpeed_ = 5;
  std::shared_ptr<PIDController> speedPidControllerPtr_;
  std::shared_ptr<LqrController> lqrController_;
  std::shared_ptr<LqrController> planner_;
  double controlFrequency_ = 100;  //控制频率
  double plannerFrequency_ = 10;   //规划频率
  TrajectoryData planningNodePublishedTrajectory_;
  TrajectoryData planningPublishedTrajectoryDebug_;  //规划下发的轨迹
  TrajectoryData last_trajectory_;                   //规划下发的轨迹
  TrajectoryData planningPublishedTrajectory_;       //跟踪的轨迹
  TrajectoryPoint goalPoint_;                        //终点
  double goalTolerance_ = 0.5;                       //到终点的容忍距离
  bool isReachGoal_ = false;
  bool firstRecord_ = true;
  bool plannerFlag_ = false;

  VisualizationPtr visualization_;  //!< Instance of the visualization class
  //!< (local/global plan, obstacles, ...)

  std::vector<Poi_f> obstcle_list_;

  FrenetInitialConditions frenet_init_conditions_;

  Vec_f wx_, wy_;

  Spline2D *csp_obj_;

  float c_speed_ = 10.0 / 3.6;
  float c_d_ = 2.0;
  float c_d_d_ = 0.0;
  float c_d_dd_ = 0.0;
  float s0_ = 0.0;

  nav_msgs::Path global_plan_;
  std::string frame_id_;

  std::shared_ptr<RosVizTools> localTrajectoryMarkerPtr_;  //发布全局路径

  ReferenceLineInfo reference_line_;

  std::string map_path_;

  double end_x_, end_y_, end_s_;
  bool near_goal_ = false;
  bool use_reference_line_ = false;
};
}  // namespace shenlan
#endif /* __LQR_CONTROLLER_NODE_H__ */
