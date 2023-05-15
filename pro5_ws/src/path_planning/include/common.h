/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   Runqi Qiu
 * @Last Modified time: 2022-10-29 22:33:03
 */
//#include <lgsvl_msgs/VehicleControlData.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

#include <fstream>
#include <iostream>
#include <string>

#include "map.h"
#include "reference_line.h"
#include "ros/ros.h"
#include "tf/tf.h"

struct VehicleState {
  double x;
  double y;
  double heading;           // 车辆朝向
  double kappa;             // 曲率(切线斜率)
  double velocity;          // 速度
  double angular_velocity;  // 角速度
  double acceleration;      // 加速度

  // 规划起点
  double planning_init_x;
  double planning_init_y;

  double roll;
  double pitch;
  double yaw;

  double target_curv;  // 期望点的曲率

  double vx;
  double vy;

  // added
  double start_point_x;
  double start_point_y;

  double relative_x = 0;
  double relative_y = 0;

  double relative_distance = 0;
};

struct TrajectoryPoint {
  double x;
  double y;
  double heading;
  double kappa;
  double v;
  double a;
};

// 轨迹
struct TrajectoryData {
  std::vector<TrajectoryPoint> trajectory_points;
};

struct LateralControlError {
  double lateral_error;       // 横向误差
  double heading_error;       // 转向误差
  double lateral_error_rate;  // 横向误差速率
  double heading_error_rate;  // 转向误差速度
};

struct ControlCmd {
  double steer_target;
  double acc;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

struct tPoint {
  double t;
};
typedef std::vector<double> T_Points;

struct sPoint {
  double s;
  double s_d;
  double s_dd;
  double s_ddd;
  double cd = 0.0;
  double cv = 0.0;
  double cf = 0.0;
};
typedef std::vector<sPoint> S_Points;

struct dPoint {
  double d;
  double d_d;
  double d_dd;
  double d_ddd;
};
typedef std::vector<dPoint> D_Points;

struct xPoint {
  double x;
  double y;
  double yaw;
  double ds;
  double c;
};
typedef std::vector<xPoint> X_Points;

typedef std::shared_ptr<LateralControlError> LateralControlErrorPtr;
