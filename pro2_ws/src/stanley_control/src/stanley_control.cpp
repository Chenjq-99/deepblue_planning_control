#include "stanley_control.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include <math.h>

using namespace std;

namespace shenlan {
namespace control {

double atan2_to_PI(const double atan2) {
  return atan2 * M_PI / 180;
}

double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.x - x;
  const double dy = point.y - y;
  return dx * dx + dy * dy;
}

void StanleyController::LoadControlConf() {
  k_y_ = 0.5;
}

// /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
// 提示，在该函数中你需要调用计算误差 ComputeLateralErrors
void StanleyController::ComputeControlCmd(
    const VehicleState &vehicle_state,
    const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {
    
    // 把planning_published_trajectory copy进StanleyController里的trajectory_points_
    trajectory_points_ = planning_published_trajectory.trajectory_points;

    // 获取车辆状态x, y, heading, vx
    double vehicle_x = vehicle_state.x;
    double vehicle_y = vehicle_state.y;
    double vehicle_theta = vehicle_state.heading;
    double vehicle_vel = vehicle_state.velocity + 0.001; 

    //计算误差
    double e_y, e_theta;
    
    ComputeLateralErrors(vehicle_x, vehicle_y, vehicle_theta, e_y, e_theta);
    
    // 计算输出
    double steer_output = e_theta + atan2(k_y_ * e_y, vehicle_vel);

    // 限幅
    if(steer_output > M_PI/3) 
    {
        steer_output=  M_PI / 3 ;
    }
    else if (steer_output < -M_PI/3)
    {
        steer_output =  -M_PI / 3;
    }
    cmd.steer_target = steer_output;
    
}

// /** to-do **/ 计算需要的误差，包括横向误差，heading误差
void StanleyController::ComputeLateralErrors(const double vehicle_x, const double vehicle_y,
                                             const double vehicle_theta, double &e_y,
                                             double &e_theta) {
    // 找到trajectory上离当前位置最近的点
    TrajectoryPoint target_point = QueryNearestPointByPosition(vehicle_x, vehicle_y);
    
    double dx = target_point.x - vehicle_x;
    double dy = target_point.y - vehicle_y;
    double target_point_heading = target_point.heading;
    double delta = target_point_heading - atan2(dy, dx);
    e_y = sin(delta) * sqrt(PointDistanceSquare(target_point, vehicle_x, vehicle_y));
    e_theta = vehicle_theta - target_point_heading;
    if(e_theta > M_PI)
    {
        e_theta =e_theta - 2 * M_PI;
    }
    else if (e_theta < -M_PI)
    {
        
        e_theta = e_theta + 2 * M_PI;
    }
}

TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x,
                                                               const double y) {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  // cout << " index_min: " << index_min << endl;
  //cout << "tarjectory.heading: " << trajectory_points_[index_min].heading << endl;
  theta_ref_ = trajectory_points_[index_min].heading;

  return trajectory_points_[index_min];
}

}  // namespace control
}  // namespace shenlan
