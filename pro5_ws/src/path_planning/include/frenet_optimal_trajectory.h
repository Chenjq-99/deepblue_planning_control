/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-07-18 21:01:48
 */

// ***Description***:
// Many thanks to the author of the Frenet algorithm here, this paper may be
// very helpful to you, "Optimal Trajectory Generation for Dynamic Street
// Scenarios in a Frenet Frame"
// https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
// Thanks to open source codes, python robotics, this website can help you
// quickly verify some algorithms, which is very useful for beginners.
// https://github.com/AtsushiSakai/PythonRobotics

#pragma once

#include <sys/time.h>

#include <iostream>
#include <limits>
#include <vector>

#include "cubic_spline.hpp"
#include "frenet_path.h"
#include "quartic_polynomial.h"
#include "quintic_polynomial.h"
using namespace std;

namespace shenlan {
// namespace planner {
class FrenetOptimalTrajectory {
 public:
  FrenetOptimalTrajectory();
  ~FrenetOptimalTrajectory();
  /**
   * @brief
   *
   * @param value_list
   * @return float
   */
  float sum_of_power(std::vector<float> value_list);
  /**
   * @brief
   *
   * @param c_speed
   * @param c_d
   * @param c_d_d
   * @param c_d_dd
   * @param s0
   * @return Vec_Path
   */
  Vec_Path calc_frenet_paths(float c_speed, float c_d, float c_d_d,
                             float c_d_dd, float s0);
  /**
   * @brief
   *
   * @param path_list
   * @param csp
   */
  void calc_global_paths(Vec_Path& path_list, Spline2D csp);
  /**
   * @brief
   *
   * @param path
   * @param ob
   * @return true
   * @return false
   */
  bool check_collision(FrenetPath path, const Vec_Poi ob);
  Vec_Path check_paths(Vec_Path path_list, const Vec_Poi ob);
  /**
   * @brief
   *
   * @param csp
   * @param s0
   * @param c_speed
   * @param c_d
   * @param c_d_d
   * @param c_d_dd
   * @param ob
   * @return FrenetPath
   */
  FrenetPath frenet_optimal_planning(Spline2D csp, float s0, float c_speed,
                                     float c_d, float c_d_d, float c_d_dd,
                                     Vec_Poi ob);
  /**
   * @brief
   *
   * @param csp
   * @param frenet_init_conditions
   * @param ob
   * @return FrenetPath
   */
  FrenetPath frenet_optimal_planning(
      Spline2D csp, const FrenetInitialConditions& frenet_init_conditions,
      Vec_Poi ob);
};
}  // namespace shenlan
