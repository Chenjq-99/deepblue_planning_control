/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-07-10 22:45:59
 */
#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <base_local_planner/goal_functions.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/publisher.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <iterator>
#include "frenet_path.h"
#include "map.h"

using namespace zjlmap;
namespace shenlan {

class Visualization {
 public:
  Visualization();

  Visualization(ros::NodeHandle& nh);

  void initialize(ros::NodeHandle& nh);

  void publishGlobalPlan(
      const std::vector<geometry_msgs::PoseStamped>& global_plan) const;

  void publishGlobalPlan(const nav_msgs::Path& global_plan);

  void publishLocalPlan(
      const std::vector<geometry_msgs::PoseStamped>& local_plan) const;

  void publishLocalPlan(const FrenetPath& final_path);

  void publishLocalPlan(const nav_msgs::Path& final_path);

  void publishLocalTrajectroy(const FrenetPath& final_path);

  void publishReferenceLine(const ReferenceLineInfo& ref_line);

  void publishObstacles(const std::vector<Poi_f>& obstacles);

  static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

 protected:
  bool printErrorWhenNotInitialized();
  ros::Publisher ref_line_pub_;
  ros::Publisher global_plan_pub_;
  ros::Publisher local_plan_pub_;
  ros::Publisher local_trajectory_pub_;
  ros::Publisher obstacles_marker_pub_;
  bool initialized_;
};

typedef boost::shared_ptr<Visualization> VisualizationPtr;
typedef boost::shared_ptr<const Visualization> VisualizationConstPtr;

}  // namespace shenlan

#endif /* VISUALIZATION_H_ */
