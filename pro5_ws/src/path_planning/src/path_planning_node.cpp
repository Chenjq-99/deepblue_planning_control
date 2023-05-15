/**
 * @Author: YunKai Xia
 * @Date:   2022-06-17 22:17:32
 * @Last Modified by:   Runqi Qiu
 * @Last Modified time: 2022-10-29 22:33:03
 */

#include "path_planning_node.h"

namespace shenlan {

using Vec_f = std::vector<float>;
using Poi_f = std::array<float, 2>;
using Vec_Poi = std::vector<Poi_f>;

PathPlanningNode::PathPlanningNode() : pnh_("~") {}
PathPlanningNode::~PathPlanningNode() {}
bool PathPlanningNode::init() {
  std::string vehicle_odom_topic;
  std::string vehicle_cmd_topic;
  std::string roadmap_path;
  std::string path_vis_topic;
  std::string frame_id;
  std::string imu_topic;
  std::string local_traj_topic = "local_traj";
  double speed_P, speed_I, speed_D, target_speed, vis_frequency;
  pnh_.getParam("vehicle_odom_topic",
                vehicle_odom_topic);  //读取车辆定位的topic名
  pnh_.getParam("vehicle_cmd_topic",
                vehicle_cmd_topic);             //读取车辆控制的topic名
  pnh_.getParam("roadmap_path", roadmap_path);  //读取路网文件名
  pnh_.getParam("map_path", map_path_);
  pnh_.getParam("path_vis_topic", path_vis_topic);  //读取可视化路网名
  pnh_.getParam("target_speed", target_speed);      //读取目标速度
  pnh_.getParam("goal_tolerance", goalTolerance_);  //读取目标速度
  pnh_.getParam("speed_P", speed_P);                //读取PID参数
  pnh_.getParam("speed_I", speed_I);
  pnh_.getParam("speed_D", speed_D);
  pnh_.getParam("control_frequency", controlFrequency_);  //读取控制的频率
  pnh_.getParam("vis_frequency", vis_frequency);  //读取路网显示的频率
  pnh_.getParam("frame_id", frame_id);            //读取全局坐标系名
  pnh_.getParam("c_speed", c_speed_);  //读取Frenet规划器，初始目标速度
  pnh_.getParam("c_d", c_d_);  //读取Frenet规划器，初始横向偏差
  pnh_.getParam("c_d_d", c_d_d_);  //读取Frenet规划器，初始横向速度偏差
  pnh_.getParam("c_d_dd", c_d_dd_);  //读取Frenet规划器，初始横向加速度偏差
  pnh_.getParam("s0", s0_);  //读取Frenet规划器，初始纵向距离
  pnh_.getParam("imu_topic", imu_topic);

  frame_id_ = frame_id;

  //加载路网文件
  if (!loadRoadmap(roadmap_path, target_speed)) return false;
  
  GetWayPoints();  //在参考路径的基础上进行踩点

  speedPidControllerPtr_ = std::shared_ptr<PIDController>(
      new PIDController(speed_P, speed_I, speed_D));
  
  lqrController_ = std::shared_ptr<LqrController>(new LqrController());
  lqrController_->LoadControlConf();
  lqrController_->Init();

  // 可视化
  visualization_ = VisualizationPtr(new Visualization(nh_));

  // 加载参考线
  if (use_reference_line_) {
    LoadReferenceLine();
  }

  // 构建相对平滑的Frenet曲线坐标系，一个中间暂时方案
  csp_obj_ = new Spline2D(wx_, wy_);

  // 全局路径可视化
  PlotGlobalPath();

  //  Update Obstacle
  // 添加虚拟障碍物
  UpdateStaticObstacle();

  roadmapMarkerPtr_ =
      std::shared_ptr<RosVizTools>(new RosVizTools(nh_, path_vis_topic));

  localTrajectoryMarkerPtr_ =
      std::shared_ptr<RosVizTools>(new RosVizTools(nh_, local_traj_topic));

  VehiclePoseSub_ = nh_.subscribe(vehicle_odom_topic, 10,
                                  &PathPlanningNode::odomCallback, this);
  
  ImuSub_ = nh_.subscribe(imu_topic, 10, &PathPlanningNode::IMUCallback, this);

  // controlPub_ =
  //     nh_.advertise<lgsvl_msgs::VehicleControlData>(vehicle_cmd_topic, 1000);

  controlPub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>(vehicle_cmd_topic, 1000);

  visTimer_ = nh_.createTimer(ros::Duration(1 / vis_frequency),
                              &PathPlanningNode::visTimerLoop,
                              this);  //注册可视化线程

  plannerTimer_ = nh_.createTimer(ros::Duration(1 / plannerFrequency_),
                                  &PathPlanningNode::plannerTimerLoop,
                                  this);  //这侧规划线程

  controlTimer_ = nh_.createTimer(ros::Duration(1 / controlFrequency_),
                                  &PathPlanningNode::controlTimerLoop,
                                  this);  //这侧控制线程

  addRoadmapMarker(planningPublishedTrajectory_.trajectory_points, frame_id);

  goalPoint_ =
      planningPublishedTrajectory_.trajectory_points.back();  //确定目标点

  ROS_INFO("planner node and lqr_control_node init finish!");
  return true;
}

double PathPlanningNode::pid_control() {
  double ego_speed = vehicleState_.velocity;
  // 位置误差
  double v_err = targetSpeed_ - ego_speed;  // 速度误差
  cout << "v_err: " << v_err << "targetSpeed_ is " << targetSpeed_ << endl;
  double acceleration_cmd =
      speedPidControllerPtr_->Control(v_err, 1 / controlFrequency_);
  return acceleration_cmd;
};

void PathPlanningNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  cout << "position.x: " << msg->pose.pose.position.x << " "
       << "position.y: " << msg->pose.pose.position.y << endl;
  if (firstRecord_) {
    vehicleState_.planning_init_x = msg->pose.pose.position.x;
    vehicleState_.planning_init_y = msg->pose.pose.position.y;
    firstRecord_ = false;
  }
  vehicleState_.x = msg->pose.pose.position.x;  //
  vehicleState_.y = msg->pose.pose.position.y;

  // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw)
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(vehicleState_.roll, vehicleState_.pitch,
                          vehicleState_.yaw);

  vehicleState_.heading = vehicleState_.yaw;  // pose.orientation是四元数
  // cout << "vehicle_state_.heading: " << vehicle_state_.heading << endl;

  vehicleState_.velocity =  // 速度
      std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                msg->twist.twist.linear.y * msg->twist.twist.linear.y);
}

void PathPlanningNode::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  vehicleState_.angular_velocity =
      msg->angular_velocity.z;  // 平面角速度(绕z轴转动的角速度)
  vehicleState_.acceleration =
      sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x +
           msg->linear_acceleration.y * msg->linear_acceleration.y);  // 加速度
};

bool PathPlanningNode::loadRoadmap(const std::string &roadmap_path,
                                   const double target_speed) {
  // 读取参考线路径
  std::ifstream infile;
  infile.open(roadmap_path);  //将文件流对象与文件连接起来
  if (!infile.is_open()) {
    return false;
  }
  std::vector<std::pair<double, double>> xy_points;
  std::string s, x, y;
  while (getline(infile, s)) {
    std::stringstream word(s);
    word >> x;
    word >> y;
    double pt_x = std::atof(x.c_str());
    double pt_y = std::atof(y.c_str());
    xy_points.push_back(std::make_pair(pt_x, pt_y));
  }
  infile.close();
  // Construct the reference_line path profile
  using namespace shenlan::control;
  std::vector<double> headings, accumulated_s, kappas, dkappas;
  //根据离散的点组成的路径，生成路网航向角,累计距离，曲率，曲率的导数
  std::unique_ptr<ReferenceLine> reference_line =
      std::make_unique<ReferenceLine>(xy_points);
  reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas,
                                     &dkappas);

  ROS_INFO_STREAM("traj size= " << headings.size());

  for (size_t i = 0; i < headings.size(); i++) {
    TrajectoryPoint trajectory_pt;
    trajectory_pt.x = xy_points[i].first;
    trajectory_pt.y = xy_points[i].second;
    trajectory_pt.heading = headings[i];
    trajectory_pt.kappa = kappas[i];
    planningPublishedTrajectory_.trajectory_points.push_back(trajectory_pt);
  }
  return true;
}

void PathPlanningNode::LoadReferenceLine() {
  // 加载参考线
  std::shared_ptr<zjlmap::Map> map_data_ =
      std::make_shared<zjlmap::Map>();  // 构建map地图指针
  int handle = 0;
  zjlmap::ErrorCode ec = map_data_->load(map_path_.c_str(),
                                         handle);  // 加载地图文件
  ROS_INFO("I heard: [%d]", ec);

  zjlmap::LaneId lane_id(
      0, 0, 0);  // 结构体 {road_id, section_idx, local_id} 描述定位某一段lane
  zjlmap::LaneInfo lane_info = map_data_->query_lane_info(lane_id);
  ReferenceLineInfo reference_line;

  map_data_->calc_lane_center_line_curv(lane_id, lane_info.begin, lane_info.end,
                                        0.25, reference_line);

  for (size_t i = 0; i < reference_line.size(); i++) {
    ROS_INFO("reference pt %d,  x: %f, y: %f", i, reference_line[i].x,
             reference_line[i].y);
  }

  // For Plot
  reference_line_ = reference_line;

  // std::ofstream target_line_cout(
  //     "local-planner-ws/src/path_planning/data/CubeTown.txt");
  // target_line_cout.setf(std::ios::fixed, std::ios::floatfield);
  // target_line_cout.precision(2);
  // for (size_t i = 0; i < reference_line.size(); i++) {
  //   target_line_cout << reference_line[i].y << "  " << reference_line[i].x
  //                    << std::endl;
  // }
  // target_line_cout.close();
}

void PathPlanningNode::addRoadmapMarker(
    const std::vector<TrajectoryPoint> &path, const std::string &frame_id) {
  roadmapMarkerPtr_->clear();
  std::string ns = "reference_path";
  visualization_msgs::Marker marker_linestrip = RosVizTools::newLineStrip(
      0.5, ns, 0, ros_viz_tools::LIGHT_BLUE, frame_id);
  for (auto path_point : path) {
    geometry_msgs::Point p;
    p.x = path_point.x;
    p.y = path_point.y;
    p.z = 0;
    marker_linestrip.points.push_back(p);
  }
  std::cout << "path size is " << marker_linestrip.points.size() << std::endl;
  roadmapMarkerPtr_->append(marker_linestrip);
  return;
}

void PathPlanningNode::addLocalTrajectoryMarker(
    const std::vector<TrajectoryPoint> &path, const std::string &frame_id) {
  localTrajectoryMarkerPtr_->clear();
  std::string ns = "local_trajectory";
  visualization_msgs::Marker marker_linestrip = RosVizTools::newLineStrip(
      0.05, ns, 0, ros_viz_tools::LIGHT_BLUE, frame_id);
  for (auto path_point : path) {
    geometry_msgs::Point p;
    p.x = path_point.x;
    p.y = path_point.y;
    p.z = 0;
    marker_linestrip.points.push_back(p);
  }
  std::cout << "trajectory size is " << marker_linestrip.points.size()
            << std::endl;
  localTrajectoryMarkerPtr_->append(marker_linestrip);
  return;
}

void PathPlanningNode::visTimerLoop(const ros::TimerEvent &) {
  // std::cout << "publish path vis " << std::endl;
  roadmapMarkerPtr_->publish();
  visualization_->publishReferenceLine(reference_line_);
  visualization_->publishGlobalPlan(global_plan_);

  localTrajectoryMarkerPtr_->publish();

  for (const auto &obs : obstcle_list_) {
    // if (pointDistance(obs, vehicleState_.x, vehicleState_.y) < 50.0) {
    //   visualization_->publishObstacles(obstcle_list_);
    // }
    visualization_->publishObstacles(obstcle_list_);
  }
}

void PathPlanningNode::plannerTimerLoop(const ros::TimerEvent &) {
  if (!firstRecord_ && !isReachGoal_) {  //有定位数据开始规划
    // TODO:这里之后可以再被优化，采用更好的Frenet坐标系取点方式。
    const double ego_s = GetNearestReferenceLength(vehicleState_);
    const double ego_l = GetNearestReferenceLatDist(vehicleState_);
    const double ego_speed = vehicleState_.velocity;

    s0_ = ego_s;
    if (std::abs(ego_speed) > 1e-3) {
      c_speed_ = ego_speed;
    }
    c_d_ = ego_l;

    // Idea:
    // 判断是否是终点,这里之后需要优化一下，加一个精准停车功能，然后缩小误差范围，发送Stop命令
    if (std::abs(s0_ - end_s_) < 2.0) {
      // break;
      isReachGoal_ = true;
      ROS_INFO("Goal Reached!");
    }

    FrenetOptimalTrajectory frenet_optimal_trajectory;
    // to-do step 1 finish frenet_optimal_planning
    FrenetPath final_path = frenet_optimal_trajectory.frenet_optimal_planning(
        *csp_obj_, s0_, c_speed_, c_d_, c_d_d_, c_d_dd_, obstcle_list_);

    if (!final_path.s.empty() && !near_goal_) {
      s0_ = final_path.s[1];
      c_d_ = final_path.d[1];
      c_d_d_ = final_path.d_d[1];
      c_d_dd_ = final_path.d_dd[1];
      c_speed_ = final_path.s_d[1];

      // 可视化
      visualization_->publishLocalPlan(final_path);
      const auto trajectory = GetTrajectoryFormFrenetPath(final_path);
      planningPublishedTrajectoryDebug_ = trajectory;
      last_trajectory_ = trajectory;

      if (std::abs(final_path.s.back() - end_s_) < 2.0) {
        ROS_INFO("Near Goal");
        near_goal_ = true;
      }
    } else {
      // Backup
      planningPublishedTrajectoryDebug_ = last_trajectory_;
    }

    // addLocalTrajectoryMarker(
    //     planningPublishedTrajectoryDebug_.trajectory_points, frame_id_);

    plannerFlag_ = true;
  }
}

void PathPlanningNode::controlTimerLoop(const ros::TimerEvent &) {
  ControlCmd cmd;
  if (plannerFlag_) {  //有定位数据开始控制

    //小于容忍距离，车辆速度设置为0
    if (pointDistance(goalPoint_, vehicleState_.x, vehicleState_.y) <
        goalTolerance_) {
      targetSpeed_ = 0;
      ROS_INFO("Goal Readched");
      isReachGoal_ = true;
    }
    if (!isReachGoal_) {
      lqrController_->ComputeControlCommand(
          vehicleState_, planningPublishedTrajectoryDebug_, cmd);
    }

    // lgsvl_msgs::VehicleControlData control_cmd;
    // control_cmd.header.stamp = ros::Time::now();
    // double acc_cmd = pid_control();
    // control_cmd.acceleration_pct = acc_cmd;
    // control_cmd.target_gear = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
    // control_cmd.target_wheel_angle = cmd.steer_target;
    // controlPub_.publish(control_cmd);

    carla_msgs::CarlaEgoVehicleControl control_cmd;
    control_cmd.header.stamp = ros::Time::now();
    control_cmd.reverse = false;
    control_cmd.manual_gear_shift = false;
    control_cmd.hand_brake = false;
    control_cmd.gear = 0;

    // Longitudinal Control
    double acc_cmd = pid_control();

    if (acc_cmd >= 0) {
      control_cmd.throttle = min(1.0, acc_cmd);
      control_cmd.brake = 0.0;
    } else {
      control_cmd.throttle = 0.0;
      control_cmd.brake = min(1.0, -acc_cmd);
    }

    if (targetSpeed_ == 0) {
      control_cmd.throttle = 0.0;
    }

    // Lateral Control
    control_cmd.steer = cmd.steer_target;

    controlPub_.publish(control_cmd);
  }
}

TrajectoryData PathPlanningNode::GetTrajectoryFormFrenetPath(
    const FrenetPath &path) {
  TrajectoryData trajectory;
  const int traj_size = path.t.size();
  trajectory.trajectory_points.reserve(traj_size);

  for (int i = 0; i < traj_size; i++) {
    TrajectoryPoint trajectory_pt;
    trajectory_pt.x = path.x[i];
    trajectory_pt.y = path.y[i];
    trajectory_pt.v = path.ds[i];
    trajectory_pt.a = 0.0;
    trajectory_pt.heading = path.yaw[i];
    trajectory_pt.kappa = path.c[i];
    trajectory.trajectory_points.push_back(trajectory_pt);
  }
  return trajectory;
}

Spline2D PathPlanningNode::CreateRefrenceSpiline() {
  Vec_f wx({0.0, 10.0, 20.5, 35.0, 70.5});
  Vec_f wy({0.0, -6.0, 5.0, 6.5, 0.0});

  return Spline2D(wx_, wy_);
}

void PathPlanningNode::UpdateStaticObstacle() {
  std::vector<Poi_f> obstcles{{255, -195.1}, {175, -199.1}};
  obstcle_list_ = obstcles;
}

void PathPlanningNode::GetWayPoints() {
  const int refline_size =
      planningPublishedTrajectory_.trajectory_points.size();

  const auto &trajectory_pt = planningPublishedTrajectory_.trajectory_points;

  double sum_s = 0;
  wx_.push_back(trajectory_pt[0].x);
  wy_.push_back(trajectory_pt[0].y);

  for (int i = 1; i < refline_size; i++) {
    const double dx = trajectory_pt[i].x - trajectory_pt[i - 1].x;
    const double dy = trajectory_pt[i].y - trajectory_pt[i - 1].y;
    const double s = std::sqrt(dx * dx + dy * dy);
    sum_s += s;

    // 每隔2米的距离进行踩点
    if (sum_s > 2.0) {
      wx_.push_back(trajectory_pt[i].x);
      wy_.push_back(trajectory_pt[i].y);
      ROS_INFO_STREAM("waypt, x= " << wx_.back() << ", y= " << wy_.back());
      sum_s = 0;
    }
  }

  ROS_INFO_STREAM("refline_size= " << refline_size
                                   << ", waypoint size= " << wx_.size());
}

void PathPlanningNode::PlotGlobalPath() {
  Vec_f r_x;
  Vec_f r_y;
  Vec_f ryaw;
  Vec_f rcurvature;
  Vec_f rs;

  global_plan_.poses.clear();
  global_plan_.header.frame_id = frame_id_;
  global_plan_.header.stamp = ros::Time::now();
  // 0.1米的间隔进行踩点
  for (float i = 0; i < csp_obj_->s.back(); i += 0.1) {
    std::array<float, 2> point_ = csp_obj_->calc_postion(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    ryaw.push_back(csp_obj_->calc_yaw(i));
    rcurvature.push_back(csp_obj_->calc_curvature(i));
    rs.push_back(i);

    geometry_msgs::PoseStamped pt;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = frame_id_;
    pt.pose.position.x = point_[0];
    pt.pose.position.y = point_[1];
    pt.pose.position.z = i;  //使用position.z存储路径的s
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(csp_obj_->calc_yaw(i));
    global_plan_.poses.push_back(pt);
  }

  end_x_ = r_x.back();
  end_y_ = r_y.back();
  end_s_ = rs.back();
  ROS_INFO_STREAM("s_end= " << end_s_);
}

int PathPlanningNode::GetNearestReferenceIndex(const VehicleState &ego_state) {
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double distance =
        DistanceXY(ego_state, global_plan_.poses[i].pose.position);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return min_index;
}

double PathPlanningNode::GetNearestReferenceLength(
    const VehicleState &ego_state) {
  return global_plan_.poses[GetNearestReferenceIndex(ego_state)]
      .pose.position.z;  // s存在position.z中
}

double PathPlanningNode::GetNearestReferenceLatDist(
    const VehicleState &ego_state) {
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;

  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i) {
    const double distance =
        DistanceXY(ego_state, global_plan_.poses[i].pose.position);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  const int sign = LeftOfLine(ego_state, global_plan_.poses[min_index],
                              global_plan_.poses[min_index + 1])
                       ? 1
                       : -1;
  return sign * min_dist;
}

bool PathPlanningNode::LeftOfLine(const VehicleState &p,
                                  const geometry_msgs::PoseStamped &p1,
                                  const geometry_msgs::PoseStamped &p2) {
  const double tmpx = (p1.pose.position.x - p2.pose.position.x) /
                          (p1.pose.position.y - p2.pose.position.y) *
                          (p.y - p2.pose.position.y) +
                      p2.pose.position.x;

  if (tmpx >
      p.x)  //当tmpx>p.x的时候，说明点在线的左边，小于在右边，等于则在线上。
    return true;
  return false;
}

}  // namespace shenlan