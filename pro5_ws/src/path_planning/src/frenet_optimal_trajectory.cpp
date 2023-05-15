/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   Runqi Qiu
 * @Last Modified time: 2022-10-29 22:33:03
 */

// ***Description***:
// Many thanks to the author of the Frenet algorithm here, this paper may be
// very helpful to you, "Optimal Trajectory Generation for Dynamic Street
// Scenarios in a Frenet Frame"
// https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
// Thanks to open source codes, python robotics, this website can help you
// quickly verify some algorithms, which is very useful for beginners.
// https://github.com/AtsushiSakai/PythonRobotics

#include "frenet_optimal_trajectory.h"
#include "ros/ros.h"

namespace shenlan
{
#define MAX_SPEED 50.0 / 3.6    // maximum speed [m/s]
#define MAX_ACCEL 6.0           // maximum acceleration [m/ss]
#define MAX_CURVATURE 100.0     // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0      // maximum road width [m]
#define D_ROAD_W 1.0            // road width sampling length [m]
#define DT 0.2                  // time tick [s]
#define MAXT 3.0                // max prediction time [m]
#define MINT 2.0                // min prediction time [m]
#define TARGET_SPEED 17.0 / 3.6 // target speed [m/s]
#define D_T_S 5.0 / 3.6         // target speed sampling length [m/s]
#define N_S_SAMPLE 1            // sampling number of target speed
#define ROBOT_RADIUS 2.5        // robot radius [m]

#define KJ 1.5
#define KT 0.1
#define KD 1.4
#define KLAT 1.0
#define KLON 1.0

    FrenetOptimalTrajectory::FrenetOptimalTrajectory()
    {
    }
    FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {}

    float FrenetOptimalTrajectory::sum_of_power(std::vector<float> value_list)
    {
        float sum = 0;
        for (float item : value_list)
        {
            sum += item * item;
        }
        return sum;
    };

    // 01 获取采样轨迹
    Vec_Path FrenetOptimalTrajectory::calc_frenet_paths(float c_speed, float c_d,
                                                        float c_d_d, float c_d_dd,
                                                        float s0)
    {
        std::vector<FrenetPath> fp_list;

        // 对横向位移 d 进行采样
        for (float di = 0; di < 5; di += D_ROAD_W)
        {
            // 对纵向时间序列采样
            for (float Ti = MINT; Ti < MAXT; Ti += DT)
            {
                // 当 (di,Ti) 确定后，可获得一条连接当前状态与 (di, Ti) 的五次多项式轨迹曲线
                FrenetPath fp;
                QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);

                // 记录离散时间下对应的轨迹点
                for (float t = 0; t < Ti; t += DT)
                {
                    fp.t.push_back(t);
                    fp.d.push_back(lat_qp.calc_point(t));
                    fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                    fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
                    fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
                }

                // 对纵向车速进行采样
                for (float tv = TARGET_SPEED - D_T_S * N_S_SAMPLE;
                     tv < TARGET_SPEED + D_T_S * N_S_SAMPLE; tv += D_T_S)
                {
                    // 当 (vi, Ti) 确定后，可获得一条连接当前状态和 (vi, Ti) 的四次多项式轨迹曲线
                    FrenetPath fp_bot = fp;
                    QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

                    // 初始化最大速度和最大加速度
                    fp_bot.max_speed = std::numeric_limits<float>::min();
                    fp_bot.max_accel = std::numeric_limits<float>::min();

                    // 记录离散时间下对应的轨迹点
                    for (float t_ : fp.t)
                    {
                        fp_bot.s.push_back(lon_qp.calc_point(t_));
                        fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
                        fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
                        fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));

                        // 更新最大加速度和最大速度
                        if (fp_bot.s_d.back() > fp_bot.max_speed)
                        {
                            fp_bot.max_speed = fp_bot.s_d.back();
                        }

                        if (fp_bot.s_dd.back() > fp_bot.max_accel)
                        {
                            fp_bot.max_accel = fp_bot.s_dd.back();
                        }
                    }

                    // 计算代价函数
                    float Jp = sum_of_power(fp.d_ddd);     // square of jerk
                    float Js = sum_of_power(fp_bot.s_ddd); // square of jerk
                    // square of diff from target speed
                    float ds = (TARGET_SPEED - fp_bot.s_d.back());

                    fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
                    fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
                    fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

                    // 将轨迹添加至候选轨迹中
                    fp_list.push_back(fp_bot);
                }
            }
        }
        return fp_list;
    };

    // 02
    // 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
    void FrenetOptimalTrajectory::calc_global_paths(Vec_Path &path_list,
                                                    Spline2D csp)
    {
        // 记录采样轨迹的其他参数

        // 遍历 fp_list 中所有的轨迹
        for (Vec_Path::iterator path_p = path_list.begin(); path_p != path_list.end();
             path_p++)
        {
            // 若轨迹比参考路径长，则及时截断
            for (unsigned int i = 0; i < path_p->s.size(); i++)
            {
                if (path_p->s[i] >= csp.s.back())
                {
                    break;
                }

                // 将 frenet 轨迹点还原至 cartesian 坐标系中
                std::array<float, 2> poi = csp.calc_postion(path_p->s[i]);

                float iyaw = csp.calc_yaw(path_p->s[i]);
                float di = path_p->d[i];

                float x = poi[0] + di * std::cos(iyaw + M_PI / 2.0);
                float y = poi[1] + di * std::sin(iyaw + M_PI / 2.0);

                path_p->x.push_back(x);
                path_p->y.push_back(y);
            }

            // 在 cartesian 坐标系中计算轨迹的航向角和车速
            for (unsigned int i = 0; i < path_p->x.size() - 1; i++)
            {
                float dx = path_p->x[i + 1] - path_p->x[i];
                float dy = path_p->y[i + 1] - path_p->y[i];
                path_p->yaw.push_back(std::atan2(dy, dx));
                path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
            }

            // 补全缺失的航向角和车速
            path_p->yaw.push_back(path_p->yaw.back());
            path_p->ds.push_back(path_p->ds.back());

            // 计算轨迹去率并更新最大曲率
            path_p->max_curvature = std::numeric_limits<float>::min();
            for (unsigned int i = 0; i < path_p->x.size() - 1; i++)
            {
                path_p->c.push_back((path_p->yaw[i + 1] - path_p->yaw[i]) /
                                    path_p->ds[i]);
                if (path_p->c.back() > path_p->max_curvature)
                {
                    path_p->max_curvature = path_p->c.back();
                }
            }
        }
    };

    bool FrenetOptimalTrajectory::check_collision(FrenetPath path,
                                                  const Vec_Poi ob)
    {
        for (auto point : ob)
        {
            for (unsigned int i = 0; i < path.x.size(); i++)
            {
                float dist = std::pow((path.x[i] - point[0]), 2) +
                             std::pow((path.y[i] - point[1]), 2);

                if (dist <= ROBOT_RADIUS * ROBOT_RADIUS)
                {
                    return false;
                }
            }
        }
        return true;
    };

    // 03
    // 检查路径，通过限制做大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
    Vec_Path FrenetOptimalTrajectory::check_paths(Vec_Path path_list,
                                                  const Vec_Poi ob)
    {
        Vec_Path output_fp_list;

        // 若路径满足速度、加速度、曲率约束且无碰撞，则为可行解
        for (FrenetPath path : path_list)
        {
            if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL &&
                path.max_curvature < MAX_CURVATURE && check_collision(path, ob))
            {
                output_fp_list.push_back(path);
            }
        }
        return output_fp_list;
    };

    // to-do step 1 finish frenet_optimal_planning
    FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
        Spline2D csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd,
        Vec_Poi ob)
    {
        // 01
        // 获取采样轨迹数组
        Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);

        // 02
        // 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
        calc_global_paths(fp_list, csp);

        // 03
        // 检查路径，通过限制做大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
        Vec_Path fesible_fp_list = check_paths(fp_list, ob);

        // 04
        // 根据cost选出最优轨迹
        float min_cost = std::numeric_limits<float>::max();
        FrenetPath final_path;
        for (auto path : fesible_fp_list)
        {
            if (min_cost >= path.cf)
            {
                min_cost = path.cf;
                final_path = path;
            }
        }
        return final_path;
    };

    FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
        Spline2D csp, const FrenetInitialConditions &frenet_init_conditions,
        Vec_Poi ob)
    {
        float c_speed = frenet_init_conditions.c_speed;
        float c_d = frenet_init_conditions.c_d;
        float c_d_d = frenet_init_conditions.c_d_d;
        float c_d_dd = frenet_init_conditions.c_d_dd;
        float s0 = frenet_init_conditions.s0;

        Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
        calc_global_paths(fp_list, csp);
        Vec_Path save_paths = check_paths(fp_list, ob);

        float min_cost = std::numeric_limits<float>::max();
        FrenetPath final_path;
        for (auto path : save_paths)
        {
            if (min_cost >= path.cf)
            {
                min_cost = path.cf;
                final_path = path;
            }
        }
        return final_path;
    }

} // namespace shenlan
