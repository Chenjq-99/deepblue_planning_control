/**
 * @Author: YunKai Xia
 * @Date:   2022-06-15 16:18:15
 * @Last Modified by:   Runqi Qiu
 * @Last Modified time: 2022-10-8 22:42:02
 */
#include <math.h>

#include <iostream>
#include <vector>

namespace shenlan {
namespace control {
class ReferenceLine {
 public:
  ReferenceLine(const std::vector<std::pair<double, double>>& xy_points);
  ~ReferenceLine() = default;

  bool ComputePathProfile(std::vector<double>* headings,
                          std::vector<double>* accumulated_s,
                          std::vector<double>* kappas,
                          std::vector<double>* dkappas);

 private:
  std::vector<std::pair<double, double>> xy_points_;
};

}  // namespace control
}  // namespace shenlan