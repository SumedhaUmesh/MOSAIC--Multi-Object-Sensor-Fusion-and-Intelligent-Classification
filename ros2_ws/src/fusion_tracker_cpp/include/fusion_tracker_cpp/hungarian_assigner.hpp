#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace fusion_tracker_cpp {

class HungarianAssigner {
public:
  std::vector<std::pair<std::size_t, std::size_t>>
  assign(const std::vector<std::vector<double>> &cost_matrix, double max_cost) const;
};

}  // namespace fusion_tracker_cpp
