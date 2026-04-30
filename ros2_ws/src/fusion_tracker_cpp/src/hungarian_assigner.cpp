#include "fusion_tracker_cpp/hungarian_assigner.hpp"

#include <limits>
#include <set>

namespace fusion_tracker_cpp {

std::vector<std::pair<std::size_t, std::size_t>> HungarianAssigner::assign(
    const std::vector<std::vector<double>> &cost_matrix, double max_cost) const {
  std::vector<std::pair<std::size_t, std::size_t>> matches;
  std::set<std::size_t> used_cols;

  for (std::size_t r = 0; r < cost_matrix.size(); ++r) {
    double best = std::numeric_limits<double>::max();
    std::size_t best_col = static_cast<std::size_t>(-1);
    for (std::size_t c = 0; c < cost_matrix[r].size(); ++c) {
      if (used_cols.count(c) > 0) {
        continue;
      }
      if (cost_matrix[r][c] < best) {
        best = cost_matrix[r][c];
        best_col = c;
      }
    }
    if (best_col != static_cast<std::size_t>(-1) && best <= max_cost) {
      used_cols.insert(best_col);
      matches.emplace_back(r, best_col);
    }
  }
  return matches;
}

}  // namespace fusion_tracker_cpp
