#pragma once

#include <cstddef>
#include <vector>

namespace fusion_tracker_cpp {

class HungarianAssigner {
public:
  // Minimum-cost perfect matching on a square cost matrix (rows == cols).
  // Returns col_assignment[i] = column index matched to row i.
  std::vector<std::size_t> assignSquare(const std::vector<std::vector<double>> &cost_matrix) const;
};

}  // namespace fusion_tracker_cpp
