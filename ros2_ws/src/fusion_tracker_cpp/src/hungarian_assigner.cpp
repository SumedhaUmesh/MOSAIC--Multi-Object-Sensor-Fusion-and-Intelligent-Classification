#include "fusion_tracker_cpp/hungarian_assigner.hpp"

#include <limits>
#include <stdexcept>
#include <vector>

namespace fusion_tracker_cpp {

std::vector<std::size_t> HungarianAssigner::assignSquare(const std::vector<std::vector<double>> &a) const {
  const int n = static_cast<int>(a.size());
  if (n == 0) {
    return {};
  }
  for (const auto &row : a) {
    if (row.size() != static_cast<std::size_t>(n)) {
      throw std::invalid_argument("HungarianAssigner::assignSquare requires a square matrix");
    }
  }

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(static_cast<std::size_t>(n) + 1, 0.0);
  std::vector<double> v(static_cast<std::size_t>(n) + 1, 0.0);
  std::vector<int> p(static_cast<std::size_t>(n) + 1, 0);
  std::vector<int> way(static_cast<std::size_t>(n) + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(static_cast<std::size_t>(n) + 1, inf);
    std::vector<char> used(static_cast<std::size_t>(n) + 1, 0);
    do {
      used[static_cast<std::size_t>(j0)] = 1;
      int i0 = p[static_cast<std::size_t>(j0)];
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (!used[static_cast<std::size_t>(j)]) {
          double cur = a[static_cast<std::size_t>(i0 - 1)][static_cast<std::size_t>(j - 1)] -
                       u[static_cast<std::size_t>(i0)] - v[static_cast<std::size_t>(j)];
          if (cur < minv[static_cast<std::size_t>(j)]) {
            minv[static_cast<std::size_t>(j)] = cur;
            way[static_cast<std::size_t>(j)] = j0;
          }
          if (minv[static_cast<std::size_t>(j)] < delta) {
            delta = minv[static_cast<std::size_t>(j)];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[static_cast<std::size_t>(j)]) {
          u[static_cast<std::size_t>(p[static_cast<std::size_t>(j)])] += delta;
          v[static_cast<std::size_t>(j)] -= delta;
        } else {
          minv[static_cast<std::size_t>(j)] -= delta;
        }
      }
      j0 = j1;
    } while (p[static_cast<std::size_t>(j0)] != 0);
    do {
      int j1 = way[static_cast<std::size_t>(j0)];
      p[static_cast<std::size_t>(j0)] = p[static_cast<std::size_t>(j1)];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> col_for_row_1based(static_cast<std::size_t>(n) + 1, 0);
  for (int j = 1; j <= n; ++j) {
    col_for_row_1based[static_cast<std::size_t>(p[static_cast<std::size_t>(j)])] = j;
  }

  std::vector<std::size_t> out(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    out[static_cast<std::size_t>(i)] =
        static_cast<std::size_t>(col_for_row_1based[static_cast<std::size_t>(i + 1)] - 1);
  }
  return out;
}

}  // namespace fusion_tracker_cpp
