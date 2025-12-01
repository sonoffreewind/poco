#include <cassert>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>
#include "conflict.hpp"

namespace
{

  // Internal sub-procedures, hidden from the header file for better
  // encapsulation.
  ConflictRelation getConflictByTwoMoveDir(Point s_i, Point g_i, Point s_j,
                                           Point g_j);
  ConflictRelation getConflictByThreeMoveDir(Point s_i, Point g_i, Point s_j,
                                             Point g_j);
  ConflictRelation getConflictByFourMoveDir(Point s_i, Point g_i, Point s_j,
                                            Point g_j);

  // Sign function (returns -1, 0, or 1)
  int sign(int val)
  {
    if (val < 0) return -1;
    if (val > 0) return 1;
    return 0;
  }

  // Manhattan distance between two points
  int manhattanDistance(const Point& p1, const Point& p2)
  {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
  }

  // Coordinate transformation helper: rotates a point counter-clockwise
  void rotate270degree(Point& p)
  {
    int t = p.x;
    p.x = p.y;
    p.y = -t;
  }
  void rotate180degree(Point& p)
  {
    p.x = -p.x;
    p.y = -p.y;
  }
  void rotate90degree(Point& p)
  {
    int t = p.x;
    p.x = -p.y;
    p.y = t;
  }

}  // end anonymous namespace

// --- Public Function Definitions ---

ConflictRelation getConflictRelation(const Point& start_i, const Point& goal_i,
                                     const Point& start_j, const Point& goal_j)
{
  // --- Initial Trivial Checks ---
  if (start_i == start_j || goal_i == goal_j) {
    return ConflictRelation::Hostile;
  }

  BoundingBox D_i = getBoundingBox(start_i, goal_i);
  BoundingBox D_j = getBoundingBox(start_j, goal_j);
  if (!isBoundingBoxesOverlap(D_i, D_j)) {
    return ConflictRelation::Free;
  }

  // --- Calculate Directional Vectors and B ---
  int b_ix = sign(goal_i.x - start_i.x);
  int b_iy = sign(goal_i.y - start_i.y);
  int b_jx = sign(goal_j.x - start_j.x);
  int b_jy = sign(goal_j.y - start_j.y);

  // B is the total number of non-zero directional components.
  int B = std::abs(b_ix) + std::abs(b_iy) + std::abs(b_jx) + std::abs(b_jy);

  // Dispatch to the appropriate sub-procedure based on B.
  if (B == 0)
    return ConflictRelation::Free;
  else if (B == 1)
    return ConflictRelation::Hostile;
  else if (B == 2)
    return getConflictByTwoMoveDir(start_i, goal_i, start_j, goal_j);
  else if (B == 3)
    return getConflictByThreeMoveDir(start_i, goal_i, start_j, goal_j);
  else   // (B == 4)
    return getConflictByFourMoveDir(start_i, goal_i, start_j, goal_j);
}

namespace
{  // All procedures are internal implementation details

  /**
   * @brief Logic for Case B=2, where both agents move in 1D.
   */
  ConflictRelation getConflictByTwoMoveDir(Point s_i, Point g_i, Point s_j,
                                           Point g_j)
  {
    int b_ix = sign(g_i.x - s_i.x);
    int b_iy = sign(g_i.y - s_i.y);

    // Normalize agent i's movement to the positive x-axis
    if (b_iy == 1) {  // up -> right
      rotate270degree(s_i);
      rotate270degree(g_i);
      rotate270degree(s_j);
      rotate270degree(g_j);
    } else if (b_ix == -1) {  // left -> right
      rotate180degree(s_i);
      rotate180degree(g_i);
      rotate180degree(s_j);
      rotate180degree(g_j);
    } else if (b_iy == -1) {  // down -> right
      rotate90degree(s_i);
      rotate90degree(g_i);
      rotate90degree(s_j);
      rotate90degree(g_j);
    }

    int b_jx = sign(g_j.x - s_j.x);
    int b_jy = sign(g_j.y - s_j.y);
    int T_i = manhattanDistance(s_i, g_i);
    int T_j = manhattanDistance(s_j, g_j);

    if (b_jx == 1 && b_jy == 0) {  // Case 1.1: Parallel
      if (T_i == T_j) return ConflictRelation::Free;
      if (T_i > T_j) {
        return (s_i.x + T_j + 1 <= g_j.x && g_j.x <= g_i.x - 1)
                   ? ConflictRelation::Hostile
                   : ConflictRelation::Free;
      } else {  // T_i < T_j
        return (s_j.x + T_i + 1 <= g_i.x && g_i.x <= g_j.x - 1)
                   ? ConflictRelation::Hostile
                   : ConflictRelation::Free;
      }
    } else if (b_jx == -1 && b_jy == 0) {  // Case 1.2: Head-on
      return ConflictRelation::Hostile;
    } else if (b_jx == 0 && b_jy != 0) {  // Case 1.3: Perpendicular
      if (std::abs(s_i.x - s_j.x) == std::abs(s_i.y - s_j.y))
        return ConflictRelation::Hostile;
      if (T_i == T_j) return ConflictRelation::Free;
      if (T_i > T_j) {
        return (g_j.y == s_i.y && g_j.x >= s_i.x + T_j + 1 &&
                g_j.x <= g_i.x - 1)
                   ? ConflictRelation::Hostile
                   : ConflictRelation::Free;
      } else {  // T_i < T_j
        bool hostile_condition = (g_i.x == s_j.x) &&
                                 (b_jy * (s_j.y + T_i + 1) <= b_jy * s_i.y) &&
                                 (b_jy * s_i.y <= b_jy * (s_j.y + T_j - 1));
        return hostile_condition ? ConflictRelation::Hostile
                                 : ConflictRelation::Free;
      }
    }
    return ConflictRelation::Free;
  }

  /**
   * @brief Logic for Case B=3, where one agent moves in 2D and the other in 1D.
   */
  ConflictRelation getConflictByThreeMoveDir(Point s_i, Point g_i, Point s_j,
                                             Point g_j)
  {
    int b_ix = sign(g_i.x - s_i.x);
    int b_iy = sign(g_i.y - s_i.y);

    // Ensure agent i is the one moving in 2D
    if (std::abs(b_ix) + std::abs(b_iy) == 1) {
      std::swap(s_i, s_j);
      std::swap(g_i, g_j);
      b_ix = sign(g_i.x - s_i.x);
      b_iy = sign(g_i.y - s_i.y);
    }

    // Normalize agent i's movement to the first quadrant (+,+)
    if (b_ix == 1 && b_iy == -1) {
      rotate90degree(s_i);
      rotate90degree(g_i);
      rotate90degree(s_j);
      rotate90degree(g_j);
    } else if (b_ix == -1 && b_iy == -1) {
      rotate180degree(s_i);
      rotate180degree(g_i);
      rotate180degree(s_j);
      rotate180degree(g_j);
    } else if (b_ix == -1 && b_iy == 1) {
      rotate270degree(s_i);
      rotate270degree(g_i);
      rotate270degree(s_j);
      rotate270degree(g_j);
    }

    int b_jx = sign(g_j.x - s_j.x);
    int b_jy = sign(g_j.y - s_j.y);

    // For symmetry, reflect across y=x if agent j moves vertically
    if (b_jx == 0 && b_jy != 0) {
      std::swap(s_i.x, s_i.y);
      std::swap(g_i.x, g_i.y);
      std::swap(s_j.x, s_j.y);
      std::swap(g_j.x, g_j.y);
      b_jx = sign(g_j.x - s_j.x);
      b_jy = 0;
    }

    int T_i = manhattanDistance(s_i, g_i);
    int T_j = manhattanDistance(s_j, g_j);
    BoundingBox D_i = getBoundingBox(s_i, g_i);
    BoundingBox D_j = getBoundingBox(s_j, g_j);
    BoundingBox D_ij = getIntersection(D_i, D_j);

    if (b_jx == 1 && b_jy == 0) {            // Case 2.1
      if (s_i.x + s_i.y == s_j.x + s_j.y) {  // Phase I
        bool can_avoid =
            !isInBoundingBox(g_i, D_j) && (s_j.x > s_i.x || g_j.x < g_i.x);
        return can_avoid ? ConflictRelation::Compatible
                         : ConflictRelation::Hostile;
      }
      if (T_i == T_j) return ConflictRelation::Free;  // Phase II
      if (T_i > T_j) {
        bool conflict = isInBoundingBox(g_j, D_i) &&
                        (s_j.x + s_j.y > s_i.x + s_i.y) &&
                        (g_j.x + g_j.y < g_i.x + g_i.y);
        return conflict ? ConflictRelation::Compatible : ConflictRelation::Free;
      } else {  // T_i < T_j
        bool conflict = isInBoundingBox(g_i, D_j) && (g_i.x > s_j.x + T_i) &&
                        (g_i.x < g_j.x);
        return conflict ? ConflictRelation::Hostile : ConflictRelation::Free;
      }
    } else if (b_jx == -1 && b_jy == 0) {  // Case 2.2
      D_ij = getIntersection(D_i, D_j);

      // Check if the set D_ij_sta is empty
      int cx = (s_i.x + s_i.y + s_j.x - s_j.y) / 2;
      if (D_ij.min_x <= D_ij.max_x && D_ij.min_y <= D_ij.max_y &&
          ((s_i.x + s_i.y + s_j.x - s_j.y) & 1) == 0 && D_ij.min_x <= cx &&
          D_ij.max_x >= cx) {
        bool can_locate = (g_i.x == cx && g_i.y == s_j.y);
        return can_locate ? ConflictRelation::Hostile
                          : ConflictRelation::Compatible;
      }
      // Check if the set D_ij_pie is empty
      cx = (s_i.x + s_i.y + s_j.x - s_j.y - 1) / 2;
      if (D_ij.min_x <= D_ij.max_x && D_ij.min_y <= D_ij.max_y &&
          ((s_i.x + s_i.y + s_j.x - s_j.y) & 1) == 1 && D_ij.min_x <= cx &&
          D_ij.max_x >= cx + 1) {
        return ConflictRelation::Compatible;
      }
      if (T_i == T_j) return ConflictRelation::Free;  // Phase II
      if (T_i > T_j) {
        bool conflict = isInBoundingBox(g_j, D_i) &&
                        (2* g_j.x + g_j.y > s_i.x + s_i.y + s_j.x) &&
                        (g_j.x + g_j.y < g_i.x + g_i.y);
        return conflict ? ConflictRelation::Compatible : ConflictRelation::Free;
      } else {  // T_i < T_j
        bool conflict = isInBoundingBox(g_i, D_j) && (g_i.x < s_j.x - T_i) &&
                        (g_i.x > s_j.x - T_j);
        return conflict ? ConflictRelation::Hostile : ConflictRelation::Free;
      }
    }
    // Cases 2.3 and 2.4 are symmetric and handled by coordinate swapping in the
    // main dispatcher.
    return ConflictRelation::Free;
  }

  /**
   * @brief Logic for Case B=4, where both agents move in 2D.
   */
  ConflictRelation getConflictByFourMoveDir(Point s_i, Point g_i, Point s_j,
                                            Point g_j)
  {
    int b_ix = sign(g_i.x - s_i.x);
    int b_iy = sign(g_i.y - s_i.y);

    // Normalize agent i's movement to the first quadrant (+,+)
    if (b_ix == 1 && b_iy == -1) {
      rotate90degree(s_i);
      rotate90degree(g_i);
      rotate90degree(s_j);
      rotate90degree(g_j);
    } else if (b_ix == -1 && b_iy == -1) {
      rotate180degree(s_i);
      rotate180degree(g_i);
      rotate180degree(s_j);
      rotate180degree(g_j);
    } else if (b_ix == -1 && b_iy == 1) {
      rotate270degree(s_i);
      rotate270degree(g_i);
      rotate270degree(s_j);
      rotate270degree(g_j);
    }

    int b_jx = sign(g_j.x - s_j.x);
    int b_jy = sign(g_j.y - s_j.y);

    // For symmetry, reduce Case 3.3 to Case 3.2 via y=x reflection
    if (b_jx == -1 && b_jy == 1) {
      std::swap(s_i.x, s_i.y);
      std::swap(g_i.x, g_i.y);
      std::swap(s_j.x, s_j.y);
      std::swap(g_j.x, g_j.y);
      b_jx = 1;
      b_jy = -1;
    }

    int T_i = manhattanDistance(s_i, g_i);
    int T_j = manhattanDistance(s_j, g_j);
    BoundingBox D_i = getBoundingBox(s_i, g_i);
    BoundingBox D_j = getBoundingBox(s_j, g_j);
    BoundingBox D_ij = getIntersection(D_i, D_j);

    if (b_jx == 1 && b_jy == 1) {            // Case 3.1
      if (s_i.x + s_i.y == s_j.x + s_j.y) {  // Phase I
        bool i_can_avoid = (!isInBoundingBox(g_i, D_j)) &&
                           (s_i.x < s_j.x || g_i.x > g_j.x) &&
                           (s_i.y < s_j.y || g_i.y > g_j.y);
        bool j_can_avoid = (!isInBoundingBox(g_j, D_i)) &&
                           (s_j.x < s_i.x || g_j.x > g_i.x) &&
                           (s_j.y < s_i.y || g_j.y > g_i.y);
        return (i_can_avoid || j_can_avoid) ? ConflictRelation::Compatible
                                            : ConflictRelation::Hostile;
      }
      if (T_i == T_j) return ConflictRelation::Free;
      if (T_i > T_j) {
        return (isInBoundingBox(g_j, D_i) && s_j.x + s_j.y > s_i.x + s_i.y &&
                g_j.x + g_j.y < g_i.x + g_i.y)
                   ? ConflictRelation::Compatible
                   : ConflictRelation::Free;
      } else {
        return (isInBoundingBox(g_i, D_j) && s_i.x + s_i.y > s_j.x + s_j.y &&
                g_i.x + g_i.y < g_j.x + g_j.y)
                   ? ConflictRelation::Compatible
                   : ConflictRelation::Free;
      }
    } else if (b_jx == 1 && b_jy == -1) {  // Case 3.2

      // Check if the set D_ij_sta is empty
      int cy = (s_i.x + s_i.y - s_j.x + s_j.y) / 2;
      if (D_ij.min_x <= D_ij.max_x && D_ij.min_y <= D_ij.max_y &&
          ((s_i.x + s_i.y - s_j.x + s_j.y) & 1) == 0 && D_ij.min_y <= cy &&
          D_ij.max_y >= cy) {
        return ConflictRelation::Compatible;
      }

      // Check if the set D_ij_pie is empty
      cy = (s_i.x + s_i.y - s_j.x + s_j.y - 1) / 2;
      if (D_ij.min_x <= D_ij.max_x && D_ij.min_y <= D_ij.max_y &&
          ((s_i.x + s_i.y - s_j.x + s_j.y) & 1) == 1 && D_ij.min_y <= cy &&
          D_ij.max_y >= cy + 1) {
        return ConflictRelation::Compatible;
      }

      if (T_i == T_j) return ConflictRelation::Free;
      if (T_i > T_j) {
        return (isInBoundingBox(g_j, D_i) && 2*g_j.y > s_i.x + s_i.y - s_j.x + s_j.y &&
                g_j.x + g_j.y < g_i.x + g_i.y)
                   ? ConflictRelation::Compatible
                   : ConflictRelation::Free;
      } else {
        bool can_conflict = isInBoundingBox(g_i, D_j) &&
                            (2 * g_i.y < s_i.x + s_i.y - s_j.x + s_j.y) &&
                            (g_i.x - g_i.y < g_j.x - g_j.y);
        return can_conflict ? ConflictRelation::Compatible
                            : ConflictRelation::Free;
      }
    } else if (b_jx == -1 && b_jy == -1) {  // Case 3.4

      // Check if the set D_ij_sta is empty
      int cxy = (s_j.x + s_j.y + s_i.x + s_i.y) / 2;
      if (D_ij.min_x <= D_ij.max_x && D_ij.min_y <= D_ij.max_y &&
          ((s_j.x + s_j.y + s_i.x + s_i.y) & 1) == 0 &&
          D_ij.min_x + D_ij.min_y <= cxy && D_ij.max_x + D_ij.max_y >= cxy) {
        return ConflictRelation::Compatible;
      }

      // Check if the set D_ij_pie is empty
      cxy = (s_j.x + s_j.y + s_i.x + s_i.y - 1) / 2;
      if (D_ij.min_x <= D_ij.max_x && D_ij.min_y <= D_ij.max_y &&
          ((s_j.x + s_j.y + s_i.x + s_i.y) & 1) == 1 &&
          D_ij.min_x + D_ij.min_y <= cxy &&
          cxy <= D_ij.max_x + D_ij.max_y - 1 &&
          (D_ij.min_y <= D_ij.max_y - 1 || D_ij.min_x <= D_ij.max_x - 1)) {
        return ConflictRelation::Compatible;
      }

      // Generic Phase II logic for all B=4 cases where Phase I is Free
      if (T_i == T_j) return ConflictRelation::Free;
      if (T_i > T_j) {
        return (isInBoundingBox(g_j, D_i) && 2*g_j.x + 2*g_j.y > s_j.x + s_j.y + s_i.x + s_i.y &&
                g_j.x + g_j.y < g_i.x + g_i.y)
                   ? ConflictRelation::Compatible
                   : ConflictRelation::Free;
      } else {
        bool conflict = isInBoundingBox(g_i, D_j) &&
                        (2 * (g_i.x + g_i.y) < s_j.x + s_j.y + s_i.x + s_i.y) &&
                        (g_j.x + g_j.y < g_i.x + g_i.y);
        return conflict ? ConflictRelation::Compatible : ConflictRelation::Free;
      }
    }
    // Case 3.3 is handled by coordinate transformation in the main dispatcher.
    return ConflictRelation::Free;
  }

}  // end anonymous namespace
