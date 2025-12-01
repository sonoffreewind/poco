#pragma once

#include <algorithm>  // For std::min and std::max
#include <string>
#include <vector>

/**
 * @brief Enum to represent the three conflict relationship types.
 */
enum class ConflictRelation { Free = 0, Compatible = 1, Hostile = 2 };

/**
 * @brief Represents a point/vertex with integer coordinates on the grid.
 */
struct Point {
  int x = 0;
  int y = 0;

  Point() = default;
  Point(int _x, int _y) : x(_x), y(_y) {}

  bool operator==(const Point& other) const{
        return x == other.x && y == other.y;
  }
  bool operator!=(const Point& other) const{
        return !(*this == other);
  }
};

/**
 * @brief Represents an axis-aligned bounding box.
 */
struct BoundingBox {
  int min_x, max_x, min_y, max_y;

  /**
   * @brief Checks if the bounding box represents a valid, non-empty area.
   * @return True if the box is valid, false otherwise.
   */
  bool isValid() const { return min_x <= max_x && min_y <= max_y; }
};

// --- Helper and Public Function Declarations ---

/**
 * @brief Calculates the shortest path bounding box (D_i) for a given agent.
 * @param start The start position for the agent to calculate the bounding box.
 * @param goal The goal position for the agent to calculate the bounding box.
 * @return A BoundingBox struct representing the agent's total travel area.
 */
inline BoundingBox getBoundingBox(const Point& start, const Point& goal)
{
  return {std::min(start.x, goal.x),
          std::max(start.x, goal.x),
          std::min(start.y, goal.y),
          std::max(start.y, goal.y)};
}

/**
 * @brief Checks for overlap between two bounding boxes.
 * @param b1 The first bounding box.
 * @param b2 The second bounding box.
 * @return True if the boxes overlap, false otherwise.
 */
inline bool isBoundingBoxesOverlap(const BoundingBox& b1, const BoundingBox& b2)
{
  return !(b1.max_x < b2.min_x || b1.min_x > b2.max_x || b1.max_y < b2.min_y ||
           b1.min_y > b2.max_y);
}

/**
 * @brief Calculates the intersection of two bounding boxes.
 * @param b1 The first bounding box.
 * @param b2 The second bounding box.
 * @return A BoundingBox representing the intersection. If no overlap, returns
 * an invalid box.
 */
inline BoundingBox getIntersection(const BoundingBox& b1, const BoundingBox& b2)
{
  if (!isBoundingBoxesOverlap(b1, b2)) {
    return {1, 0, 1, 0};  // Return an invalid bounding box
  }
  return {std::max(b1.min_x, b2.min_x), std::min(b1.max_x, b2.max_x),
          std::max(b1.min_y, b2.min_y), std::min(b1.max_y, b2.max_y)};
}

/**
 * @brief Checks if a point is within a given bounding box.
 * @param p The point to check.
 * @param box The bounding box.
 * @return True if the point is inside or on the boundary of the box, false
 * otherwise.
 */
inline bool isInBoundingBox(const Point& p, const BoundingBox& box)
{
  return (p.x >= box.min_x && p.x <= box.max_x && p.y >= box.min_y &&
          p.y <= box.max_y);
}

// --- Public Function Declarations ---

/**
 * @brief Determines the conflict relationship between two agents over all their
 * shortest paths.
 * @details This is the main dispatcher function that corresponds to Algorithm 1
 * in the manuscript. It handles trivial cases and then normalizes coordinates
 * before calling the appropriate sub-procedure.
 * @param start_i, goal_i The start and goal for the first agent.
 * @param start_j, goal_j The start and goal for the second agent.
 * @return The conflict relationship (Hostile, Compatible, or Free).
 */
ConflictRelation getConflictRelation(const Point& start_i, const Point& goal_i,
                                     const Point& start_j, const Point& goal_j);
