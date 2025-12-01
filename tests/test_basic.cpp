#include <cassert>
#include <iostream>
#include <vector>
#include "conflict.hpp"

namespace
{
// Coordinate transformation helper: rotates a point counter-clockwise
  Point rotate270degree(Point p)
  {
    int t = p.x;
    p.x = p.y;
    p.y = -t;
    return p;
  }
  Point rotate180degree(Point p)
  {
    p.x = -p.x;
    p.y = -p.y;
    return p;
  }
  Point rotate90degree(Point p)
  {
    int t = p.x;
    p.x = -p.y;
    p.y = t;
    return p;
  }
}

void test_point_operations() {
    std::cout << "Testing Point operations..." << std::endl;
    
    Point p1(3, 4);
    Point p2(3, 4);
    Point p3(5, 6);
    
    assert(p1 == p2);
    assert(p1 != p3);
    std::cout << "✓ Point equality tests passed" << std::endl;
}

void test_bounding_box() {
    std::cout << "Testing BoundingBox operations..." << std::endl;
    
    Point start(1, 2);
    Point goal(5, 7);
    BoundingBox box = getBoundingBox(start, goal);
    
    assert(box.min_x == 1 && box.max_x == 5);
    assert(box.min_y == 2 && box.max_y == 7);
    assert(box.isValid());
    
    // Test overlap
    BoundingBox box2{3, 8, 4, 9};
    assert(isBoundingBoxesOverlap(box, box2));
    
    BoundingBox box3{10, 15, 10, 15};
    assert(!isBoundingBoxesOverlap(box, box3));
    
    std::cout << "✓ BoundingBox tests passed" << std::endl;
}

void test_basic_conflicts() {
    std::cout << "Testing basic conflict cases..." << std::endl;
    
    // Same start position - should be Hostile
    ConflictRelation rel1 = getConflictRelation(Point(0,0), Point(1,0), Point(0,0), Point(0,1));
    assert(rel1 == ConflictRelation::Hostile);
    
    // Same goal position - should be Hostile  
    ConflictRelation rel2 = getConflictRelation(Point(0,0), Point(1,1), Point(2,2), Point(1,1));
    assert(rel2 == ConflictRelation::Hostile);
    
    // No overlap - should be Free
    ConflictRelation rel3 = getConflictRelation(Point(0,0), Point(1,1), Point(5,5), Point(6,6));
    assert(rel3 == ConflictRelation::Free);
    
    std::cout << "✓ Basic conflict tests passed" << std::endl;
}

void test_getConflictRelation()
{
  std::cout << "Testing getConflictRelation..." << std::endl;

  // 测试算例数据结构
  struct RelationTestCase {
    uint id;
    Point s1,g1,s2,g2;
    ConflictRelation r;

    RelationTestCase(uint case_id,
            const Point & _s1, const Point & _g1, const Point & _s2, const Point & _g2,
            const ConflictRelation & relation )
      : id(case_id), s1(_s1), g1(_g1), s2(_s2), g2(_g2), r(relation) {}
  };

  // 定义测试算例
  int case_id = 1;
  std::vector<RelationTestCase> test_cases = {
    // 基础边界情况
    RelationTestCase(case_id++, Point(0, 0), Point(1, 0), Point(0, 0), Point(0, 1), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 0), Point(2, 2), Point(1, 1), Point(2, 2), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 0), Point(1, 1), Point(5, 5), Point(6, 6), ConflictRelation::Free),

    // B=0: 两个智能体都不移动
    RelationTestCase(case_id++, Point(0, 0), Point(0, 0), Point(2, 2), Point(2, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(0, 0), Point(0, 0), Point(0, 0), ConflictRelation::Hostile),

    // B=1: 一个移动一个静止
    RelationTestCase(case_id++, Point(0, 0), Point(0, 0), Point(0, 0), Point(2, 0), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(1, 1), Point(1, 1), Point(0, 1), Point(2, 1), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(1, 0), Point(1, 0), Point(0, 1), Point(2, 1), ConflictRelation::Free),

    // B=2 Case 1.1: 平行移动
    RelationTestCase(case_id++, Point(0, 0), Point(3, 0), Point(1, 0), Point(4, 0), ConflictRelation::Free),

    RelationTestCase(case_id++, Point(0, 0), Point(4, 0), Point(1, 0), Point(2, 0), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 0), Point(4, 0), Point(1, 0), Point(5, 0), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(4, 0), Point(-1, 0), Point(3, 0), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(1, 0), Point(2, 0), Point(0, 0), Point(4, 0), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(1, 0), Point(5, 0), Point(0, 0), Point(4, 0), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(-1, 0), Point(3, 0), Point(0, 0), Point(4, 0), ConflictRelation::Free),

    // B=2 Case 1.2: 头对头
    RelationTestCase(case_id++, Point(0, 0), Point(3, 0), Point(4, 0), Point(1, 0), ConflictRelation::Hostile),

    // B=2 Case 1.3: 垂直交叉
    RelationTestCase(case_id++, Point(0, 0), Point(2, 0), Point(1, -1), Point(1, 1), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 0), Point(3, 0), Point(2, -1), Point(2, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(3, 0), Point(2, 1), Point(2, -2), ConflictRelation::Free),
    // Ti > Tj
    RelationTestCase(case_id++, Point(0, 1), Point(4, 1), Point(2, 0), Point(2, 1), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 1), Point(4, 1), Point(2, 0), Point(2, 3), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 1), Point(4, 1), Point(1, -1), Point(1, 1), ConflictRelation::Free),
    // Ti < Tj
    RelationTestCase(case_id++, Point(0, 3), Point(2, 3), Point(2, 0), Point(2, 5), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 1), Point(2, 1), Point(2, 0), Point(2, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(1, 2), Point(3, 2), Point(2, 0), Point(2, 5), ConflictRelation::Free),

    // B=3 Case 2.1: 2D vs 1D 水平
    //six + siy = sjx + sjy
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 2), Point(4, 2), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-5, 5), Point(6, 5), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-3, 3), Point(5, 3), ConflictRelation::Hostile),
    // Ti = Tj
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 2), Point(9, 2), ConflictRelation::Free),
    // Ti > Tj
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 2), Point(6, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 1), Point(4, 1), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 2), Point(8, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 2), Point(5, 2), ConflictRelation::Compatible),
    // Ti < Tj
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 2), Point(10, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 1), Point(9, 1), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-4, 5), Point(6, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-7, 5), Point(4, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-6, 5), Point(6, 5), ConflictRelation::Hostile),

    // B=3 Case 2.2: 2D vs 1D 反向水平
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(15, 5), Point(0, 5), ConflictRelation::Hostile),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(13, 5), Point(0, 5), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(14, 5), Point(0, 5), ConflictRelation::Compatible),
    // Ti = Tj
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(1, 5), Point(-6, 5), ConflictRelation::Free),
    // Ti > Tj
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(1, 5), Point(-1, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(-2, 2), Point(-3, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(6, 4), Point(3, 4), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(3, 4), Point(1, 4), ConflictRelation::Compatible),
    // Ti < Tj
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(3, 4), Point(-5, 4), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(3, 5), Point(-5, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(13, 5), Point(3, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(2, 5), Point(10, 5), Point(1, 5), ConflictRelation::Hostile),

    // B=4 Case 3.1: 两个2D同向
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 1), Point(4, 6), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(1, -1), Point(6, 4), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 2), Point(6, 4), ConflictRelation::Hostile),
    // Ti = Tj
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 3), Point(6, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 1), Point(6, 3), ConflictRelation::Free),
    // Ti > Tj
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 1), Point(6, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-2, 1), Point(5, 2), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 3), Point(5, 6), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(-1, 3), Point(5, 4), ConflictRelation::Compatible),
    // Ti < Tj : 通过将 Ti > Tj 的情况转换可验证

    // B=4 Case 3.2: 2D vs 2D 交叉
    // Ti = Tj
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(0, 7), Point(6, 0), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(0, 11), Point(2, 3), ConflictRelation::Free),
    // Ti > Tj
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(3, 6), Point(6, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(4, 6), Point(6, 4), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(3, 6), Point(3, 5), ConflictRelation::Compatible),
    // Ti < Tj : 通过将 Ti > Tj 的情况转换可验证

    // B=4 Case 3.4: 2D vs 2D 头对头
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(8, 8), Point(3, 3), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(7, 7), Point(3, 3), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(13, 2), Point(5, 0), ConflictRelation::Free),
    // T_i > T_j
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(1, 6), Point(-1, 5), ConflictRelation::Free),
    RelationTestCase(case_id++, Point(3, 0), Point(5, 5), Point(9, 2), Point(5, 0), ConflictRelation::Compatible),
    RelationTestCase(case_id++, Point(0, 0), Point(5, 5), Point(3, 6), Point(3, 4), ConflictRelation::Compatible),
  };

  // 遍历每个测试算例
  for (size_t i = 0; i < test_cases.size(); ++i) {
    const auto& t = test_cases[i];
    std::vector<Point> P = {t.s1, t.g1, t.s2, t.g2,
                  rotate90degree(t.s1),rotate90degree(t.g1),rotate90degree(t.s2),rotate90degree(t.g2),
                  rotate180degree(t.s1),rotate180degree(t.g1),rotate180degree(t.s2),rotate180degree(t.g2),
                  rotate270degree(t.s1),rotate270degree(t.g1),rotate270degree(t.s2),rotate270degree(t.g2)
    };
    for(uint j = 0; j < 16; j+=4){
      ConflictRelation actual = getConflictRelation(P[j], P[j+1], P[j+2], P[j+3]);
      assert(actual == t.r);
    }
  }
  std::cout << "✓ getConflictRelation tests passed" << std::endl;
}

int main() {
    std::cout << "=== POCO Comprehensive Tests ===" << std::endl;
    
    try {
        test_point_operations();
        test_bounding_box();
        test_basic_conflicts();
        test_getConflictRelation();
        std::cout << "\n=== All tests passed! ===" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
