#include <gtest/gtest.h>
#include "../include/right_left_planner/path_segment.hpp"

namespace right_left_planner {

TEST(PathSegmentTest, withinDelta) {
    ASSERT_TRUE(withinDelta(1.0, 1.0, 0.1));
    ASSERT_TRUE(withinDelta(1.0, 1.1, 0.1));
    ASSERT_TRUE(withinDelta(1.0, 0.9, 0.1));
    ASSERT_FALSE(withinDelta(1.0, 1.2, 0.1));
    ASSERT_FALSE(withinDelta(1.0, 0.8, 0.1));
}

TEST(PathSegmentTest, getAngle) {
    Point p0(1,1);
    Point p1(1, 0);
    Point p2(0, 0);
    Point p3(0, 1);

    ASSERT_DOUBLE_EQ(getAngle(p1, p2, p0), M_PI_4);
    ASSERT_DOUBLE_EQ(getAngle(p1, p2, p3), M_PI_2);
    ASSERT_DOUBLE_EQ(getAngle(p3, p2, p1), -M_PI_2);

    Point p4(0, 0);
    Point p5(1, -1);
    Point p6(4, 0);

    double angle = invertAngle(-getAngle(p6, p5, p4)) * 180 / M_PI;

    std::cout << "Angle: " << angle << std::endl;

    ASSERT_TRUE(withinDelta(angle, 63.0, 1));
}

TEST(PathSegmentTest, wrapPolygon) {
    std::shared_ptr<bool> cw_visited = std::make_shared<bool>(false);
    std::shared_ptr<bool> ccw_visited = std::make_shared<bool>(false);

    SegmentMetadata testMetadata = SegmentMetadata();
    testMetadata.polygon_visited_clockwise = cw_visited;
    testMetadata.polygon_visited_counterclockwise = ccw_visited;

    StoredSegmentPtr s1 = std::make_shared<StoredSegment>(StoredSegment(
                Segment(Point(1,1),Point(1,-1)),
                testMetadata
    ));
    StoredSegmentPtr s2 = std::make_shared<StoredSegment>(StoredSegment(
            Segment(Point(1,-1), Point(3,-1)),
            testMetadata
    ));

    s1->second.counterclockwise_segment = s2;
    s2->second.clockwise_segment = s1;

    StoredSegmentPtr s3 = std::make_shared<StoredSegment>(StoredSegment(
            Segment(Point(3, -1), Point(3, 1)),
            testMetadata
    ));
    s2->second.counterclockwise_segment = s3;
    s3->second.clockwise_segment = s2;

    StoredSegmentPtr s4 = std::make_shared<StoredSegment>(StoredSegment(
            Segment(Point(3, 1), Point(1,1)),
            testMetadata
    ));

    s3->second.counterclockwise_segment = s4;
    s4->second.clockwise_segment = s3;

    s4->second.counterclockwise_segment = s1;
    s1->second.clockwise_segment = s4;

    PathSegment testSegment = PathSegment();
    testSegment.segment = Segment(Point(0,0), Point(4,0));

    ASSERT_DOUBLE_EQ(testSegment.getLength(), 4.0);
    ASSERT_DOUBLE_EQ(testSegment.getCost(), 4.0);

    auto result = testSegment.wrapPolygon(s1, std::numeric_limits<double>::max(), nullptr);
}

} // namespace right_left_planner
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
