#include "task_imitation/bimanual_manipulation.h"

#include <gtest/gtest.h>

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;

namespace pbi {
TEST(BimanualManipulationTest, MergeSimple) {
  JointTrajectory left;
  left.joint_names.push_back("left");
  JointTrajectoryPoint left_pt;
  left_pt.positions.push_back(11);
  left_pt.time_from_start = ros::Duration(1);
  left.points.push_back(left_pt);

  JointTrajectory right;
  right.joint_names.push_back("right");
  JointTrajectoryPoint right_pt;
  right_pt.positions.push_back(12);
  right_pt.time_from_start = ros::Duration(2);
  right.points.push_back(right_pt);

  JointTrajectory merged = MergeTrajectories(left, right);
  ASSERT_EQ(merged.points.size(), 2);
  ASSERT_EQ(merged.joint_names.size(), 2);
  EXPECT_EQ(merged.joint_names[0], "left");
  EXPECT_EQ(merged.joint_names[1], "right");

  const JointTrajectoryPoint& merged_pt1 = merged.points[0];
  EXPECT_FLOAT_EQ(merged_pt1.time_from_start.toSec(), 1);
  ASSERT_EQ(merged_pt1.positions.size(), 2);
  EXPECT_FLOAT_EQ(merged_pt1.positions[0], 11);
  EXPECT_FLOAT_EQ(merged_pt1.positions[1], 12);

  const JointTrajectoryPoint& merged_pt2 = merged.points[1];
  EXPECT_FLOAT_EQ(merged_pt2.time_from_start.toSec(), 2);
  ASSERT_EQ(merged_pt2.positions.size(), 2);
  EXPECT_FLOAT_EQ(merged_pt2.positions[0], 11);
  EXPECT_FLOAT_EQ(merged_pt2.positions[1], 12);
}

TEST(BimanualManipulationTest, MergeOnePastTheOther) {
  // One pt: [0-1]
  JointTrajectory left;
  left.joint_names.push_back("left");
  JointTrajectoryPoint left_pt;
  left_pt.positions.push_back(11);
  left_pt.time_from_start = ros::Duration(1);
  left.points.push_back(left_pt);

  // Two points: [0-2], [2-3]
  JointTrajectory right;
  right.joint_names.push_back("right");
  JointTrajectoryPoint right_pt1;
  right_pt1.positions.push_back(12);
  right_pt1.time_from_start = ros::Duration(2);
  JointTrajectoryPoint right_pt2;
  right_pt2.positions.push_back(13);
  right_pt2.time_from_start = ros::Duration(3);
  right.points.push_back(right_pt1);
  right.points.push_back(right_pt2);

  // Expect:
  //   [0-1] (11, 12)
  //   [1-2] (11, 12)
  //   [2-3] (11, 13)
  JointTrajectory merged = MergeTrajectories(left, right);
  ASSERT_EQ(merged.points.size(), 3);
  ASSERT_EQ(merged.joint_names.size(), 2);
  EXPECT_EQ(merged.joint_names[0], "left");
  EXPECT_EQ(merged.joint_names[1], "right");

  const JointTrajectoryPoint& merged_pt1 = merged.points[0];
  EXPECT_FLOAT_EQ(merged_pt1.time_from_start.toSec(), 1);
  ASSERT_EQ(merged_pt1.positions.size(), 2);
  EXPECT_FLOAT_EQ(merged_pt1.positions[0], 11);
  EXPECT_FLOAT_EQ(merged_pt1.positions[1], 12);

  const JointTrajectoryPoint& merged_pt2 = merged.points[1];
  EXPECT_FLOAT_EQ(merged_pt2.time_from_start.toSec(), 2);
  ASSERT_EQ(merged_pt2.positions.size(), 2);
  EXPECT_FLOAT_EQ(merged_pt2.positions[0], 11);
  EXPECT_FLOAT_EQ(merged_pt2.positions[1], 12);

  const JointTrajectoryPoint& merged_pt3 = merged.points[2];
  EXPECT_FLOAT_EQ(merged_pt3.time_from_start.toSec(), 3);
  ASSERT_EQ(merged_pt3.positions.size(), 2);
  EXPECT_FLOAT_EQ(merged_pt3.positions[0], 11);
  EXPECT_FLOAT_EQ(merged_pt3.positions[1], 13);
}
}  // namespace pbi

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
