#include "task_imitation/program_slice.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include <gtest/gtest.h>

using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;

namespace pbi {
TEST(ProgramSliceTest, PlannedStepGetTrajSinglePoint) {
  // Step with a single point from time 1 to time 3 with positions[0] = 5
  PlannedStep step;
  step.traj.header.stamp = ros::Time(1);
  JointTrajectoryPoint pt;
  pt.time_from_start = ros::Duration(2);
  pt.positions.push_back(5);
  step.traj.points.push_back(pt);

  // Requested range is before the point
  JointTrajectory traj = step.GetTraj(ros::Time(0), ros::Time(0.5));
  EXPECT_EQ(0, traj.points.size());

  // Request starts before the point, ends in the middle of the point
  traj = step.GetTraj(ros::Time(0), ros::Time(1.5));
  ASSERT_EQ(1, traj.points.size());
  EXPECT_EQ(5, traj.points[0].positions[0]);
  EXPECT_EQ(0.5, traj.points[0].time_from_start.toSec());

  // Request starts before the point, ends after the point
  traj = step.GetTraj(ros::Time(0), ros::Time(4));
  ASSERT_EQ(1, traj.points.size());
  EXPECT_EQ(5, traj.points[0].positions[0]);
  EXPECT_EQ(2, traj.points[0].time_from_start.toSec());

  // Request starts in the middle of the point, ends in the middle of the point
  traj = step.GetTraj(ros::Time(2), ros::Time(2.5));
  ASSERT_EQ(1, traj.points.size());
  EXPECT_EQ(5, traj.points[0].positions[0]);
  EXPECT_EQ(0.5, traj.points[0].time_from_start.toSec());

  // Request starts in the middle of the point, ends after the point
  traj = step.GetTraj(ros::Time(2), ros::Time(5.5));
  ASSERT_EQ(1, traj.points.size());
  EXPECT_EQ(5, traj.points[0].positions[0]);
  EXPECT_EQ(1, traj.points[0].time_from_start.toSec());

  // Request starts after the point, ends after the point
  traj = step.GetTraj(ros::Time(5), ros::Time(5.5));
  EXPECT_EQ(0, traj.points.size());
}

TEST(ProgramSliceTest, PlannedStepGetTrajEmpty) {
  // Step with a single point from time 1 to time 3 with positions[0] = 5
  PlannedStep step;
  step.traj.header.stamp = ros::Time(1);
  JointTrajectoryPoint pt;
  pt.time_from_start = ros::Duration(2);
  pt.positions.push_back(5);
  step.traj.points.push_back(pt);

  // Requested range is just before the point
  JointTrajectory traj = step.GetTraj(ros::Time(0), ros::Time(1));
  EXPECT_EQ(0, traj.points.size());
}

TEST(ProgramSliceTest, PlannedStepGetTrajIncludeEnd) {
  // Step with a single point from time 1 to time 3 with positions[0] = 5
  PlannedStep step;
  step.traj.header.stamp = ros::Time(1);
  JointTrajectoryPoint pt;
  pt.time_from_start = ros::Duration(2);
  pt.positions.push_back(5);
  step.traj.points.push_back(pt);

  // Requested range is just before the point
  JointTrajectory traj = step.GetTraj(ros::Time(1.5), ros::Time(3));
  ASSERT_EQ(1, traj.points.size());
  EXPECT_EQ(5, traj.points[0].positions[0]);
  EXPECT_EQ(1.5, traj.points[0].time_from_start.toSec());
}

TEST(ProgramSliceTest, PlannedStepGetTrajMultiplePoints) {
  // Step with 3 points:
  // Time 1-3: positions[0] = 5
  // Time 3-4: positions[0] = 4
  // Time 4-5: positions[0] = 3
  PlannedStep step;
  step.traj.header.stamp = ros::Time(1);

  JointTrajectoryPoint pt;
  pt.time_from_start = ros::Duration(2);
  pt.positions.push_back(5);
  step.traj.points.push_back(pt);

  pt.time_from_start = ros::Duration(3);
  pt.positions[0] = 4;
  step.traj.points.push_back(pt);

  pt.time_from_start = ros::Duration(4);
  pt.positions[0] = 3;
  step.traj.points.push_back(pt);

  JointTrajectory traj = step.GetTraj(ros::Time(1.5), ros::Time(4.5));
  ASSERT_EQ(3, traj.points.size());
  EXPECT_EQ(1.5, traj.header.stamp.toSec());
  EXPECT_EQ(5, traj.points[0].positions[0]);
  EXPECT_EQ(4, traj.points[1].positions[0]);
  EXPECT_EQ(3, traj.points[2].positions[0]);
  EXPECT_EQ(1.5, traj.points[0].time_from_start.toSec());
  EXPECT_EQ(1, traj.points[1].time_from_start.toSec());
  EXPECT_EQ(0.5, traj.points[2].time_from_start.toSec());
}
}  // namespace pbi

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
