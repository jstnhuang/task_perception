#include "task_imitation/program_executor.h"

#include <gtest/gtest.h>
#include <vector>

#include "task_perception_msgs/ProgramSlice.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "task_imitation/program_slice.h"

using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;
using task_perception_msgs::ProgramSlice;

namespace pbi {
TEST(ProgramExecutorTest, SliceProgramOneStepOneHand) {
  PlannedStep pregrasp;
  JointTrajectoryPoint pregrasp_pt;
  pregrasp_pt.positions.push_back(7);
  pregrasp_pt.time_from_start = ros::Duration(1);
  pregrasp.traj.header.stamp = ros::Time(1);
  pregrasp.traj.points.push_back(pregrasp_pt);

  std::vector<PlannedStep> left_steps;
  left_steps.push_back(pregrasp);
  std::vector<PlannedStep> right_steps;

  std::vector<ProgramSlice> slices = SliceProgram(left_steps, right_steps);
  ASSERT_EQ(slices.size(), 1);
  EXPECT_EQ(slices[0].right_traj.points.size(), 0);
  // Check left traj exists from time 1-2
  ASSERT_EQ(slices[0].left_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slices[0].left_traj.header.stamp.toSec(), 1);
  EXPECT_EQ(slices[0].left_traj.points[0].positions[0], 7);
  EXPECT_FLOAT_EQ(slices[0].left_traj.points[0].time_from_start.toSec(), 1);
}

TEST(ProgramExecutorTest, SliceProgramTwoStepsOneHand) {
  PlannedStep pregrasp;
  JointTrajectoryPoint pregrasp_pt;
  pregrasp_pt.positions.push_back(7);
  pregrasp_pt.time_from_start = ros::Duration(1);
  pregrasp.traj.header.stamp = ros::Time(1);
  pregrasp.traj.points.push_back(pregrasp_pt);

  PlannedStep grasp;
  JointTrajectoryPoint grasp_pt;
  grasp_pt.positions.push_back(8);
  grasp_pt.time_from_start = ros::Duration(1);
  grasp.traj.header.stamp = ros::Time(2.1);
  grasp.traj.points.push_back(grasp_pt);

  std::vector<PlannedStep> left_steps;
  left_steps.push_back(pregrasp);
  left_steps.push_back(grasp);
  std::vector<PlannedStep> right_steps;

  std::vector<ProgramSlice> slices = SliceProgram(left_steps, right_steps);
  ASSERT_EQ(slices.size(), 2);

  // Right traj is empty for both slices
  EXPECT_EQ(slices[0].right_traj.points.size(), 0);
  EXPECT_EQ(slices[1].right_traj.points.size(), 0);

  // Check left traj for first slice
  const JointTrajectory& left_traj0 = slices[0].left_traj;
  EXPECT_FLOAT_EQ(left_traj0.header.stamp.toSec(), 1);
  ASSERT_EQ(left_traj0.points.size(), 1);
  EXPECT_EQ(left_traj0.points[0].positions[0], 7);
  EXPECT_FLOAT_EQ(left_traj0.points[0].time_from_start.toSec(), 1);

  const JointTrajectory& left_traj1 = slices[1].left_traj;
  EXPECT_FLOAT_EQ(left_traj1.header.stamp.toSec(), 2.1);
  ASSERT_EQ(left_traj1.points.size(), 1);
  EXPECT_EQ(left_traj1.points[0].positions[0], 8);
  EXPECT_FLOAT_EQ(left_traj1.points[0].time_from_start.toSec(), 1);
}

TEST(ProgramExecutorTest, SliceProgramTwoStepsOneHandExactOverlap) {
  // Two steps from [1-2] and [2-3]
  PlannedStep pregrasp;
  JointTrajectoryPoint pregrasp_pt;
  pregrasp_pt.positions.push_back(7);
  pregrasp_pt.time_from_start = ros::Duration(1);
  pregrasp.traj.header.stamp = ros::Time(1);
  pregrasp.traj.points.push_back(pregrasp_pt);

  PlannedStep grasp;
  JointTrajectoryPoint grasp_pt;
  grasp_pt.positions.push_back(8);
  grasp_pt.time_from_start = ros::Duration(1);
  grasp.traj.header.stamp = ros::Time(2);
  grasp.traj.points.push_back(grasp_pt);

  std::vector<PlannedStep> left_steps;
  left_steps.push_back(pregrasp);
  left_steps.push_back(grasp);
  std::vector<PlannedStep> right_steps;

  std::vector<ProgramSlice> slices = SliceProgram(left_steps, right_steps);
  ASSERT_EQ(slices.size(), 2);

  // Right traj is empty for both slices
  EXPECT_EQ(slices[0].right_traj.points.size(), 0);
  EXPECT_EQ(slices[1].right_traj.points.size(), 0);

  // Check left traj for first slice
  const JointTrajectory& left_traj0 = slices[0].left_traj;
  EXPECT_FLOAT_EQ(left_traj0.header.stamp.toSec(), 1);
  ASSERT_EQ(left_traj0.points.size(), 1);
  EXPECT_EQ(left_traj0.points[0].positions[0], 7);
  EXPECT_FLOAT_EQ(left_traj0.points[0].time_from_start.toSec(), 1);

  const JointTrajectory& left_traj1 = slices[1].left_traj;
  EXPECT_FLOAT_EQ(left_traj1.header.stamp.toSec(), 2);
  ASSERT_EQ(left_traj1.points.size(), 1);
  EXPECT_EQ(left_traj1.points[0].positions[0], 8);
  EXPECT_FLOAT_EQ(left_traj1.points[0].time_from_start.toSec(), 1);
}

TEST(ProgramExecutorTest, SliceProgramOneStepPerHandNoOverlap) {
  // Each hand has one step, but they don't overlap in time
  // Expect two slices, one for each step

  // Left arm trajectory from time 1-2
  PlannedStep left_step;
  JointTrajectoryPoint left_pt;
  left_pt.positions.push_back(7);
  left_pt.time_from_start = ros::Duration(1);
  left_step.traj.header.stamp = ros::Time(1);
  left_step.traj.points.push_back(left_pt);

  // Right arm trajectory from time 3-4
  PlannedStep right_step;
  JointTrajectoryPoint right_pt;
  right_pt.positions.push_back(8);
  right_pt.time_from_start = ros::Duration(1);
  right_step.traj.header.stamp = ros::Time(3);
  right_step.traj.points.push_back(right_pt);

  std::vector<PlannedStep> left_steps;
  left_steps.push_back(left_step);
  std::vector<PlannedStep> right_steps;
  right_steps.push_back(right_step);

  std::vector<ProgramSlice> slices = SliceProgram(left_steps, right_steps);
  ASSERT_EQ(slices.size(), 2);

  // Check first slice
  EXPECT_EQ(slices[0].right_traj.points.size(), 0);
  ASSERT_EQ(slices[0].left_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slices[0].left_traj.header.stamp.toSec(), 1);
  EXPECT_FLOAT_EQ(slices[0].left_traj.points[0].positions[0], 7);
  EXPECT_FLOAT_EQ(slices[0].left_traj.points[0].time_from_start.toSec(), 1);

  // Check second slice
  EXPECT_EQ(slices[1].left_traj.points.size(), 0);
  ASSERT_EQ(slices[1].right_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slices[1].right_traj.header.stamp.toSec(), 3);
  EXPECT_FLOAT_EQ(slices[1].right_traj.points[0].positions[0], 8);
  EXPECT_FLOAT_EQ(slices[1].right_traj.points[0].time_from_start.toSec(), 1);
}

TEST(ProgramExecutorTest, SliceProgramOneStepPerHandWithOverlap) {
  // Each hand has one step, but they don't overlap in time
  // Expect two slices, one for each step

  // Left arm trajectory from time 1-3
  PlannedStep left_step;
  JointTrajectoryPoint left_pt;
  left_pt.positions.push_back(7);
  left_pt.time_from_start = ros::Duration(2);
  left_step.traj.header.stamp = ros::Time(1);
  left_step.traj.points.push_back(left_pt);

  // Right arm trajectory from time 2-4
  PlannedStep right_step;
  JointTrajectoryPoint right_pt;
  right_pt.positions.push_back(8);
  right_pt.time_from_start = ros::Duration(2);
  right_step.traj.header.stamp = ros::Time(2);
  right_step.traj.points.push_back(right_pt);

  std::vector<PlannedStep> left_steps;
  left_steps.push_back(left_step);
  std::vector<PlannedStep> right_steps;
  right_steps.push_back(right_step);

  std::vector<ProgramSlice> slices = SliceProgram(left_steps, right_steps);

  // We expect three slices: 1-2, 2-3, and 3-4
  ASSERT_EQ(slices.size(), 3);

  // First slice
  EXPECT_EQ(slices[0].right_traj.points.size(), 0);
  ASSERT_EQ(slices[0].left_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slices[0].left_traj.header.stamp.toSec(), 1);
  EXPECT_FLOAT_EQ(slices[0].left_traj.points[0].positions[0], 7);
  EXPECT_FLOAT_EQ(slices[0].left_traj.points[0].time_from_start.toSec(), 1);

  // Second slice
  ASSERT_EQ(slices[1].left_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slices[1].left_traj.header.stamp.toSec(), 2);
  EXPECT_FLOAT_EQ(slices[1].left_traj.points[0].positions[0], 7);
  EXPECT_FLOAT_EQ(slices[1].left_traj.points[0].time_from_start.toSec(), 1);

  ASSERT_EQ(slices[1].right_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slices[1].right_traj.header.stamp.toSec(), 2);
  EXPECT_FLOAT_EQ(slices[1].right_traj.points[0].positions[0], 8);
  EXPECT_FLOAT_EQ(slices[1].right_traj.points[0].time_from_start.toSec(), 1);

  // Check second slice
  EXPECT_EQ(slices[2].left_traj.points.size(), 0);
  ASSERT_EQ(slices[2].right_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slices[2].right_traj.header.stamp.toSec(), 3);
  EXPECT_FLOAT_EQ(slices[2].right_traj.points[0].positions[0], 8);
  EXPECT_FLOAT_EQ(slices[2].right_traj.points[0].time_from_start.toSec(), 1);
}
}  // namespace pbi

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "task_imitation_program_executor_test");
  ros::NodeHandle nh;
  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
