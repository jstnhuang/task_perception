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
  pregrasp.traj.joint_names.push_back("test_joint");

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
  pregrasp.traj.joint_names.push_back("test_joint");

  PlannedStep grasp;
  JointTrajectoryPoint grasp_pt;
  grasp_pt.positions.push_back(8);
  grasp_pt.time_from_start = ros::Duration(1);
  grasp.traj.header.stamp = ros::Time(2.1);
  grasp.traj.points.push_back(grasp_pt);
  grasp.traj.joint_names.push_back("test_joint");

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
  pregrasp.traj.joint_names.push_back("test_joint");

  PlannedStep grasp;
  JointTrajectoryPoint grasp_pt;
  grasp_pt.positions.push_back(8);
  grasp_pt.time_from_start = ros::Duration(1);
  grasp.traj.header.stamp = ros::Time(2);
  grasp.traj.points.push_back(grasp_pt);
  grasp.traj.joint_names.push_back("test_joint");

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
  left_step.traj.joint_names.push_back("test_joint");

  // Right arm trajectory from time 3-4
  PlannedStep right_step;
  JointTrajectoryPoint right_pt;
  right_pt.positions.push_back(8);
  right_pt.time_from_start = ros::Duration(1);
  right_step.traj.header.stamp = ros::Time(3);
  right_step.traj.points.push_back(right_pt);
  right_step.traj.joint_names.push_back("test_joint");

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
  left_step.traj.joint_names.push_back("test_joint");

  // Right arm trajectory from time 2-4
  PlannedStep right_step;
  JointTrajectoryPoint right_pt;
  right_pt.positions.push_back(8);
  right_pt.time_from_start = ros::Duration(2);
  right_step.traj.header.stamp = ros::Time(2);
  right_step.traj.points.push_back(right_pt);
  right_step.traj.joint_names.push_back("test_joint");

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

TEST(ProgramExecutorTest, SliceProgramGripperOpensLater) {
  // Left hand has two slices: one that closes the gripper [0.1-1] and one that
  // opens the gripper [2-3].
  // Right hand has one slice that just moves the gripper [1-2].
  // Expect three slices: [0.1-1], [1-2], [2-3]. For the left hand:
  // 1. is_closing = true
  // 2. is_opening = false and is_closing = false
  // 3. is_opening = true

  PlannedStep left_step_1;
  JointTrajectoryPoint left_pt;
  left_pt.positions.push_back(7);
  left_pt.time_from_start = ros::Duration(0.9);
  left_step_1.traj.header.stamp = ros::Time(0.1);
  left_step_1.traj.points.push_back(left_pt);
  left_step_1.traj.joint_names.push_back("test_joint");
  left_step_1.is_closing = true;

  PlannedStep left_step_2;
  JointTrajectoryPoint left_pt_2;
  left_pt_2.positions.push_back(8);
  left_pt_2.time_from_start = ros::Duration(1);
  left_step_2.traj.header.stamp = ros::Time(2);
  left_step_2.traj.points.push_back(left_pt_2);
  left_step_2.traj.joint_names.push_back("test_joint");
  left_step_2.is_opening = true;

  PlannedStep right_step;
  JointTrajectoryPoint right_pt;
  right_pt.positions.push_back(9);
  right_pt.time_from_start = ros::Duration(1);
  right_step.traj.header.stamp = ros::Time(1);
  right_step.traj.points.push_back(right_pt);
  right_step.traj.joint_names.push_back("test_joint");

  std::vector<PlannedStep> left_steps;
  left_steps.push_back(left_step_1);
  left_steps.push_back(left_step_2);
  std::vector<PlannedStep> right_steps;
  right_steps.push_back(right_step);

  std::vector<ProgramSlice> slices = SliceProgram(left_steps, right_steps);

  // We expect three slices: 0.1-1, 1-2, and 2-3
  ASSERT_EQ(slices.size(), 3);

  // First slice
  const ProgramSlice& slice_0 = slices.at(0);
  ASSERT_EQ(slice_0.left_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slice_0.left_traj.header.stamp.toSec(), 0.1);
  EXPECT_FLOAT_EQ(slice_0.left_traj.points[0].positions[0], 7);
  EXPECT_FLOAT_EQ(slice_0.left_traj.points[0].time_from_start.toSec(), 0.9);
  EXPECT_TRUE(slice_0.is_left_closing);
  EXPECT_FALSE(slice_0.is_left_opening);

  EXPECT_EQ(slice_0.right_traj.points.size(), 0);
  EXPECT_FALSE(slice_0.is_right_closing);
  EXPECT_FALSE(slice_0.is_right_opening);

  // Second slice
  const ProgramSlice& slice_1 = slices.at(1);
  EXPECT_EQ(slice_1.left_traj.points.size(), 0);
  EXPECT_FALSE(slice_1.is_left_closing);
  EXPECT_FALSE(slice_1.is_left_opening);

  ASSERT_EQ(slice_1.right_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slice_1.right_traj.header.stamp.toSec(), 1);
  EXPECT_FLOAT_EQ(slice_1.right_traj.points[0].positions[0], 9);
  EXPECT_FLOAT_EQ(slice_1.right_traj.points[0].time_from_start.toSec(), 1);
  EXPECT_FALSE(slice_1.is_right_closing);
  EXPECT_FALSE(slice_1.is_right_opening);

  // Third slice
  const ProgramSlice& slice_2 = slices.at(2);
  ASSERT_EQ(slice_2.left_traj.points.size(), 1);
  EXPECT_FLOAT_EQ(slice_2.left_traj.header.stamp.toSec(), 2);
  EXPECT_FLOAT_EQ(slice_2.left_traj.points[0].positions[0], 8);
  EXPECT_FLOAT_EQ(slice_2.left_traj.points[0].time_from_start.toSec(), 1);
  EXPECT_FALSE(slice_2.is_left_closing);
  EXPECT_TRUE(slice_2.is_left_opening);

  EXPECT_EQ(slice_2.right_traj.points.size(), 0);
  EXPECT_FALSE(slice_2.is_right_closing);
  EXPECT_FALSE(slice_2.is_right_opening);
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
