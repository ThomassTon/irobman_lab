#pragma once 
#include "sequencing.h"
#include "../planners/prioritized_planner.h"
#include "../planners/cooperation_planner.h"
#include "../planners/stacking_planner.h"


Plan plan_multiple_arms_unsynchronized(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  const auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

Plan plan_single_arm_unsynchronized(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  const auto seq = generate_single_arm_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_single_arm_given_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

Plan plan_cooperation_arm_unsynchronized(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  const auto seq = generate_cooperation_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_cooperation_arms_given_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

Plan plan_single_arm_stacking(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  const auto seq = generate_single_arm_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_stacking_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

Plan plan_cooperation_arm_stacking(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses, uint &num_tasks) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  num_tasks = rtpm.begin()->second.size();

  const auto seq = generate_cooperation_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_stacking_sequence_collaboration(C, rtpm, seq, home_poses);
  return plan_result.plan;
}