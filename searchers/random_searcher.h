#pragma once 
#include "sequencing.h"
#include "../planners/prioritized_planner.h"

Plan plan_multiple_arms_random_search(rai::Configuration &C,
                                      const RobotTaskPoseMap &rtpm,
                                      const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  TaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  for (uint i = 0; i < 100; ++i) {
    const auto seq = generate_random_sequence(robots, num_tasks);

    const double lb = compute_lb_for_sequence(seq, rtpm, home_poses);
    std::cout << "LB for sequence " << lb << std::endl;
    for (auto s : seq) {
      std::cout << "(" << s.first << " " << s.second << ")";
    }
    std::cout << std::endl;

    if (lb > best_makespan) {
      continue;
    }

    // plan for it
    const auto plan_result = plan_multiple_arms_given_sequence(
        C, rtpm, seq, home_poses, best_makespan);
    if (plan_result.status == PlanStatus::success) {
      const Plan plan = plan_result.plan;
      const double makespan = get_makespan_from_plan(plan);

      std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                << best_makespan << std::endl;
      for (auto s : seq) {
        std::cout << "(" << s.first << " " << s.second << ")";
      }
      std::cout << "\n\n\n" << std::endl;

      if (makespan < best_makespan) {
        best_makespan = makespan;
        best_plan = plan;
        break;
      }
    }
  }
  return best_plan;
}