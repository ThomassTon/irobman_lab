#pragma once 
#include "sequencing.h"
#include "../planners/prioritized_planner.h"
#include "../utils/plan.h"

Plan plan_multiple_arms_greedy_random_search(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
  // make foldername for current run
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream buffer;
  buffer << "greedy_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  TaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::pair<TaskSequence, Plan>> cache;

  uint iter = 0;
  bool early_stop = false;
  for (uint i = 0; i < 20000; ++i) {
    if (early_stop) {
      break;
    }
    std::cout << "Generating completely new seq. " << i << std::endl;
    TaskSequence seq;
    // seq = generate_alternating_random_sequence(robots, num_tasks, rtpm);
    // seq = generate_single_arm_sequence(robots, num_tasks);
    seq = generate_alternating_greedy_sequence(robots, num_tasks, rtpm,
                                               home_poses);
    /*if (true || i == 0) {
      // seq = generate_single_arm_sequence(robots, num_tasks);
      //seq = generate_random_sequence(robots, num_tasks);
      // seq = generate_alternating_greedy_sequence(robots, num_tasks, rtpm,
    home_poses); } else if (i == 1) { seq =
    generate_alternating_random_sequence(robots, num_tasks, rtpm); } else if (i
    == 2) { seq = generate_single_arm_sequence(robots, num_tasks); } else { seq
    = generate_random_sequence(robots, num_tasks);
    }*/

    if (!sequence_is_feasible(seq, rtpm)) {
      std::cout << "Generated sequence no feasible" << std::endl;
      continue;
    }

    Plan plan;
    double prev_makespan = 1e6;
    for (uint j = 0; j < 30; ++j) {
      ++iter;
      TaskSequence new_seq = seq;
      while (true) {
        if (j > 0) {
          new_seq = neighbour(seq, robots);
        }

        // ensure that sequence is actually feasible, i.e. robots can do the
        // assigned tasks
        if (sequence_is_feasible(new_seq, rtpm)) {
          break;
        }
      }

      const double lb = compute_lb_for_sequence(new_seq, rtpm, home_poses);
      std::cout << "LB for sequence " << lb << std::endl;
      for (const auto &s : new_seq) {
        std::cout << "(" << s.first << " " << s.second << ")";
      }
      std::cout << std::endl;

      if (lb > best_makespan) {
        std::cout << "skipping planning, since lb is larger than best plan"
                  << std::endl;
        continue;
      }

      // plan for it
      PlanResult new_plan_result;
      if (plan.empty()) {
        new_plan_result = plan_multiple_arms_given_sequence(
            C, rtpm, new_seq, home_poses, prev_makespan);
      } else {
        // compute index where the new sequence starts
        uint change_in_sequence = 0;
        for (uint k = 0; k < seq.size(); ++k) {
          if (seq[k].first != new_seq[k].first ||
              seq[k].second != new_seq[k].second) {
            change_in_sequence = k;
            break;
          }
        }
        std::cout << "planning only subsequence " << change_in_sequence
                  << std::endl;
        new_plan_result = plan_multiple_arms_given_subsequence_and_prev_plan(
            C, rtpm, new_seq, change_in_sequence, plan, home_poses,
            prev_makespan);
      }

      if (new_plan_result.status == PlanStatus::success) {
        const Plan new_plan = new_plan_result.plan;
        const double makespan = get_makespan_from_plan(new_plan);

        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                                  end_time - start_time)
                                  .count();

        // cache.push_back(std::make_pair(new_seq, new_plan));

        export_plan(robots, home_poses, new_plan, new_seq, buffer.str(), iter,
                    duration);

        std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                  << best_makespan << " (" << prev_makespan << ")" << std::endl;
        for (const auto &s : new_seq) {
          std::cout << "(" << s.first << " " << s.second << ")";
        }
        std::cout << "\n\n\n" << std::endl;

        if (makespan < prev_makespan) {
          seq = new_seq;
          plan = new_plan;
          prev_makespan = makespan;

          // visualize_plan(C, plan, save_video, save_video_path); //!
          // early_stop = true;
          // break;
        }

        if (makespan < best_makespan) {
          best_makespan = makespan;
          best_plan = plan;
          best_seq = new_seq;
          early_stop = true;
          break;

          // visualize_plan(C, best_plan);
        }
      } else {
        const std::string folder =
            "./out/" + buffer.str() + "/" + std::to_string(iter) + "/";
        const int res = system(STRING("mkdir -p " << folder).p);
        (void)res;

        {
          std::ofstream f;
          f.open(folder + "comptime.txt", std::ios_base::trunc);
          const auto end_time = std::chrono::high_resolution_clock::now();
          const auto duration =
              std::chrono::duration_cast<std::chrono::seconds>(end_time -
                                                               start_time)
                  .count();
          f << duration;
        }
        {
          std::ofstream f;
          if (new_plan_result.status == PlanStatus::failed) {
            f.open(folder + "failed.txt", std::ios_base::trunc);
          } else if (new_plan_result.status == PlanStatus::aborted) {
            f.open(folder + "aborted.txt", std::ios_base::trunc);
          }
        }
      }
      if (new_plan_result.status == PlanStatus::failed) {
        break;
      }
    }
  }
  return best_plan;
}