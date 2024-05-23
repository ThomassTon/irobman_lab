#pragma once
#include "../utils/plan.h"
#include "math.h"
#include <random>

TaskSequence generate_random_sequence(const std::vector<Robot> &robots,
                                      const uint num_tasks) {
  TaskSequence seq;

  for (uint i = 0; i < num_tasks; ++i) {
    // sample robot
    const uint r = rand() % robots.size();

    // make pair
    seq.push_back(std::make_pair(robots[r], i));
  }

  auto rng = std::default_random_engine{
      std::chrono::system_clock::now().time_since_epoch().count()};
  std::shuffle(std::begin(seq), std::end(seq), rng);

  return seq;
}

TaskSequence generate_single_arm_sequence(const std::vector<Robot> &robots,
                                          const uint num_tasks) {
  // sample robot _once_.
  const uint r = rand() % robots.size();

  TaskSequence seq;
  for (uint i = 0; i < num_tasks; ++i) {
    // make pair
    seq.push_back(std::make_pair(robots[r], i));
  }

  auto rng = std::default_random_engine{
      std::chrono::system_clock::now().time_since_epoch().count()};
  std::shuffle(std::begin(seq), std::end(seq), rng);

  return seq;
}

TaskSequence
generate_alternating_random_sequence(const std::vector<Robot> &robots,
                                     const uint num_tasks,
                                     const RobotTaskPoseMap &rtpm) {
  uint r = rand() % robots.size();

  auto available_tasks = straightPerm(num_tasks);
  TaskSequence seq;
  while (available_tasks.size() > 0) {
    for (uint j = 0; j < 10; ++j) {
      const uint task_index = available_tasks[rand() % available_tasks.size()];

      // check if the task is feasible with the chosen robot
      if (rtpm.at(robots[r])[task_index].size() != 0) {
        seq.push_back(std::make_pair(robots[r], task_index));

        available_tasks.erase(std::remove(available_tasks.begin(),
                                          available_tasks.end(), task_index),
                              available_tasks.end());

        break;
      }
    }
    r = (r + 1) % robots.size();
  }

  return seq;
}

TaskSequence generate_alternating_greedy_sequence(
    const std::vector<Robot> &robots, const uint num_tasks,
    const RobotTaskPoseMap &rtpm, const std::map<Robot, arr> &home_poses) {
  std::cout << "Generating alternating greedy" << std::endl;
  auto available_tasks = straightPerm(num_tasks);

  // sample starting_ robot.
  uint r = rand() % robots.size();
  std::map<Robot, arr> poses = home_poses;

  TaskSequence seq;
  while (available_tasks.size() > 0) {
    // find minimum dist pose to current robot
    auto min_dist = 1e6;
    uint task_index = 0;
    bool assigned_task = false;
    for (auto t : available_tasks) {
      if (rtpm.at(robots[r])[t].size() != 0) {
        const auto dist = absMax(poses[robots[r]] - rtpm.at(robots[r])[t][0]);
        if (dist < min_dist) {
          task_index = t;
          min_dist = dist;
          assigned_task = true;
        }
      }
    }

    if (assigned_task) {
      // remove task from available tasks
      available_tasks.erase(std::remove(available_tasks.begin(),
                                        available_tasks.end(), task_index),
                            available_tasks.end());

      // make pair
      std::cout << "adding " << robots[r] << " with index " << r << std::endl;
      seq.push_back(std::make_pair(robots[r], task_index));
    }

    r = (r + 1) % robots.size();
  }

  return seq;
}

TaskSequence swap_robot(const TaskSequence &seq,
                        const std::vector<Robot> &robots) {
  const uint task_index = rand() % seq.size();
  while (true) {
    const uint r = rand() % robots.size();

    TaskSequence seq_new = seq;
    if (seq_new[task_index].first != robots[r]) {
      seq_new[task_index].first = robots[r];
      return seq_new;
    }
  }
}

TaskSequence swap_tasks(const TaskSequence &seq) {
  while (true) {
    const uint t1_index = rand() % seq.size();
    const uint t2_index = rand() % seq.size();

    if (t1_index != t2_index) {
      TaskSequence seq_new = seq;
      auto tmp = seq_new[t1_index];
      seq_new[t1_index] = seq_new[t2_index];
      seq_new[t2_index] = tmp;

      return seq_new;
    }
  }
}

TaskSequence reverse_subtour(const TaskSequence &seq) {
  uint start = rand() % seq.size();
  uint end = rand() % seq.size();

  while (start == end) {
    start = rand() % seq.size();
    end = rand() % seq.size();
  }

  if (start > end) {
    std::swap(start, end);
  }

  TaskSequence seq_new = seq;
  for (uint i = 0; i <= end - start; ++i) {
    seq_new[start + i] = seq[end - i];
  }

  return seq_new;
}

TaskSequence neighbour(const TaskSequence &seq,
                       const std::vector<Robot> &robots) {
  arr rnd(1);
  rndUniform(rnd, 0, 1);

  if (rnd(0) < 1. / 3.) {
    std::cout << "Swapping robots" << std::endl;
    return swap_robot(seq, robots);
  } else if (rnd(0) < 2. / 3.) {
    std::cout << "Swapping tasks" << std::endl;
    return swap_tasks(seq);
  } else {
    std::cout << "Reversing subtour" << std::endl;
    return reverse_subtour(seq);
  }
}

bool sequence_is_feasible(const TaskSequence &seq,
                          const RobotTaskPoseMap &rtpm) {
  for (const auto &s : seq) {
    const Robot r = s.first;
    const auto task_index = s.second;

    if (rtpm.at(r)[task_index].size() == 0) {
      return false;
    }
  }

  return true;
}

void get_plan_from_cache() {}

double compute_lb_for_sequence(const TaskSequence &seq,
                               const RobotTaskPoseMap &rtpm,
                               const std::map<Robot, arr> &start_poses,
                               const uint start_index = 0,
                               const std::map<Robot, double> start_times = {}) {
  // the lower bound can be computed by finding the minimum time
  // of the current task, and using the precedence constraints as well.
  std::map<Robot, double> robot_time = start_times;
  std::map<Robot, arr> robot_pos = start_poses;

  for (uint i = start_index; i < seq.size(); ++i) {
    const auto task_tuple = seq[i];
    const auto robot = task_tuple.first;
    const auto task_index = task_tuple.second;

    // std::cout << robot << std::endl;

    if (!robot_time.count(robot)) {
      robot_time[robot] = 0.;
    }

    std::cout << robot << " " << task_index << std::endl;

    const arr start_pose = robot_pos[robot];
    const arr goal_pose = rtpm.at(robot)[task_index][0];

    // const arr dist = goal_pose - start_pose;
    // std::cout << goal_pose - start_pose << std::endl;
    const double max_dist = absMax(goal_pose - start_pose);
    const double max_acc = 0.1;
    const double time_to_accelerate = VMAX / max_acc;
    const double acc_dist = 0.5 * VMAX * VMAX / max_acc * 2;

    double dt = 0;
    if (acc_dist > max_dist) {
      // this is wrong
      std::cout << "using acc. timing only" << std::endl;
      dt = 2 * time_to_accelerate;
    } else {
      std::cout << "using acc. timing and max-vel" << std::endl;
      dt = (max_dist - acc_dist) / VMAX + 2 * time_to_accelerate;
    }

    robot_time[robot] += dt;

    for (const auto &rt : robot_time) {
      if (robot_time[robot] < rt.second) {
        robot_time[robot] = rt.second;
      }
    }

    robot_pos[robot] = goal_pose;
  }

  double max = 0;
  for (const auto rt : robot_time) {
    if (max < rt.second) {
      max = rt.second;
    }
  }

  return max;
}