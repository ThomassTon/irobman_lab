#pragma once 
#include "plan_in_animation.h"
#include "../utils/plan.h"

FrameL get_robot_frames(rai::Configuration &C, const Robot &robot) {
  FrameL frames;

  const rai::String base STRING(robot << "base");
  FrameL roots = C.getFrames(C.getFrameIDs({base}));
  for (auto *f : roots) {
    f->getSubtree(frames);
  }
  frames.append(roots);

  return frames;
}

PlanResult plan_multiple_arms_given_subsequence_and_prev_plan(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const uint start_index, const Plan prev_paths,
    const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {
  rai::Configuration CPlanner = C;
  // C.watch(true);

  // prepare planning-configuration
  delete_unnecessary_frames(CPlanner);

  // CPlanner.watch(true);

  std::map<Robot, FrameL> robot_frames;
  for (auto r : home_poses) {
    const auto robot = r.first;
    robot_frames[robot] = get_robot_frames(CPlanner, robot);
  }

  // remove things from paths
  std::vector<uint> unplanned_tasks;
  for (uint i = start_index; i < sequence.size(); ++i) {
    unplanned_tasks.push_back(sequence[i].second);
  }

  std::map<Robot, std::vector<TaskPart>> paths;

  for (const auto p : prev_paths) {
    const auto r = p.first;
    for (auto plan : p.second) {
      if (std::find(unplanned_tasks.begin(), unplanned_tasks.end(),
                    plan.task_index) == unplanned_tasks.end()) {
        paths[r].push_back(plan);
        std::cout << "adding (" << r << " " << plan.task_index << ")"
                  << std::endl;
      }
    }
  }

  // figure out which robots need an exit path
  std::vector<std::pair<Robot, uint>> robot_exit_paths;
  for (const auto p : paths) {
    const auto robot = p.first;
    // do not plan an exit path if
    // -- ther is already one
    // -- there is no other path
    // -- we are planning for this robot next
    if (p.second.size() > 0 && !paths[robot].back().is_exit) {
      robot_exit_paths.push_back({robot, paths[robot].back().t(-1)});
    }
  }

  std::sort(robot_exit_paths.begin(), robot_exit_paths.end(),
            [](auto &left, auto &right) { return left.second < right.second; });

  for (const auto p : robot_exit_paths) {
    const auto robot = p.first;
    // do not plan an exit path if
    // -- ther is already one
    // -- there is no other path
    // -- we are planning for this robot next

    std::cout << "adding exit path for " << robot << std::endl;
    // plan exit path for robot
    rai::Animation A;
    for (const auto &p2 : paths) {
      for (const auto path2 : p2.second) {
        A.A.append(path2.anim);
      }
    }

    std::cout << "start time:" << p.second << std::endl;
    setActive(CPlanner, robot);
    auto exit_path =
        plan_in_animation(A, CPlanner, p.second, paths[robot].back().path[-1],
                          home_poses.at(robot), p.second + 5, robot, true);
    exit_path.r = robot;
    exit_path.task_index = paths[robot].back().task_index;
    exit_path.is_exit = true;
    exit_path.name = "exit";

    /*for (uint i=0; i<exit_path.t.N; ++i){
      std::cout << exit_path.path[i] << std::endl;
      CPlanner.setJointState(exit_path.path[i]);
      CPlanner.watch(true);
    }*/

    if (exit_path.has_solution) {
      std::cout << "adding path for " << robot << " at time " << p.second
                << std::endl;
      const auto exit_anim_part = make_animation_part(
          CPlanner, exit_path.path, robot_frames[robot], p.second);
      exit_path.anim = exit_anim_part;
      paths[robot].push_back(exit_path);
    } else {
      std::cout << "Was not able to find an exit path" << std::endl;
      return PlanResult(PlanStatus::failed);
    }
    /*{
      rai::Animation An;
      for (const auto &p2 : paths) {
        for (const auto path2 : p2.second) {
          An.A.append(path2.anim);
        }
      }
      An.play(CPlanner, false);
    }*/
  }

  // actually plan
  for (uint i = start_index; i < sequence.size(); ++i) {
    uint prev_finishing_time = 0;

    for (const auto &p : paths) {
      const auto plans = p.second;
      if (plans.size() > 0) {
        if (plans.back().is_exit) {
          // if a path is an exit, we are looking at the maximum non-exit time
          prev_finishing_time = std::max(
              {uint(plans[plans.size() - 2].t(-1)), prev_finishing_time});
        } else {
          // else, we are looking at the final current time
          prev_finishing_time = std::max(
              {uint(plans[plans.size() - 1].t(-1)), prev_finishing_time});
        }
      }
    }

    /*
    const auto res = plan_task(CPlanner, sequence[i], rtpm, robot_frames,
    best_makespan_so_far, home_poses, prev_finishing_time, paths);

    if (res != PlanStatus::success){
      return PlanResult(res);
    }*/

    // set robots to home pose
    for (const auto &r : home_poses) {
      setActive(CPlanner, r.first);
      CPlanner.setJointState(r.second);
    }

    // plan for current goal
    const Robot robot = sequence[i].first;
    const uint task = sequence[i].second;
    std::cout << "planning task " << task << " for robot " << robot << " as "
              << i << " th task" << std::endl;

    bool is_bin_picking = false;
    if (rtpm.at(robot)[task].size() > 1) {
      is_bin_picking = true;
    }

    // remove exit path
    bool removed_exit_path = false;
    const uint max_start_time_shift = 35 * rtpm.at(robot)[task].size();
    if (paths[robot].size() > 0 && paths[robot].back().is_exit &&
        prev_finishing_time <
            paths[robot].back().t(-1) + 1 + max_start_time_shift) {
#if 0
      // partial removal of the exit path
      const uint max_time_diff = max_start_time_shift;
      if (prev_finishing_time > max_time_diff && prev_finishing_time - paths[robot].back().t(0) + 1 > max_time_diff){
        uint del_index = 0;
        for (uint i=0; i<paths[robot].back().t.N; ++i){
          if (paths[robot].back().t(i) == prev_finishing_time - max_time_diff){
            del_index = i;
            break;
          }
        }
        std::cout << del_index << std::endl;
        if (del_index > 0){
          std::cout << "deleting parts of the prev. exit path" << std::endl;
          std::cout << paths[robot].back().t << std::endl;

          const uint n = paths[robot].back().t.N-del_index;
          {
            arr tmp;
            tmp.resize(del_index,paths[robot].back().anim.X.d1, 7);
            for (uint i=0; i<del_index; ++i){
              tmp[i] = paths[robot].back().anim.X[i];
            }
            paths[robot].back().anim.X = tmp;
          }
          {
            arr tmp;
            tmp.resize(del_index);
            for (uint i=0; i<del_index; ++i){
              tmp(i) = paths[robot].back().t(i);
            }
            paths[robot].back().t = tmp;
          }
          {
            arr tmp;
            tmp.resize(del_index, paths[robot].back().path.d1);
            for (uint i=0; i<del_index; ++i){
              tmp[i] = paths[robot].back().path[i];
            }
            paths[robot].back().path = tmp;
          }

          std::cout << paths[robot].back().t << std::endl;
        }
      }
      else{
        std::cout << "removing exit path of " << robot << std::endl;
        std::cout << "exit path end time: " << paths[robot].back().t(-1) << std::endl;
        paths[robot].pop_back();
      }

      removed_exit_path = true;
#else
      std::cout << "removing exit path of " << robot << std::endl;
      std::cout << "exit path end time: " << paths[robot].back().t(-1)
                << std::endl;
      paths[robot].pop_back();

      removed_exit_path = true;
#endif
    }

    for (uint j = 0; j < rtpm.at(robot)[task].size(); ++j) {
      arr start_pose;
      uint start_time;

      if (paths[robot].size() > 0) {
        start_pose = paths[robot].back().path[-1];
        start_time = paths[robot].back().t(-1);
      } else {
        start_pose = home_poses.at(robot);
        start_time = 0;

        if (prev_finishing_time > max_start_time_shift + 1) {
          start_time = prev_finishing_time - max_start_time_shift + 1;
        }
      }

      std::cout << "start time " << start_time << std::endl;

      if (!removed_exit_path &&
          prev_finishing_time > start_time + max_start_time_shift + 1) {
        start_time = prev_finishing_time - max_start_time_shift + 1;
      }

      const arr goal_pose = rtpm.at(robot)[task][j];
      const uint time_lb = std::max(
          {(j == rtpm.at(robot)[task].size() - 1) ? prev_finishing_time : 0,
           start_time});

      std::cout << "prev finishing time " << prev_finishing_time << std::endl;
      std::cout << "new start time " << start_time << std::endl;
      std::cout << "lower bound time " << time_lb << std::endl;

      // make animation from path-parts
      rai::Animation A;
      for (const auto &p : paths) {
        for (const auto path : p.second) {
          A.A.append(path.anim);
        }
      }

      if (false) {
        for (uint i = 0; i < A.getT(); ++i) {
          A.setToTime(CPlanner, i);
          CPlanner.watch(false);
          rai::wait(0.1);
        }
      }

      // set configuration to plannable for current robot
      std::cout << "setting up C" << std::endl;
      setActive(CPlanner, robot);

      auto path = plan_in_animation(A, CPlanner, start_time, start_pose,
                                    goal_pose, time_lb, robot, false);

      path.r = robot;
      path.task_index = task;
      path.name = "task";

      if (path.has_solution) {
        if (false) {
          for (uint i = 0; i < path.path.d0; ++i) {
            auto q = path.path[i];
            CPlanner.setJointState(q);
            CPlanner.watch(false);
            rai::wait(0.01);
          }
          CPlanner.setJointState(home_poses.at(robot));
        }
        // make animation part
        auto tmp_frames = robot_frames[robot];
        // add obj. frame to the anim-part.
        if (is_bin_picking) {
          const auto obj = STRING("obj" << task + 1);
          auto to = CPlanner[obj];
          tmp_frames.append(to);
        }
        const auto anim_part =
            make_animation_part(CPlanner, path.path, tmp_frames, start_time);
        path.anim = anim_part;

        if (path.t(-1) > best_makespan_so_far) {
          std::cout << "Stopping early due to better prev. path. ("
                    << best_makespan_so_far << ")" << std::endl;
          return PlanResult(PlanStatus::aborted);
        }

        paths[robot].push_back(path);
      } else {
        std::cout << "Was not able to find a path" << std::endl;
        return PlanResult(PlanStatus::failed);
      }

      // re-link things if we are doing bin-picking
      if (is_bin_picking) {
        CPlanner.setJointState(path.path[-1]);
        // CPlanner.watch(true);
        const auto pen_tip = STRING(robot << "pen_tip");
        const auto obj = STRING("obj" << task + 1);

        if (j == 0) {
          auto from = CPlanner[pen_tip];
          auto to = CPlanner[obj];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }

        if (j == 1) {
          auto to = CPlanner[obj];
          auto from = CPlanner["table_base"];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }

        // CPlanner.watch(true);
      }
    }

    std::cout << "planning exit path" << std::endl;

    const uint exit_start_time = paths[robot].back().t(-1);
    const arr exit_path_start_pose = paths[robot].back().path[-1];

    rai::Animation A;
    for (const auto &p : paths) {
      for (const auto path : p.second) {
        A.A.append(path.anim);
      }
    }

    auto exit_path =
        plan_in_animation(A, CPlanner, exit_start_time, exit_path_start_pose,
                          home_poses.at(robot), exit_start_time, robot, true);
    exit_path.r = robot;
    exit_path.task_index = task;
    exit_path.is_exit = true;
    exit_path.name = "exit";

    if (exit_path.has_solution) {
      const auto exit_anim_part = make_animation_part(
          CPlanner, exit_path.path, robot_frames[robot], exit_start_time);
      exit_path.anim = exit_anim_part;
      paths[robot].push_back(exit_path);
    } else {
      std::cout << "Was not able to find an exit path" << std::endl;
      return PlanResult(PlanStatus::failed);
    }
  }

  if (false) {
    rai::Animation A;
    for (const auto &p : paths) {
      for (const auto path : p.second) {
        A.A.append(path.anim);
      }
    }

    for (uint i = 0; i < A.getT(); ++i) {
      A.setToTime(CPlanner, i);
      CPlanner.watch(false);
      rai::wait(0.1);
    }
  }

  return PlanResult(PlanStatus::success, paths);
}

// overload (not in the literal or in the c++ sense) of the above
PlanResult plan_multiple_arms_given_sequence(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {

  Plan paths;
  return plan_multiple_arms_given_subsequence_and_prev_plan(
      C, rtpm, sequence, 0, paths, home_poses, best_makespan_so_far);
}

/*PlanStatus plan_task(rai::Configuration &CPlanner, const robot_task_pair &rtp,
    const RobotTaskPoseMap &rtpm,
    const std::map<Robot, FrameL> robot_frames, const uint best_makespan_so_far,
    const std::map<Robot, arr> &home_poses, const uint prev_finishing_time, Plan
&paths){
  // set robots to home pose
  for (const auto &r : home_poses) {
    setActive(CPlanner, r.first);
    CPlanner.setJointState(r.second);
  }

  // plan for current goal
  const Robot robot = rtp.first;
  const uint task = rtp.second;

  // remove exit path
  if (paths[robot].size() > 0 &&
      prev_finishing_time < paths[robot].back().t(-1) + 1 + 25) {
    paths[robot].pop_back();
  }

  arr start_pose;
  uint start_time;

  if (paths[robot].size() > 0) {
    start_pose = paths[robot].back().path[-1];
    start_time = paths[robot].back().t(-1) + 1;
  } else {
    start_pose = home_poses.at(robot);
    start_time = 0;
  }

  std::cout << "start time " << start_time << std::endl;

  const arr goal_pose = rtpm.at(robot)[task][0];

  const uint time_lb = std::max({prev_finishing_time, start_time});

  std::cout << "lb " << time_lb << std::endl;

  if (time_lb > start_time && time_lb - start_time > 25) {
    start_time = time_lb - 25;
  }

  // make animation from path-parts
  rai::Animation A;
  for (const auto &p : paths) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  if (false) {
    for (uint i = 0; i < prev_finishing_time; ++i) {
      A.setToTime(CPlanner, i);
      CPlanner.watch(false);
      rai::wait(0.1);
    }
  }

  // set configuration to plannable for current robot
  std::cout << "setting up C" << std::endl;
  setActive(CPlanner, robot);

  auto path = plan_in_animation(A, CPlanner, start_time, start_pose,
                                goal_pose, time_lb, robot, true);

  path.r = robot;
  path.task_index = task;
  path.name = "task";

  if (path.has_solution) {
    if (false) {
      for (uint i = 0; i < path.path.d0; ++i) {
        auto q = path.path[i];
        CPlanner.setJointState(q);
        CPlanner.watch(false);
        rai::wait(0.01);
      }
      CPlanner.setJointState(home_poses.at(robot));
    }
    // make animation part
    const auto anim_part = make_animation_part(
        CPlanner, path.path, robot_frames.at(robot), start_time);
    path.anim = anim_part;

    if (prev_finishing_time > best_makespan_so_far) {
      std::cout << "Stopping early due to better prev. path. ("
                << best_makespan_so_far << ")" << std::endl;
      return PlanStatus::aborted;
    }

    paths[robot].push_back(path);
  } else {
    std::cout << "Was not able to find a path" << std::endl;
    return PlanStatus::failed;
  }

  const uint exit_start_time = path.t(-1) + 1;

  auto exit_path =
      plan_in_animation(A, CPlanner, exit_start_time, goal_pose,
                        home_poses.at(robot), exit_start_time, robot, true);
  exit_path.r = robot;
  exit_path.task_index = task;
  exit_path.is_exit = true;
  exit_path.name = "exit";

  if (exit_path.has_solution) {
    const auto exit_anim_part = make_animation_part(
        CPlanner, exit_path.path, robot_frames.at(robot), exit_start_time);
    exit_path.anim = exit_anim_part;
    paths[robot].push_back(exit_path);
  } else {
    std::cout << "Was not able to find an exit path" << std::endl;
    return PlanStatus::failed;
  }

  return PlanStatus::success;
}*/