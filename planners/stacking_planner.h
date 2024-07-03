#pragma once 
#include "plan_in_animation.h"
#include "../utils/plan.h"
#include "prioritized_planner.h"
#include <KOMO/komo.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

PlanResult plan_stacking_arms_given_subsequence_and_prev_plan(
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
      // int i=0;
      for (auto r : home_poses) {
        const auto robot = r.first;
        robot_frames[robot] = get_robot_frames(CPlanner, robot);
        // i++;
      }

        // remove things from paths
      std::vector<uint> unplanned_tasks;
      for (uint i = start_index; i < sequence.size(); ++i) {
        unplanned_tasks.push_back(sequence[i].second);
      }

      std::map<Robot, std::vector<TaskPart>> paths;

      // actually plan

      std::cout<<"sequence_size :"<<sequence.size()<<"\n";
      // return PR;
      const Robot robot = sequence[0].first;
      for (uint i = start_index; i < sequence.size(); ++i) {
        uint prev_finishing_time = 0;



        // set robots to home pose
        for (const auto &r : home_poses) {
          setActive(CPlanner, r.first);
          CPlanner.setJointState(r.second);
        }


        // plan for current goal
        
        const uint task = sequence[i].second;
        std::cout<<"task: "<<task<<"\n\n";
        std::cout << "planning task " << task << " for robot " << robot << " as "
                  << i << " th task" << std::endl;


        bool is_bin_picking = false;
        if (rtpm.at(robot)[task].size() > 1) {
          is_bin_picking = true;
        }

        // remove exit path
        bool removed_exit_path = false;
        const uint max_start_time_shift = 35 * rtpm.at(robot)[task].size();


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


          // if (false) {
          //   for (uint i = 0; i < A.getT(); ++i) {
          //     A.setToTime(CPlanner, i);
          //     CPlanner.watch(false);
          //     rai::wait(0.1);
          //   }
          // }

          // set configuration to plannable for current robot
          std::cout << "setting up C" << std::endl;
          setActive(CPlanner, robot);

          auto path = plan_in_animation(A, CPlanner, start_time, start_pose,
                                        goal_pose, time_lb, robot, false);

          path.r = robot;
          path.task_index = task;
          path.name = "task";

          if (path.has_solution) {
            // if (false) {
            //   for (uint i = 0; i < path.path.d0; ++i) {
            //     auto q = path.path[i];
            //     CPlanner.setJointState(q);
            //     CPlanner.watch(false);
            //     rai::wait(0.01);
            //   }
            //   CPlanner.setJointState(home_poses.at(robot));
            // }
            // make animation part
            auto tmp_frames = robot_frames[robot];
            // add obj. frame to the anim-part.
            if (is_bin_picking) {
              const auto obj = STRING("obj" << task + 1);
              auto to = CPlanner[obj];
              tmp_frames.append(to);
              if(task==1){
                const auto obj = STRING("obj" << task);
                auto to = CPlanner[obj];
                tmp_frames.append(to);
              }

            }
            const auto anim_part =
                make_animation_part(CPlanner, path.path, tmp_frames, start_time);
            path.anim = anim_part;

            // if (path.t(-1) > best_makespan_so_far) {
            //   std::cout << "Stopping early due to better prev. path. ("
            //             << best_makespan_so_far << ")" << std::endl;
            //   return PlanResult(PlanStatus::aborted);
            // }

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
              if(task==1){
                auto to2 = CPlanner["obj1"];
                // auto from2 = CPlanner["obj1"];
                to2->unLink();

                to2->linkFrom(to,true);

                std::cout<<"obj2: child:  "<<to->children<<"  "<<to2<<"\n\n\n\n";

              }
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


      }
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
        exit_path.task_index = sequence[sequence.size()-1].second;
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

    


  return PlanResult(PlanStatus::success, paths);
}


PlanResult plan_stacking_sequence(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {

  Plan paths;
  return plan_stacking_arms_given_subsequence_and_prev_plan(
      C, rtpm, sequence, 0, paths, home_poses, best_makespan_so_far);
}
