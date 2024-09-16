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
      uint num_task = sequence.size();
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

          const auto obj1 = STRING("obj" <<  1);
          auto to_obj1 = CPlanner[obj1];
          const auto obj2 = STRING("obj" <<  2);
          auto to_obj2 = CPlanner[obj2];

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
              for(uint i=task+1; i>0;i--){
                const auto obj = STRING("obj" << i);
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
              to->linkFrom(from, true);
              }

            else{
              if(task<(num_task-1)){  // link to previous obj (or bottom)
                auto to = CPlanner[obj];
                uint pre_index = task>0 ? task:num_task;   // The last object is at the bottom
                const auto pre_obj = STRING("obj" << 2);
                auto from = CPlanner[pre_obj];
                to->unLink();
                  // create a new joint
                to->linkFrom(from, true);
              }
              else if(task==(num_task-1)){
                auto to = CPlanner[obj];
                uint pre_index = task>0 ? task:num_task;   // The last object is at the bottom
                const auto last_obj = STRING("obj" << pre_index);
                auto from = CPlanner["table_base"];
                to->unLink();

                  // create a new joint
                to->linkFrom(from, true);
              }
              // to->unLink();

              // create a new joint
              // to->linkFrom(from, true);
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




PlanResult plan_stacking_arms_collaboration_given_subsequence_and_prev_plan(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const uint start_index, const Plan prev_paths,
    const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {
    rai::Configuration CPlanner = C;

    rai::Configuration CTest = C;

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

      // return PR;
      const Robot robot_ = sequence[0].first;
      uint task_ = sequence[0].second;
      bool is_bin_picking = true;
      arr rotationmantix;
      arr r0_b;
      arr r0_1;
      uint num_task = sequence.size()/2;
      for(uint k = 0; k< num_task; ++k){
          // std::cout<<"k: "<<k<<"\n";
          // uint run_time_array[sequence.size()][2];   // save the first start time ;

        // actually plan
        const Robot _robot = sequence[0].first;
        const uint _task = sequence[0].second;
        arr rotationmantix;
        arr r0_b;
        arr r0_1;
        // r0_1.append(0.2, -0.1 ,0.0);

        for (uint j = 0; j < rtpm.at(_robot)[_task].size(); ++j) {
          for (const auto &r : home_poses) {
            setActive(CPlanner, r.first);
            CPlanner.setJointState(r.second);
          }
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
        
          for (uint i = k*2; i < k*2+2; ++i) {
            // if(i==0){
            for (const auto &r : home_poses) {
              setActive(CPlanner, r.first);
              CPlanner.setJointState(r.second);
            }
            
            // std::cout<<"i: "<<i<<"\n";
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



            arr start_pose;
            uint start_time;

            if (paths[robot].size() > 0) {
              
              
              start_pose = paths[robot].back().path[-1];
              start_time = paths[robot].back().t(-1);
              // let two arm start at the same timepunkt
              // uint max_last_run_time = std::max(paths[sequence[0].first].back().t(-1), paths[sequence[1].first].back().t(-1));  
              // uint max_last_run_time = std::max(run_time_array[0][j-1], run_time_array[1][j-1]);
              // if(robot=="a0_"){
              //   uint max_last_run_time = std::max(paths["a0_"].back().t(-1), paths["a1_"].back().t(-1));  

              //   start_time = start_time <  max_last_run_time ?  max_last_run_time: start_time;
              // }  
              
              // start_time +=3;
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
            
            // if(i>0){
            //   start_time = start_time_array[i-1][j];
            // }
            // start_time_array[i][j]=start_time;
              
            
            
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
            setActive(CTest,sequence[0].first);

            TaskPart path;
              //  std::cout<<"path_1"<<path.path<<"\n";
        
            // std::cout<<"goal pose: "<<goal_pose<<"\n\n\n\n\n\n";
            if(j==1&&robot=="a1_"){
              // for(auto r0b :paths[sequence[0].first].back().path){

              // }
              // for (auto it : paths[sequence[0].first].back().path) {
              //   std::cout << it << "\n ";
              // }

              // auto r0b = paths[sequence[0].first].back().path[-1];
              uint size_of_path =  paths["a0_"].back().path.N /7;
              // std::cout<<"robot a0 size: "<<size_of_path<<"\n\n";
              arr t_a1;
              arr path_a1(0u,paths["a0_"].back().path.d1);
                  // arr p(0u, path.d1);
              CPlanner.setJointState(paths[robot].back().path[-1]);
              for(uint i = 0; i<size_of_path; i++){
                auto r0b = paths["a0_"].back().path[i];
                auto t = paths["a0_"].back().t(i);
                CTest.setJointState(r0b);
                const auto pen_tip =  STRING("a0_" << "pen_tip");
                auto _r0_b = CTest[pen_tip]->getPosition();
                // std::cout<<"box pose air:"<<_r0_b<<"\n\n\n\n\n\n";
                auto rotationmatrix = CTest[pen_tip]->getRotationMatrix();
                auto _goal_pose = get_trans_position(_r0_b,rotationmatrix,r0_1);
                auto goal_pose_= get_position(CPlanner,robot,_goal_pose);
                CPlanner.setJointState(goal_pose_);
                t_a1.append(t);
                path_a1.append(goal_pose_);
                // Ta
                // std::cout<<"path size"<<path.path.sizeT<<"\n\n\n\n\n\n";
              }
              TaskPart path_(t_a1,path_a1);
              path_.has_solution=true;
              path = path_;
            }
            else{
              path = plan_in_animation(A, CPlanner, start_time, start_pose,goal_pose, time_lb, robot, false);

            }
            path.r = robot;
            path.task_index = task;
            path.name = "task";

            if (path.has_solution) {

              // make animation part
              auto tmp_frames = robot_frames[robot];
              // add obj. frame to the anim-part.
              if (is_bin_picking&&robot=="a0_") {
                // const auto obj = STRING("obj" << task + 1);
                // auto to = CPlanner[obj];
                // tmp_frames.append(to);
                // if(task==1){
                //   const auto obj = STRING("obj" << task);
                //   auto to = CPlanner[obj];
                //   tmp_frames.append(to);
                // }
                for(uint i=task+1; i>0;i--){
                  const auto obj = STRING("obj" << i);
                  auto to = CPlanner[obj];
                  tmp_frames.append(to);
                }
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
              auto pose = CPlanner[pen_tip]->getPosition();
              auto rota = CPlanner[pen_tip]->getRotationMatrix();

              if (j == 0) {
                auto from = CPlanner[pen_tip];
                auto to = CPlanner[obj];

                if(robot=="a0_"){
                  to->unLink();

                    // create a new joint
                  to->linkFrom(from, true);
                  r0_b =  CPlanner[pen_tip]->getPosition();
                  rotationmantix = CPlanner[pen_tip]->getRotationMatrix();

                }
                else{
                  r0_1 = get_r_0_1(r0_b,rotationmantix, CPlanner[pen_tip]->getPosition());

                }

              }

              else if (j == 1) {
                if(robot=="a0_" && task<(num_task-1)){  // link to previous obj (or bottom)
                  auto to = CPlanner[obj];
                  uint pre_index = task>0 ? task:num_task;   // The last object is at the bottom
                  const auto pre_obj = STRING("obj" << pre_index);
                  auto from = CPlanner[pre_obj];
                  to->unLink();
                    // create a new joint
                  to->linkFrom(from, true);
                }
                // if(robot=="a0_" && task==0){  // link to previous obj (or bottom)
                //   auto to = CPlanner[obj];
                //   uint pre_index = task>0 ? task:num_task;   // The last object is at the bottom
                //   const auto pre_obj = STRING("obj" << 3);
                //   auto from = CPlanner["table_base"];
                //   to->unLink();
                //     // create a new joint
                //   to->linkFrom(from, true);
                // }
                // if(robot=="a0_" && task==1){  // link to previous obj (or bottom)
                //   auto to = CPlanner[obj];
                //   uint pre_index = task>0 ? task:num_task;   // The last object is at the bottom
                //   const auto pre_obj = STRING("obj" << 1);
                //   auto from = CPlanner["table_base"];
                //   to->unLink();
                //     // create a new joint
                //   to->linkFrom(from, true);
                // }
                else if(robot=="a0_" && task==(num_task-1)){
                  auto to = CPlanner[obj];
                  uint pre_index = task>0 ? task:num_task;   // The last object is at the bottom
                  const auto last_obj = STRING("obj" << pre_index);
                  auto from = CPlanner["table_base"];
                  to->unLink();

                    // create a new joint
                  to->linkFrom(from, true);
                }
                // to->unLink();

                // create a new joint
                // to->linkFrom(from, true);
              }

              // CPlanner.watch(true);
            }
          }


        }

      }

      for(uint i=0;i<2;i++){
        const auto robot = sequence[i].first;
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

PlanResult plan_stacking_sequence_collaboration(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {

  Plan paths;
  return plan_stacking_arms_collaboration_given_subsequence_and_prev_plan(
      C, rtpm, sequence, 0, paths, home_poses, best_makespan_so_far);
}
