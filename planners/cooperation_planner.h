#pragma once 
#include "plan_in_animation.h"
#include "../utils/plan.h"
#include "prioritized_planner.h"
#include <KOMO/komo.h>
#include <PlanningSubroutines/ConfigurationProblem.h>
auto get_position(rai::Configuration &C, const std::string &robot, const arr &pos) {

  C.addFrame(STRING("waypoint"));
  arr size;
  size.append(0.1);
  C.getFrame(STRING("waypoint"))->setShape(rai::ShapeType::ST_marker,size);
  C.getFrame(STRING("waypoint"))->setPosition(pos);

  setActive(C, robot);
  const auto home = C.getJointState();
  // C.getFrameNames();
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  KOMO komo;
  komo.verbose = 0;
  komo.setModel(C, true);

  komo.setDiscreteOpt(1);

  // komo.world.stepSwift();

  komo.add_collision(true, .01, 1e1);
  komo.add_jointLimits(true, 0., 1e1);

  auto pen_tip = STRING(robot << "pen_tip");
  auto waypoint = STRING("waypoint");

  komo.addObjective({1.},FS_positionDiff,{STRING(robot << "pen_tip"), "waypoint"},OT_eq,{1e1});

  ConfigurationProblem cp(komo.world);
  cp.computeCollisions = true;
  setActive(cp.C, robot);
  arr q;
  double min_cost=MAXFLOAT;
  for (uint i = 0; i < 20; ++i) {
    komo.run_prepare(0.0, true);
    komo.run(options);
    komo.optimize();
    const arr q0 = komo.getPath()[0]();
    // ensure via sampling as well
    const bool res1 = cp.query(q0)->isFeasible;
    const uintA collision  = cp.query(q0)->collisions();
    if (res1 && komo.getReport(false).get<double>("ineq") < 1. &&
        komo.getReport(false).get<double>("eq") < 1. &&collision.N==0) {

      return q0;
      break;
    } else {
      std::cout << "failed for a bit" << std::endl;
    }
  }
    
  return q;


}

auto get_joints_from_waypoints(rai::Configuration &C, const std::string &robot, const arr waypoints) {
  std::cout << "waypoints len"<<waypoints.d0 << "\n\n\n\n\n\n" << std::endl;

  for(uint i=0; i<waypoints.d0;i++){
    auto waypoint = STRING("waypoint"<<i);
    C.addFrame(waypoint);
    arr size;
    size.append(0.1);
    C.getFrame(waypoint)->setShape(rai::ShapeType::ST_marker,size);
    C.getFrame(waypoint)->setPosition(waypoints[i]);
  }


  setActive(C, robot);
  const auto home = C.getJointState();
  // C.getFrameNames();
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;
  
  KOMO komo;
  komo.verbose = 0;
  komo.setModel(C, true);
  komo.setDiscreteOpt(waypoints.d0);

  for(uint i=0; i<waypoints.d0;i++){
    auto waypoint = STRING("waypoint"<<i);
    komo.addObjective({double(i+1)},FS_positionDiff,{STRING(robot << "pen_tip"), waypoint},OT_eq,{1e1});
  }
  std::cout << "done komo path" << std::endl;
  arr q;
  komo.run_prepare(0.0, true);
  komo.run(options);
  komo.optimize();
  uint len_path = komo.getPath_q().d0;
  std::cout<<"len:  "<<len_path<<"!!!!\n\n\n\n";
  arr path(waypoints.d0, 7);
  for (uint j = 0; j < len_path; ++j) {
    path[j] = komo.getPath_q(j);
  }
  std::cout<<path;
  return path;
}

auto get_r_0_1(const auto& r0_b, const auto &rotationmatrix, const auto &r1_b){
  arr rb;
  arr ra;
  for(int i=0; i<3;i++){  // rotationmatrix .T
    double ele=0;
    for(int j=0;j<3;j++){
      ele +=rotationmatrix[j](i)*r1_b(j);
    }
    rb.append(ele);
  }

  for(int i=0; i<3;i++){  // rotationmatrix .T
    double ele=0;
    for(int j=0;j<3;j++){
      ele +=rotationmatrix[j](i)*r0_b(j);
    }
    ra.append(ele);
  } 
  return -ra+rb;
}

auto get_trans_position(const auto& r0_b, const auto &rotationmatrix, const auto &r0_1){
  arr ra;
  for(int i=0; i<3;i++){  // rotationmatrix .T
    double ele=0;
    for(int j=0;j<3;j++){
      ele +=rotationmatrix[i](j)*r0_1(j);
    }
    ra.append(ele);
  }
  return ra+r0_b;
}


PlanResult plan_cooperation_arms_given_subsequence_and_prev_plan(
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
      // actually plan
      const Robot _robot = sequence[0].first;
      const uint _task = sequence[0].second;
      arr rotationmantix;
      arr r0_b;
      arr r0_1;

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

          // let two arms start at the same timepunkt
          if (paths[robot].size() > 0) {
            start_pose = paths[robot].back().path[-1];
            start_time = paths[robot].back().t(-1);
            if(robot=="a0_"){
              uint max_last_run_time = std::max(paths["a0_"].back().t(-1), paths["a1_"].back().t(-1));  
              start_time = start_time <  max_last_run_time ?  max_last_run_time: start_time;
            }  
            
            start_time +=1;
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
          setActive(CTest,sequence[0].first);

          TaskPart path;
      
          if(j==1&&robot=="a1_"){                                     //compute waypoints fot robot2
            uint size_of_path =  paths["a0_"].back().path.N /7;
            arr t_a1;
            CPlanner.setJointState(paths[robot].back().path[-1]);
            arr waypoints(0u,3);
            for(uint i = 0; i<size_of_path; i++){
              auto r0b = paths["a0_"].back().path[i];
              auto t = paths["a0_"].back().t(i);
              CTest.setJointState(r0b);
              const auto pen_tip =  STRING("a0_" << "pen_tip");
              auto _r0_b = CTest[pen_tip]->getPosition();
              auto rotationmatrix = CTest[pen_tip]->getRotationMatrix();
              auto _goal_pose = get_trans_position(_r0_b,rotationmatrix,r0_1);
              waypoints.append(_goal_pose);
              t_a1.append(t);
            }
            auto path_a1 = get_joints_from_waypoints(CPlanner,robot, waypoints);
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
            if (is_bin_picking&&robot=="a0_") {
              const auto obj = STRING("obj" << task+1);
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
                to->linkFrom(from, true);
                r0_b =  CPlanner[pen_tip]->getPosition();
                rotationmantix = CPlanner[pen_tip]->getRotationMatrix();
              }
              else{
                r0_1 = get_r_0_1(r0_b,rotationmantix, CPlanner[pen_tip]->getPosition());
              }
            }
            else if (j == 1) {
              if(robot=="a0_" ){  // link to previous obj (or bottom)
                auto to = CPlanner[obj];
                auto from = CPlanner["table_base"];
                to->unLink();
                to->linkFrom(from, true);
              }
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

 
PlanResult plan_cooperation_arms_given_sequence(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {
  Plan paths;
  return plan_cooperation_arms_given_subsequence_and_prev_plan(
      C, rtpm, sequence, 0, paths, home_poses, best_makespan_so_far);
}
