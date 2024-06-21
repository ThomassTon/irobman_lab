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
  // Skeleton S = {
  //       {1., 1., SY_touch, {pen_tip, waypoint}},
  //       // {1., 1., SY_stable, {pen_tip, waypoint}},
  // //       // {2., 2., SY_poseEq, {obj, goal}},
  //   };

    // komo.setSkeleton(S);

    // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
    // OT_eq,
    //                  {1e2}, point);
    // komo.addObjective({1., 1.}, FS_distance, {STRING(prefix << "pen_tip"),
    // STRING(obj << i + 1)}, OT_eq, {1e1});

    // komo.addObjective({1.}, FS_vectorZ, {STRING(robot << "pen")}, OT_sos,
                      // {1e1}, {0., 0., -1.});
    // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
    // OT_sos, {1e0}, C[obj]->getPosition());

    // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
    // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
    // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});
    // ConfigurationProblem cp(komo.world);
    // setActive(cp.C, robot);
  komo.addObjective({1.},FS_positionDiff,{STRING(robot << "pen_tip"), "waypoint"},OT_eq,{1e1});
  // komo.addObjective({1.},FS_positionDiff,{STRING(robot << "pen_tip"), "waypoint"},OT_eq,{1e1})
  // komo.addObjective({1.},FS_position,{STRING(robot << "pen_tip"), "waypoint"},OT_eq,{}, pos);

          // addObjective({s.phase0-.1,s.phase0+.1}, FS_position, {s.frames(0)}, OT_eq, {}, {0.,0.,.1}, 2);

  // komo.addObjective({1.}, FS_vectorZ, {STRING(robot << "pen")}, OT_sos,
                    // {1e1}, {0., 0., -1.});

  ConfigurationProblem cp(komo.world);
  cp.computeCollisions = true;
  // cp.C.feature(FS_accumulatedCollisions,{});
  setActive(cp.C, robot);
  arr q;
  double min_cost=MAXFLOAT;
  for (uint i = 0; i < 20; ++i) {
    komo.run_prepare(0.0, true);
    komo.run(options);
    komo.optimize();
    // const double cost = komo.getCosts();

    const arr q0 = komo.getPath()[0]();
    // ensure via sampling as well
    const bool res1 = cp.query(q0)->isFeasible;
    const uintA collision  = cp.query(q0)->collisions();
    
    // const auto collision2 =   cp.C.getCollisionExcludePairIDs();

    // const bool res2 = cp.query(q1)->isFeasible;
    if (res1 && komo.getReport(false).get<double>("ineq") < 1. &&
        komo.getReport(false).get<double>("eq") < 1. &&collision.N==0) {
        // if (cost<min_cost)
        // {
        //   min_cost = cost;
        //   q = q0;
        // }
        
      // std::cout<<" collision:  "<<collision<<"\n";
      // rtpm[robot].push_back({q0, q1});
      // std::cout<<"q0 :"<<q0<<"   q1: "<<q1<<"\n\n\n\n";

      return q0;
      break;
    } else {
      std::cout << "failed for a bit" << std::endl;
    }
  }
    
  return q;


}

auto get_r_0_1(const auto& r0_b, const auto &rotationmatrix, const auto &r1_b){
  arr rb;
  arr ra;
  // // rotationmax[0] * r1_b;
  for(int i=0; i<3;i++){  // rotationmatrix .T
    double ele=0;
    for(int j=0;j<3;j++){
      ele +=rotationmatrix[j](i)*r1_b(j);
      // std::cout<<rotationmatrix[j](i)<<"\n\n\n\n";
    }
    rb.append(ele);
  }

  for(int i=0; i<3;i++){  // rotationmatrix .T
    double ele=0;
    for(int j=0;j<3;j++){
      ele +=rotationmatrix[j](i)*r0_b(j);
      // std::cout<<rotationmatrix[j](i)<<"\n\n\n\n";
    }
    ra.append(ele);
  } 
  // std::cout<<"rotation matrix"<<rotationmatrix<<"\n\n\n\n";
  // std::cout<<"rob: "<<r0_b<<"\n\n\n\n\n";
  // std::cout<<"r1b: "<<r1_b<<"\n\n\n\n\n";
  // std::cout<<"rb: "<<-ra+rb<<"\n\n\n\n\n";

  return -ra+rb;
}

auto get_trans_position(const auto& r0_b, const auto &rotationmatrix, const auto &r0_1){
  arr ra;
  for(int i=0; i<3;i++){  // rotationmatrix .T
    double ele=0;
    for(int j=0;j<3;j++){
      ele +=rotationmatrix[i](j)*r0_1(j);
      // std::cout<<rotationmatrix[i](j)<<"  "<<r0_1(j)<<"\n\n\n\n";
    }
    ra.append(ele);
  }
  // std::cout<<"rotation matrix"<<rotationmatrix<<"\n\n\n\n";
  // std::cout<<"ra: "<<ra<<"\n\n\n\n\n";
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
  for (auto r : home_poses) {
    const auto robot = r.first;
    robot_frames[robot] = get_robot_frames(CPlanner, robot);
  }
  std::map<Robot, std::vector<TaskPart>> paths;

  uint run_time_array[sequence.size()][2];   // save the first start time ;

  // actually plan
  const Robot _robot = sequence[0].first;
  const uint _task = sequence[0].second;
  arr rotationmantix;
  arr r0_b;
  arr r0_1;
  // r0_1.append(0.2, -0.1 ,0.0);

  for (uint j = 0; j < rtpm.at(_robot)[_task].size(); ++j) {
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

    

    for (uint i = start_index; i < sequence.size(); ++i) {
      if(i==0){
      // for (const auto &r : home_poses) {
      //   setActive(CPlanner, r.first);
      //   CPlanner.setJointState(r.second);
      // }
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

      if (paths[robot].size() > 0) {
        
        
        start_pose = paths[robot].back().path[-1];
        start_time = paths[robot].back().t(-1);
        // let two arm start at the same timepunkt
        // uint max_last_run_time = std::max(paths[sequence[0].first].back().t(-1), paths[sequence[1].first].back().t(-1));  
        uint max_last_run_time = std::max(run_time_array[0][j-1], run_time_array[1][j-1]);  
        start_time = start_time <  max_last_run_time ?  max_last_run_time: start_time;
        start_time +=3;
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
      std::cout<<"goal pose: "<<goal_pose<<"\n\n\n\n\n\n\n\n\n";
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

      
      auto path = plan_in_animation(A, CPlanner, start_time, start_pose,goal_pose, time_lb, robot, false);
        //  std::cout<<"path_1"<<path.path<<"\n";
   
      // std::cout<<"goal pose: "<<goal_pose<<"\n\n\n\n\n\n";
      if(j==1&&i==1){
        // for(auto r0b :paths[sequence[0].first].back().path){

        // }
        // for (auto it : paths[sequence[0].first].back().path) {
        //   std::cout << it << "\n ";
        // }

        // auto r0b = paths[sequence[0].first].back().path[-1];
        uint size_of_path =  paths[sequence[0].first].back().path.N /7;
        // std::cout<<"robot a0 size: "<<size_of_path<<"\n\n";
        arr t_a1;
        arr path_a1(0u,path.path.d1);
            // arr p(0u, path.d1);
        CPlanner.setJointState(paths[robot][0].path[-1]);
        for(uint i = 0; i<size_of_path; i++){
          auto r0b = paths[sequence[0].first][1].path[i];
          auto t = paths[sequence[0].first].back().t(i);
          CTest.setJointState(r0b);
          const auto pen_tip =  STRING(sequence[0].first << "pen_tip");
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

        // path = plan_in_animation(A, CPlanner, start_time, start_pose,goal_pose_, time_lb, robot, false);
        // std::cout<<"path_a1"<<path_a1<<"\n";
        // std::cout<<"path_a1_"<<path.path<<"\n";
        // std::cout<<"path_a1_t"<<path.t<<"\n";

        TaskPart path_(t_a1,path_a1);
        path_.has_solution=true;
        path = path_;



      }
      // std::cout<<"path"<<path.path<<"\n\n\n\n\n\n";
      


      path.r = robot;
      path.task_index = task;
      path.name = "task";

      if (path.has_solution) {

        // make animation part
        auto tmp_frames = robot_frames[robot];
        // add obj. frame to the anim-part.
        if (is_bin_picking && i==0) {
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
        run_time_array[i][j] = paths[robot].back().t(-1);
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
          // std::cout<<"get the pose from pen_tip"<<pose(0)<<"\n\n\n\n\n\n\n";
          // std::cout<<"get the rotation from pen_tip"<<rota[0]<<"\n\n\n\n\n\n\n";
          // std::cout<<"get the path0 from pen_tip"<<path.path[0]<<"\n\n\n\n\n\n\n";
          // std::cout<<"get the path from pen_tip"<<path.path<<"\n\n\n\n\n\n\n";
          // get_position(CPlanner,robot,pose);
          if(i==0){
            to->unLink();

              // create a new joint
            to->linkFrom(from, true);
            r0_b =  CPlanner[pen_tip]->getPosition();
            rotationmantix = CPlanner[pen_tip]->getRotationMatrix();
            std::cout<<"penpos:   "<<rotationmantix<<"\n\n\n";

            //  r0_b = CPlanner[obj]->getPosition();
            // rotationmantix = CPlanner[obj]->getRotationMatrix();
            // std::cout<<"boxpos:   "<<rotationmantix<<"\n\n\n";
          }
          else{
            r0_1 = get_r_0_1(r0_b,rotationmantix, CPlanner[pen_tip]->getPosition());
            
            // r0_1 = arr(0.2,-0.1,0.0);
            // r0_1(0)=0.2;
            // r0_1(1) = -0.1;
            // r0_1(2) = -0.05;
            // std::cout<<get_trans_position(r0_b, rotationmantix, r0_1);
          }
          // to->unLink();

          // create a new joint
          // to->linkFrom(from, true);
        }

        if (j == 1) {
          auto to = CPlanner[obj];
          auto from = CPlanner["table_base"];
          if(i==0){
            to->unLink();

              // create a new joint
            to->linkFrom(from, true);

            std::cout<<"path size2:::: "<<path.path.N<<"\n\n\n";
          }
          // to->unLink();

          // create a new joint
          // to->linkFrom(from, true);
        }

        // CPlanner.watch(true);
      }
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
