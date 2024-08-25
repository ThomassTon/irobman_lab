#include <KOMO/komo.h>
#include <Kin/F_operators.h>
#include <Kin/F_pose.h>
#include <Kin/F_qFeatures.h>
#include <Kin/featureSymbols.h>
#include <Kin/kin.h>

#include <Kin/kinViewer.h>
#include <Kin/viewer.h>

#include <Manip/rrt-time.h>
#include <PlanningSubroutines/ConfigurationProblem.h>
#include <PlanningSubroutines/Animation.h>

#include <iomanip>
#include <numeric>

#include <algorithm>
#include <chrono>
#include <random>

#include <math.h>

#include <GL/gl.h>
#include <Gui/opengl.h>

#include "samplers/pick_and_place_sampler.h"
#include "samplers/stippling_sampler.h"
#include "samplers/pick_and_place_sampler_collaboration.h"

#include "planners/plan_in_animation.h"
#include "planners/prioritized_planner.h"

#include "searchers/sequencing.h"
#include "searchers/annealing_searcher.h"
#include "searchers/random_searcher.h"
#include "searchers/greedy_random_searcher.h"
#include "searchers/unsync_searcher.h"

#include "utils/plan.h"
#include "utils/util.h"
#include "utils/env_util.h"
#include "utils/path_util.h"

// TODO:
// - fix loading and visualization of previously computed paths
// - time-rescale path
// - split main planning subroutine
// - squeaky wheel planner
// - enable things that are not only 'go to point', e.g. drawing a line
// - enable multi-arm cooperation
// - enable search over sequences with precendence constraints
// - look into more complex motion planning:
// -- joint optimization
// -- constrained sampling based planning

int main(int argc, char **argv) {
  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  rnd.seed(seed);



  const uint verbosity = rai::getParameter<double>(
      "verbosity", 1); // verbosity, does not do anything atm

  const bool save_video = rai::getParameter<bool>("save_video", false);

  const bool plan_pick_and_place =
      rai::getParameter<bool>("pnp", true); // pick and place yes/no

  const bool plan_pick_and_place_single_arm =
    rai::getParameter<bool>("pnps",false); // pick and place yes/no

  const bool plan_pick_and_place_cooperation =
    rai::getParameter<bool>("pnpc",false); 

  const rai::String mode =
      rai::getParameter<rai::String>("mode", "stacking"); // test, greedy_random_search, show_plan
  const rai::String stippling_scenario =
      rai::getParameter<rai::String>("stippling_pts", "lis_default"); // lis_default, four_by_four_grid, default_grid

  const rai::String env =
      rai::getParameter<rai::String>("env", "lab"); // environment

  std::vector<std::string> robots; // string-prefix for robots

  rai::Configuration C;
  if(plan_pick_and_place_single_arm){
    pick_and_place_single_arm(C, mode);

    robots = {"a0_"};
  }
  else if(plan_pick_and_place_cooperation){
    pick_and_place_cooperation(C,mode);

    robots = {"a0_","a1_"};   
  }
  else if (plan_pick_and_place) {
    pick_and_place(C);
    robots = {"a0_", "a1_"};
  } else {
    if (env == "lab") {
      labSetting(C);
      robots = {"a0_", "a1_"};
    } else {
      more_robots(C, 4);
      robots = {"a0_", "a1_", "a2_", "a3_"};
    }
  }

  // maps [robot] to home_pose
  const std::map<Robot, arr> home_poses = get_robot_home_poses(C, robots);

  C.watch(true);
  // show prev path
  if (mode == "show_plan") {
    load_and_viz(C, plan_pick_and_place);
    return 0;
  }

  // stippling
  RobotTaskPoseMap robot_task_pose_mapping;
  if (!plan_pick_and_place) {
    const arr pts = get_scenario(stippling_scenario);
    if (pts.N == 0) {
      return 0;
    }

    if (verbosity > 0) {
      drawPts(C, pts);
    }

    // maps [robot] to [index, pose]
    std::cout << "Computing stippling poses" << std::endl;
    robot_task_pose_mapping = compute_stippling_poses_for_arms(C, pts, robots);
  } else {
    // bin picking
    std::cout << "Computing pick and place poses" << std::endl;
    if(mode =="stacking_collaboration")
    {
      robot_task_pose_mapping = compute_pick_and_place_positions_collaboration(C, robots,2);// change box number
    }
    else if(mode =="stacking_singlearm"){
      robot_task_pose_mapping = compute_pick_and_place_positions(C, robots,2);
    }
    else if(mode =="single_arm"){
      robot_task_pose_mapping = compute_pick_and_place_positions(C, robots,5);
    }
    else if(mode =="collaboration_single_obj"||mode=="collaboration_single_obj_obstacle"||mode =="collaboration_single_obj_vertical"){
      robot_task_pose_mapping = compute_pick_and_place_positions_collaboration(C, robots,1);
    }
    
    
  }

  // initial test
  // bool save_video = false;

  if(mode=="single_arm"){
    const auto plan = plan_single_arm_unsynchronized(C, robot_task_pose_mapping, home_poses);

    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/singlearm");
  }
  else if(mode =="stacking_singlearm"){
    const auto plan = plan_single_arm_stacking(C, robot_task_pose_mapping, home_poses);

    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan_stacking(C, plan, save_video, "video/bin_picking/stacking");
  }
  else if(mode =="stacking_collaboration"){
    const auto plan = plan_cooperation_arm_stacking(C, robot_task_pose_mapping, home_poses);

    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan_stacking(C, plan, save_video, "video/bin_picking/stacking");
  }

  else if(mode =="collaboration_single_obj"){
    const auto plan = plan_cooperation_arm_unsynchronized(C, robot_task_pose_mapping, home_poses);

    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/cooperation");
  }
  else if(mode =="collaboration_single_obj_obstacle"){
    const auto plan = plan_cooperation_arm_unsynchronized(C, robot_task_pose_mapping, home_poses);

    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/cooperation");
  }

  else if(mode =="collaboration_single_obj_vertical"){
    const auto plan = plan_cooperation_arm_unsynchronized(C, robot_task_pose_mapping, home_poses);

    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/cooperation");
  }

  else if (mode == "test") {
    const auto plan = plan_multiple_arms_unsynchronized(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;

    visualize_plan(C, plan, save_video, "video/bin_picking/unsync");
  } else if (mode == "random_search") {
    const auto plan = plan_multiple_arms_random_search(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/random_search");
  } else if (mode == "greedy_random_search") {
    const auto plan = plan_multiple_arms_greedy_random_search(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/greedy_search");
  } else if (mode == "simulated_annealing") {
    const auto plan = plan_multiple_arms_simulated_annealing(   
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/annealing");
  }

  return 0;
}