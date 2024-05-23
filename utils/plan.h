#pragma once

#include "spdlog/spdlog.h"

#include <string>
#include <vector>
#include <map>
#include <PlanningSubroutines/Animation.h>

#include <Core/array.h>

#include "util.h"
#include "env_util.h"

typedef std::string Robot;
typedef std::vector<arr> TaskPoses;
typedef std::map<std::string, std::vector<TaskPoses>> RobotTaskPoseMap;
typedef std::pair<Robot, int> robot_task_pair;
typedef std::vector<robot_task_pair> TaskSequence;

const double VMAX = 0.05;

// this is the solution of one task
struct TaskPart {
  bool has_solution = false;

  TaskPart(const arr &_t, const arr &_path)
      : has_solution(true), t(_t), path(_path){};
  TaskPart(){};

  rai::Animation::AnimationPart anim;

  arr t;
  arr path;

  Robot r;
  uint task_index;

  std::string name;
  std::string algorithm;

  bool is_exit = false;
};

typedef std::map<Robot, std::vector<TaskPart>> Plan;

enum class PlanStatus { failed, aborted, success, unplanned };

struct PlanResult {
  PlanResult() : status(PlanStatus::unplanned) {}
  PlanResult(PlanStatus _status) : status(_status){};
  PlanResult(PlanStatus _status, const Plan &_plan)
      : status(_status), plan(_plan){};

  PlanStatus status;
  Plan plan;
};

double
get_makespan_from_plan(const std::map<Robot, std::vector<TaskPart>> &plan) {
  double max_time = 0.;
  for (const auto &robot_plan : plan) {
    const auto last_subpath = robot_plan.second.back();
    max_time = std::max({last_subpath.t(-1), max_time});
  }

  return max_time;
}

// TODO
std::map<Robot, std::vector<TaskPart>> reoptimize_plan(const std::map<Robot, std::vector<TaskPart>> &unscaled_plan){
  std::map<Robot, std::vector<TaskPart>> optimized_plan;
  return optimized_plan;
}

void visualize_plan(rai::Configuration C, const Plan &plan,
                    const bool save = false, 
                    const char* save_video_path = "video/") {
  rai::Animation A;
  for (const auto &p : plan) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  rai::ConfigurationViewer Vf;
  // Vf.setConfiguration(C, "\"Real World\"", true);
  Vf.setConfiguration(C, "\"Real World\"", false);

  const double makespan = get_makespan_from_plan(plan);

  if (save) {
      rai::system(STRING("mkdir -p " <<save_video_path));
  }

  for (uint t = 0; t < makespan; ++t) {
    // A.setToTime(C, t);

    // std::cout << t << std::endl;
    for (const auto tp : plan) {
      const auto r = tp.first;
      const auto parts = tp.second;

      bool done = false;
      for (auto part : parts) {
        // std::cout <<part.t(0) << " " << part.t(-1) << std::endl;
        if (part.t(0) > t || part.t(-1) < t) {
          continue;
        }

        for (uint i = 0; i < part.t.N - 1; ++i) {
          if (part.t(i) <= t && part.t(i + 1) > t) {
            setActive(C, r);
            C.setJointState(part.path[i]);
            // std::cout <<part.path[i] << std::endl;
            done = true;

            // set bin picking things
            const auto task_index = part.task_index;
            const auto obj_name = STRING("obj" << task_index + 1);

            if (part.anim.frameNames.contains(obj_name)) {
              const auto pose =
                  part.anim.X[uint(std::floor(t - part.anim.start))];
              arr tmp(1, 7);
              tmp[0] = pose[-1];
              C.setFrameState(tmp, {C[obj_name]});
            }
            break;
          }
        }

        if (done) {
          break;
        }
      }
    }

    // C.watch(false);
    Vf.setConfiguration(C, ".", false);
    rai::wait(0.01);

    if (save) {
      Vf.savePng(save_video_path);
    }
  }
}

arr get_robot_pose_at_time(const uint t, const Robot r,
                           const std::map<Robot, arr> &home_poses,
                           const std::map<Robot, std::vector<TaskPart>> &plan) {
  if (plan.count(r) > 0) {
    for (const auto &part : plan.at(r)) {
      // std::cout <<part.t(0) << " " << part.t(-1) << std::endl;
      if (part.t(0) > t || part.t(-1) < t) {
        continue;
      }

      for (uint i = 0; i < part.t.N; ++i) {
        if (part.t(i) == t) {
          return part.path[i];
          // std::cout <<part.path[i] << std::endl;
        }
      }
    }
  }

  return home_poses.at(r);
}

void drawPts(rai::Configuration C, arr pts, arr color = {0., 0., 0., 1.}) {
  for (uint i = 0; i < pts.d0; ++i) {
    const arr pt = {pts[i](0), pts[i](1), 0.075};

    auto *dot = C.addFrame("goal", "table");
    dot->setShape(rai::ST_sphere, {0.005});
    dot->setRelativePosition({pts[i](0), pts[i](1), 0.05});
    dot->setContact(0.);
    dot->setColor(color);
  }

  C.gl()->displayCamera().setPosition(0, 0., 3);
  C.gl()->displayCamera().focusOrigin();

  C.watch(true);
}

// overloaded for both
void drawPts(rai::Configuration C, std::map<uint, arr> tmp,
             arr color = {0., 0., 0., 1.}) {
  for (auto element : tmp) {
    uint j = element.first;
    arr pts = element.second;

    for (uint i = 0; i < pts.d0; ++i) {
      const arr pt = {pts[i](0), pts[i](1), 0.075};

      auto *dot = C.addFrame("goal", "table");
      dot->setShape(rai::ST_sphere, {0.01});
      dot->setRelativePosition({pts[i](0), pts[i](1), 0.05});
      dot->setContact(0.);
      dot->setColor({1. * j, 1. * j, 1. * j, 1.});
    }
  }

  C.watch(true);
}

std::map<Robot, arr> get_robot_home_poses(rai::Configuration &C,
                                          const std::vector<Robot> &robots) {
  std::map<Robot, arr> poses;
  for (auto r : robots) {
    setActive(C, r);
    poses[r] = C.getJointState();

    // std::cout << poses[r] << std::endl;
  }

  return poses;
}

void export_plan(const std::vector<Robot> &robots,
                 const std::map<Robot, arr> &home_poses,
                 const std::map<Robot, std::vector<TaskPart>> &plan,
                 const TaskSequence &seq, const std::string base_folder,
                 const uint iteration, const uint computation_time) {
  std::cout << "exporting plan" << std::endl;
  // make folder
  const std::string folder =
      "./out/" + base_folder + "/" + std::to_string(iteration) + "/";
  const int res = system(STRING("mkdir -p " << folder).p);
  (void)res;

  rai::Animation A;
  for (const auto &p : plan) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  // - add info
  // -- comp. time
  {
    std::ofstream f;
    f.open(folder + "comptime.txt", std::ios_base::trunc);
    f << computation_time;
  }

  // -- makespan
  {
    std::ofstream f;
    f.open(folder + "makespan.txt", std::ios_base::trunc);
    f << A.getT();
  }

  // -- sequence
  {
    std::ofstream f;
    f.open(folder + "sequence.txt", std::ios_base::trunc);
    for (const auto &s : seq) {
      f << "(" << s.first << " " << s.second << ")";
    }
  }

  // -- plan
  {
    std::ofstream f;
    f.open(folder + "plan.txt", std::ios_base::trunc);
    for (const auto per_robot_plan : plan) {
      const auto robot = per_robot_plan.first;
      const auto tasks = per_robot_plan.second;

      f << robot << ": ";
      for (const auto task : tasks) {
        f << task.name << "(" << task.algorithm << ")"
          << " " << task.task_index << ", " << task.t(0) << ", " << task.t(-1)
          << "; ";
      }
      f << std::endl;
    }
  }

  // -- actual path
  {
    std::ofstream f;
    f.open(folder + "robot_controls.txt", std::ios_base::trunc);
    arr path(A.getT(), home_poses.at(robots[0]).d0 * robots.size());
    for (uint i = 0; i < A.getT(); ++i) {
      uint offset = 0;
      for (uint j = 0; j < robots.size(); ++j) {
        const arr pose = get_robot_pose_at_time(i, robots[j], home_poses, plan);
        for (uint k = 0; k < pose.N; ++k) {
          path[i](k + offset) = pose(k);
        }
        offset += pose.N;
      }
    }

    f << path;
  }
}

void load_and_viz(rai::Configuration C, const bool pick_and_place) {
  arr path;
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid/greedy_20230328_000824/139/robot_controls.txt";
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_000242/86/robot_controls.txt";
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_103317/1/robot_controls.txt";

  // for bin picking:
  // robot, obj, start, end
  // for points
  // robot end
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230328_211219/29/robot_controls.txt";
  /*std::vector<std::vector<uint>> timings;
  timings.push_back({0, 0, 22, 53});
  timings.push_back({0, 5, 92, 137});
  timings.push_back({0, 3, 151, 214});
  timings.push_back({0, 2, 233, 263});

  timings.push_back({1, 1, 110, 179});
  timings.push_back({1, 4, 218, 277});*/

  /*const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230330_005636/19/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 5, 24, 69});
  timings.push_back({0, 0, 107, 138});
  timings.push_back({0, 3, 159, 220});
  timings.push_back({0, 2, 239, 269});

  timings.push_back({1, 1, 111, 170});
  timings.push_back({1, 4, 221, 269});*/

  /*
  const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_020353/3/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 39});
  timings.push_back({0, 101});
  timings.push_back({0, 184});
  timings.push_back({0, 252});
  timings.push_back({0, 295});

  timings.push_back({1, 56});
  timings.push_back({1, 143});
  timings.push_back({1, 207});
  timings.push_back({1, 260});

  timings.push_back({2, 76});
  timings.push_back({2, 157});
  timings.push_back({2, 221});

  timings.push_back({3, 32});
  timings.push_back({3, 89});
  timings.push_back({3, 172});
  timings.push_back({3, 221});
  */

  // greedy
  /*const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_165530/1/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 25});
  timings.push_back({0, 64});
  timings.push_back({0, 123});
  timings.push_back({0, 214});

  timings.push_back({1, 46});
  timings.push_back({1, 68});
  timings.push_back({1, 142});
  timings.push_back({1, 225});

  timings.push_back({2, 48});
  timings.push_back({2, 97});
  timings.push_back({2, 156});

  timings.push_back({3, 26});
  timings.push_back({3, 65});
  timings.push_back({3, 96});
  timings.push_back({3, 182});
  timings.push_back({3, 235});*/

  // opt
  const std::string filepath =
      "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/"
      "greedy_20230401_010325/17/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 25});
  timings.push_back({0, 52});
  timings.push_back({0, 66});
  timings.push_back({0, 94});
  timings.push_back({0, 105});
  timings.push_back({0, 172});

  timings.push_back({1, 116});
  timings.push_back({1, 171});

  timings.push_back({2, 34});
  timings.push_back({2, 60});
  timings.push_back({2, 141});

  timings.push_back({3, 26});
  timings.push_back({3, 35});
  timings.push_back({3, 53});
  timings.push_back({3, 59});
  timings.push_back({3, 153});
  timings.push_back({3, 194});

  FILE(filepath.c_str()) >> path;

  const std::vector<Robot> robots{"a0_", "a1_", "a2_", "a3_"};
  // const std::vector<Robot> robots{"a0_", "a1_"};

  setActive(C, robots);

  rai::ConfigurationViewer Vf;
  Vf.setConfiguration(C, "\"Real World\"", true);

  for (uint i = 0; i < path.d0; ++i) {
    C.setJointState(path[i]);
    // C.watch(false);
    // rai::wait(0.01);

    if (pick_and_place) {
      for (uint j = 0; j < timings.size(); ++j) {
        const auto pen_tip = STRING("a" << timings[j][0] << "_pen_tip");
        const auto obj = STRING("obj" << timings[j][1] + 1);

        if (i == timings[j][2]) {
          auto from = C[pen_tip];
          auto to = C[obj];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }

        if (i == timings[j][3]) {
          auto to = C[obj];
          auto from = C["table_base"];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }
      }
    } else {
      // draw dots
      for (uint j = 0; j < timings.size(); ++j) {
        const auto pen_tip = STRING("a" << timings[j][0] << "_pen_tip");
        const arr pos = C[pen_tip]->getPosition();

        if (timings[j][1] == i) {
          // add small sphere
          auto *dot = C.addFrame("goal", "table");
          dot->setShape(rai::ST_sphere, {0.01});
          dot->setPosition(pos);
          dot->setContact(0.);

          if (timings[j][0] == 0) {
            dot->setColor({0, 0., 0., 0.5});
          } else if (timings[j][0] == 1) {
            dot->setColor({1., 0., 0., 0.5});
          } else if (timings[j][0] == 2) {
            dot->setColor({1., 0., 1., 0.5});
          } else if (timings[j][0] == 3) {
            dot->setColor({1., 1., 0., 0.5});
          }
        }
      }
    }
    Vf.setConfiguration(C, ".", false);
    rai::wait(0.01);
    Vf.savePng();
  }
}

// TODO
/*std::map<Robot, std::vector<TaskPart>> rescale_plan(const std::map<Robot, std::vector<TaskPart>> &unscaled_plan){

  std::vector<double> scaling_factors;

  // go over all the robots, by assembling the complete animation, extract the position at times,
  // and make up a timing schedule that respects the velocity and acceleration limits.
  rai::Animation A;
  for (const auto &p : paths) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  const uint makespan = 0;
  for (uint t=0; t<makespan; ++t){
    const double per_robot_v = 0;
    const double per_robot_a = 0;

    const double max_v = 0;
    const double max_a = 0;

    const double scaling_factor_at_t = 0;


    scaling_factors.push_back(scaling_factor_at_t);
  }

  std::map<Robot, std::vector<TaskPart>> scaled_plan;
  for (const auto &robot_plan : unscaled_plan) {
    const auto robot = robot_plan.first;
    const auto task_parts = robot_plan.second;

    // resample t
    arr t;
    arr scaled_t;

    // rescale path
    arr path = timed_path.resample(scaled_t);
    
    // rebuild animation
    //rai::Animation::AnimationPart anim;
  }


  return scaled_plan;
}*/