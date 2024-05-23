#pragma once 
#include <KOMO/komo.h>
#include <PlanningSubroutines/ConfigurationProblem.h>
#include "../utils/plan.h"

arr sampleConfigurationForRobot(KOMO &komo, const arr &point,
                                const std::string &prefix, bool rnd = false,
                                bool ineq = false) {
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  komo.setDiscreteOpt(1);

  // komo.world.stepSwift();

  komo.add_collision(true, .01, 1e1);
  komo.add_jointLimits(true, 0., 1e1);

  komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")}, OT_eq,
                    {1e2}, point);

  if (!ineq) {
    komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
                      {1e1}, {0., 0., -1.});
  } else {
    komo.addObjective({1.}, FS_scalarProductZZ,
                      {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1},
                      {-cos(15 * 3.1415 / 180.)});
  }
  // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
  // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
  // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});
  ConfigurationProblem cp(komo.world);
  setActive(cp.C, prefix);

  for (uint i = 0; i < 10; ++i) {
    if (rnd) {
      komo.run_prepare(0.1, false);
    } else {
      komo.run_prepare(0.0, true);
    }
    komo.run(options);

    const arr q = komo.getPath()[0]();
    // komo.pathConfig.watch(true);

    // ensure via sampling as well
    const bool res = cp.query(q)->isFeasible;

    if (res && komo.getReport(false).get<double>("ineq") < 1. &&
        // komo.getReport(false).get<double>("sos") < 0.5 &&
        komo.getReport(false).get<double>("eq") < 1.) {
      // komo.pathConfig.watch(true);
      return q;
    }
  }

  std::cout << "failed for pt " << point << std::endl;
  return {};
}

arr sampleConfigurationForRobot(rai::Configuration &C, const arr &point,
                                const std::string &prefix) {
  // activate agents
  setActive(C, prefix);

  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  KOMO komo;

  komo.verbose = 0;

  // set up komo problem
  komo.setModel(C, true);

  return sampleConfigurationForRobot(komo, point, prefix);
}

std::vector<arr> computeConfigurationsForPoints(const arr &pts,
                                                rai::Configuration &C,
                                                const std::string prefix) {
  std::vector<arr> configurations;

  const auto start = std::chrono::high_resolution_clock::now();

  // activate agents
  setActive(C, prefix);

  const auto home = C.getJointState();

  KOMO komo;
  komo.verbose = 0;
  komo.setModel(C, true);

  for (uint i = 0; i < pts.d0; ++i) {
    const arr pt = {pts[i](0), pts[i](1), 0.075};
    const arr q = sampleConfigurationForRobot(
        komo, C["table"]->getPosition() + pt, prefix);
    // const arr q = sampleConfigurationForRobot(C, C["table"]->getPosition() +
    // pt, prefix);
    configurations.push_back(q);

    // C.setJointState(q);
    //
    komo.clearObjectives();
    komo.world.setJointState(home);
  }
  C.setJointState(home);

  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of IK pose computation: " << duration.count() / 1000.
            << " ms (per point: " << duration.count() / 1000. / pts.d0 << "ms )"
            << std::endl;

  return configurations;
}

RobotTaskPoseMap
compute_stippling_poses_for_arms(rai::Configuration &C, const arr &pts,
                                 const std::vector<Robot> &robots) {
  RobotTaskPoseMap rtpm;
  for (const Robot &r : robots) {
    const TaskPoses poses = computeConfigurationsForPoints(pts, C, r);
    std::vector<TaskPoses> tp;
    for (const arr &p : poses) {
      if (p.N > 0) {
        tp.push_back({p});
      } else {
        tp.push_back({});
      }
    }
    rtpm[r] = tp;
  }

  return rtpm;
}