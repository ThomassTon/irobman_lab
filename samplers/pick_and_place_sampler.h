#pragma once 
#include <KOMO/komo.h>
#include <PlanningSubroutines/ConfigurationProblem.h>
#include "../utils/plan.h"

RobotTaskPoseMap
compute_pick_and_place_positions(rai::Configuration &C,
                                 const std::vector<std::string> &robots,
                                 const uint n = 6) {
  RobotTaskPoseMap rtpm;

  for (const auto prefix : robots) {
    for (uint i = 0; i < n; ++i) {
      setActive(C, prefix);

      const auto home = C.getJointState();

      OptOptions options;
      options.stopIters = 100;
      options.damping = 1e-3;

      KOMO komo;
      komo.verbose = 0;
      komo.setModel(C, true);

      komo.setDiscreteOpt(2);

      // komo.world.stepSwift();

      komo.add_collision(true, .01, 1e1);
      komo.add_jointLimits(true, 0., 1e1);

      auto pen_tip = STRING(prefix << "pen_tip");
      auto obj = STRING("obj" << i + 1);
      auto goal = STRING("goal" << i + 1);

      Skeleton S = {
          {1., 1., SY_touch, {pen_tip, obj}},
          {1., 2., SY_stable, {pen_tip, obj}},
          {2., 2., SY_poseEq, {obj, goal}},
      };

      komo.setSkeleton(S);

      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_eq,
      //                  {1e2}, point);
      // komo.addObjective({1., 1.}, FS_distance, {STRING(prefix << "pen_tip"),
      // STRING(obj << i + 1)}, OT_eq, {1e1});

      komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
                        {1e1}, {0., 0., -1.});
      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_sos, {1e0}, C[obj]->getPosition());

      // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
      // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
      // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});
      ConfigurationProblem cp(komo.world);
      setActive(cp.C, prefix);

      for (uint i = 0; i < 10; ++i) {
        komo.run_prepare(0.0, true);
        komo.run(options);

        const arr q0 = komo.getPath()[0]();
        const arr q1 = komo.getPath()[1]();
        // komo.pathConfig.watch(true);

        // ensure via sampling as well
        const bool res1 = cp.query(q0)->isFeasible;
        const bool res2 = cp.query(q1)->isFeasible;

        if (res1 && res2 && komo.getReport(false).get<double>("ineq") < 1. &&
            komo.getReport(false).get<double>("eq") < 1.) {
          rtpm[prefix].push_back({q0, q1});
          break;
        } else {
          std::cout << "failed for a bit" << std::endl;
        }
      }
    }
  }

  return rtpm;
}