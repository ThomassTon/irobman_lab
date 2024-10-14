#pragma once 
#include <KOMO/komo.h>
#include <PlanningSubroutines/ConfigurationProblem.h>
#include "../utils/plan.h"

RobotTaskPoseMap
compute_pick_and_place_positions_collaboration(rai::Configuration &C,
                                 const std::vector<std::string> &robots,
                                 const uint n = 6) {
  RobotTaskPoseMap rtpm;

    for (uint i = 0; i < n; ++i) {
        setActive(C, robots);

        const auto home = C.getJointState();

        OptOptions options;
        options.stopIters = 100;
        options.damping = 1e-3;

        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        komo.setDiscreteOpt(2);

        komo.add_collision(true, .01, 1e1);
        komo.add_jointLimits(true, 0., 1e1);

        auto pen_tip_0 = STRING(robots[0] << "pen_tip");
        auto pen_tip_1 = STRING(robots[1] << "pen_tip");

        auto obj = STRING("obj" << i + 1);
        auto goal = STRING("goal" << i + 1);

        Skeleton S = {
            {1., 1., SY_touch, {pen_tip_0, obj}},
            {1., 1., SY_touch, {pen_tip_1, obj}},
            {1., 2., SY_stable, {pen_tip_0, obj}},
            {2., 2., SY_poseEq, {obj, goal}},
        };

        komo.setSkeleton(S);

        komo.addObjective({1.,1.}, FS_distance, {obj, STRING(robots[0] << "pen_tip")}, OT_ineq, {1e1},{-0.0}); 
        komo.addObjective({1.,1.}, FS_distance, {obj, STRING(robots[1] << "pen_tip")}, OT_ineq, {1e1},{-0.0}); 

        ConfigurationProblem cp(komo.world);
        setActive(cp.C, robots);

        for (uint i = 0; i < 100; ++i) {
        komo.run_prepare(0.0, true);
        komo.run(options);

        const arr q0 = komo.getPath()[0]();
        const arr q1 = komo.getPath()[1]();

        // ensure via sampling as well
        const bool res1 = cp.query(q0)->isFeasible;
        const bool res2 = cp.query(q1)->isFeasible;
        cp.C.setJointState(q0);
        if (res1 && res2 && komo.getReport(false).get<double>("ineq") < 1. &&
            komo.getReport(false).get<double>("eq") < 1.) {
            rtpm[robots[0]].push_back({q0.sub(0,6), q1.sub(0,6)});
            rtpm[robots[1]].push_back({q0.sub(7,13), q1.sub(7,13)});
            break;
        } else {
            std::cout << "failed for a bit" << std::endl;
            cp.C.setJointState(home);
        }
        }
    }

  return rtpm;
}