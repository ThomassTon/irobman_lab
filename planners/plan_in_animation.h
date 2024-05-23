#pragma once
#include <KOMO/komo.h>
#include "../utils/plan.h"
#include "../utils/path_util.h" // for smoothing and partial_shortcut

arr plan_with_komo_given_horizon(const rai::Animation &A, rai::Configuration &C,
                                 const arr &q0, const arr &q1, const arr &ts,
                                 const std::string prefix, double &ineq,
                                 double &eq) {
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;
  options.stopLineSteps = 5;

  std::cout << "setting up komo" << std::endl;
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., ts.N, 5, 2);
  komo.verbose = 0;
  komo.solver = rai::KS_sparse;

  komo.add_collision(true, .001, 1e1);
  komo.add_qControlObjective({}, 2, 1e1);
  komo.add_qControlObjective({}, 1, 1e1);

  komo.setConfiguration(-2, q0);
  komo.setConfiguration(-1, q0);
  komo.setConfiguration(0, q0);

  // make pen tip go a way from the table
  const double offset = 0.06;
  komo.addObjective({0.1, 0.9}, FS_distance,
                    {"table", STRING(prefix << "pen_tip")}, OT_ineq, {1e1},
                    {-offset});
  // komo.addObjective({0.1, 0.8}, FS_distance,
  //                  {"table", STRING(prefix << "pen_tip")}, OT_sos, {1e1});

  // position
  // komo.addObjective({0}, FS_qItself, {}, OT_eq, {1e1}, q0);
  komo.addObjective({1}, FS_qItself, {}, OT_eq, {1e2}, q1);

  // speed
  // komo.addObjective({0.0, 0.05}, FS_qItself, {}, OT_eq, {1e1}, {},
  //                  1); // slow at beginning
  komo.addObjective({0.95, 1.0}, FS_qItself, {}, OT_eq, {1e1}, {},
                    1); // slow at end

  // acceleration
  // komo.addObjective({0.0, 0.05}, FS_qItself, {}, OT_eq, {1e1}, {},
  //                  2); // slow at beginning
  komo.addObjective({0.95, 1.0}, FS_qItself, {}, OT_eq, {1e1}, {},
                    2); // slow at end

  if (false) {
    const arr v_constr = q0 * 0. + VMAX;
    komo.addObjective({0.0, 1.}, FS_qItself, {}, OT_ineq, {1e0}, {v_constr},
                      1); // slow at beginning
    komo.addObjective({0.0, 1.}, FS_qItself, {}, OT_ineq, {-1e0}, {-v_constr},
                      1); // slow at beginning
  }

  setKomoToAnimation(komo, C, A, ts);

  std::cout << "running komo" << std::endl;
  komo.run_prepare(0.01);
  komo.run(options);
  std::cout << "done komo" << std::endl;

  /*if (komo.getReport(false).get<double>("ineq") > 1. ||
  komo.getReport(false).get<double>("eq") > 1.){ std::cout << "infeasible komo
  sol, ineq: " << komo.getReport(false).get<double>("ineq")
      << " eq. " << komo.getReport(false).get<double>("eq") << std::endl;
    return {};
  }*/

  ineq = komo.getReport(false).get<double>("ineq");
  eq = komo.getReport(false).get<double>("eq");

  // if (eq > 2 || ineq > 2){
  komo.getReport(true);
  // komo.pathConfig.watch(true);
  //}

  // komo.pathConfig.watch(true);

  // check if the path is actually feasible
  arr path(ts.N, q0.N);
  for (uint j = 0; j < ts.N; ++j) {
    path[j] = komo.getPath_q(j);
  }

  if (length(path[0] - q0) > 1e-3) {
    std::cout << length(path[0] - q0) << std::endl;
  }

  if (length(path[-1] - q1) > 1e-3) {
    std::cout << length(path[-1] - q1) << std::endl;
  }

  path[0] = q0;
  path[-1] = q1;

  return path;
}

double get_max_speed(const arr &path) {
  double max_speed = 0;
  for (uint i = 0; i < path.d0 - 1; ++i) {
    max_speed = std::max({max_speed, absMax(path[i] - path[i + 1])});
  }

  return max_speed;
}

double get_earliest_feasible_time(TimedConfigurationProblem &TP, const arr &q,
                                  const uint t_max, const uint t_min) {
  uint t_earliest_feas = t_max;
  while (t_earliest_feas > t_min) {
    const auto res = TP.query(q, t_earliest_feas);
    if (!res->isFeasible) {
      t_earliest_feas += 2;
      break;
    }
    --t_earliest_feas;
  }

  if (t_max == t_earliest_feas + 2) {
    std::cout << "C" << std::endl;
    std::cout << "C" << std::endl;
    std::cout << "C" << std::endl;
  }

  return t_earliest_feas;
}


TaskPart plan_in_animation_komo(const rai::Animation &A, rai::Configuration &C,
                                const uint t0, const arr &q0, const arr &q1,
                                const uint time_lb, const std::string prefix,
                                const int time_ub_prev_found = -1) {
  TimedConfigurationProblem TP(C, A);

  // Check if start q is feasible
  const auto start_res = TP.query(q0, t0);
  if (!start_res->isFeasible && min(start_res->coll_y) < -0.1) {
    std::cout << t0 << std::endl;
    LOG(-1) << "q_start is not feasible! This should not happen ";
    start_res->writeDetails(cout, C);
    std::cout << "colliding by " << min(start_res->coll_y) << std::endl;

    // TP.A.setToTime(TP.C, t0);
    // TP.C.setJointState(q0);
    // TP.C.watch(true);

    return TaskPart();
  }

  const uint dt_max_vel = uint(std::ceil(absMax(q0 - q1) / VMAX));

  // the goal should always be free at the end of the animation, as we always
  // plan an exit path but it can be the case that we are currently planning an
  // exit path, thus we include the others
  const uint t_max_to_check = std::max({time_lb, t0 + dt_max_vel, A.getT()});
  // establish time at which the goal is free, and stays free
  const double t_earliest_feas = get_earliest_feasible_time(
      TP, q1, t_max_to_check, std::max({time_lb, t0 + dt_max_vel}));

  std::cout << "Final time for komo: " << t_earliest_feas
            << " dt: " << t_earliest_feas - t0 << std::endl;

  // the minimum horizon length is mainly given by the earliest possible finish
  // time
  const uint min_horizon_length =
      std::max({5u, dt_max_vel, uint(t_earliest_feas - t0)});
  const uint max_horizon_length =
      std::max({25u, time_lb - t0, (A.getT() > t0) ? A.getT() - t0 : 0u});

  // uint horizon = std::ceil((min_horizon_length + max_horizon_length)/2.);
  uint horizon = std::ceil(min_horizon_length);

  const uint max_komo_run_attempts = 5;
  uint iters = 0;
  while (true) {
    std::cout << "komo horizon: " << horizon << std::endl;
    arr ts(horizon);
    for (uint j = 0; j < horizon; ++j) {
      ts(j) = t0 + j;
    }

    if (time_ub_prev_found > 0 && time_ub_prev_found < ts(-1)) {
      std::cout << "found cheaper path before" << std::endl;
      return TaskPart();
    }

    // ensure that the goal is truly free. Sanity check.
    const auto res = TP.query(q1, t0 + horizon);
    if (!res->isFeasible) {
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      res->writeDetails(cout, C);
      // TP.A.setToTime(TP.C, t0 + horizon);
      // TP.C.setJointState(q1);
      // TP.C.watch(true);

      return TaskPart();
    }

    if (false) {
      TP.A.setToTime(TP.C, t0);
      TP.C.setJointState(q0);
      TP.C.watch(true);

      TP.A.setToTime(TP.C, t0 + horizon);
      TP.C.setJointState(q1);
      TP.C.watch(true);
    }

    // set up komo problem
    double ineq = 0;
    double eq = 0;
    const arr path =
        plan_with_komo_given_horizon(A, C, q0, q1, ts, prefix, ineq, eq);

    if (false) {
      for (uint i = 0; i < ts.N; ++i) {
        TP.A.setToTime(TP.C, ts(i));
        TP.C.setJointState(path[i]);
        TP.C.watch(true);
      }
    }

    if (false) {
      for (uint i = 0; i < ts.N; ++i) {
        const auto res = TP.query(path[i], ts(i));
        if (!res->isFeasible) {
          std::cout << "R " << ts(i) << std::endl;
          TP.A.setToTime(TP.C, ts(i));
          TP.C.setJointState(path[i]);
          TP.C.watch(true);
        }
      }
    }

    // check max. speed
    const double max_speed = get_max_speed(path);
    std::cout << "max speed: " << max_speed << std::endl;

    if (eq > 15 && ineq > 15) {
      return TaskPart();
    }

    if (max_speed < VMAX && eq < 1.5 && ineq < 1.5) {
      std::cout << "done, accepting komo. ineq: " << ineq << " eq. " << eq
                << std::endl;
      return TaskPart(ts, path);
    }

    if (iters >= max_komo_run_attempts) {
      std::cout << "too many reruns. ineq: " << ineq << " eq. " << eq
                << std::endl;
      return TaskPart();
    }

    // const uint num_add_timesteps = std::max({uint((iters+1)*2), uint(req_t -
    // ts.N)}); const uint num_add_timesteps = std::ceil(1. *
    // (max_horizon_length
    // - min_horizon_length) / max_komo_run_attempts);
    const int add_timesteps_for_speed =
        std::max({0, int(std::ceil(horizon * ((max_speed / VMAX) - 1)))});
    const uint num_add_timesteps =
        std::max({(iters + 1) * 3u, uint(add_timesteps_for_speed)});
    std::cout << "adding " << num_add_timesteps << " steps" << std::endl;

    horizon += num_add_timesteps;

    std::cout << "rerunning komo; ineq: " << ineq << " eq: " << eq << std::endl;

    ++iters;
  }
}

TaskPart plan_in_animation_rrt(const rai::Animation &A, rai::Configuration &C,
                               const uint t0, const arr &q0, const arr &q1,
                               const uint time_lb, const std::string prefix,
                               int time_ub_prev_found = -1) {
  TimedConfigurationProblem TP(C, A);
  TP.activeOnly = true;

  // Check if start q is feasible
  const auto start_res = TP.query(q0, t0);
  if (!start_res->isFeasible) {
    std::cout << t0 << std::endl;
    LOG(-1) << "q_start is not feasible! This should not happen ";
    start_res->writeDetails(cout, C);
    std::cout << "colliding by " << min(start_res->coll_y) << std::endl;

    // TP.A.setToTime(TP.C, t0);
    // TP.C.setJointState(q0);
    // TP.C.watch(true);

    return TaskPart();
  }

  PathFinder_RRT_Time planner(TP);
  planner.vmax = VMAX;
  planner.lambda = 0.5;
  // planner.disp = true;
  // planner.optimize = optimize;
  // planner.step_time = 5;
  planner.maxIter = 500;
  planner.goalSampleProbability = 0.9; // 0.9

  const uint dt_max_vel = uint(std::ceil(absMax(q0 - q1) / VMAX));

  // the goal should always be free at the end of the animation, as we always
  // plan an exit path but it can be the case that we are currently planning an
  // exit path, thus we include the others
  const uint t_max_to_check = std::max({time_lb, t0 + dt_max_vel, A.getT()});
  // establish time at which the goal is free, and stays free
  const uint t_earliest_feas = get_earliest_feasible_time(
      TP, q1, t_max_to_check, std::max({time_lb, t0 + dt_max_vel}));

  std::cout << "t_earliest_feas " << t_earliest_feas << std::endl;
  std::cout << "last anim time " << A.getT() << std::endl;

  if (false) {
    TP.A.setToTime(TP.C, A.getT());
    TP.C.setJointState(q1);
    TP.C.watch(true);
  }

  // run once without upper bound
  // auto res_check_feasibility = planner.plan(q0, t0, q1, t_earliest_feas);

  const uint max_delta = 7;
  const uint max_iter = 12;
  TimedPath timedPath({}, {});
  for (uint i = 0; i < max_iter; ++i) {
    const uint time_ub = t_earliest_feas + max_delta * (i);

    std::cout << i << " " << time_ub << std::endl;
    if (time_ub > time_ub_prev_found && time_ub_prev_found > 0) {
      std::cout << "Aborting bc. faster path found" << std::endl;
      break;
    }

    auto res = planner.plan(q0, t0, q1, t_earliest_feas, time_ub);

    if (res.time.N != 0) {
      timedPath = res;
      break;
    }
  }

  if (timedPath.time.N == 0) {
    /*TP.A.setToTime(TP.C, t0);
    TP.C.setJointState(q0);
    TP.C.watch(true);

    TP.A.setToTime(TP.C, t_earliest_feas);
    TP.C.setJointState(q1);
    TP.C.watch(true);

    for (uint i=0; i<A.getT(); ++i){
      std::cout << i << std::endl;
      TP.A.setToTime(TP.C, i);
      TP.C.setJointState(q1);
      TP.C.watch(true);
    }*/
    return TaskPart();
  }

  // resample
  const uint N = std::ceil(timedPath.time(timedPath.time.N - 1) - t0) + 1;
  arr t(N);
  for (uint i = 0; i < t.N; ++i) {
    t(i) = i + t0;
  }

  const arr path = timedPath.resample(t, TP.C);

  // check if resampled path is still fine
  for (uint i = 0; i < t.N; ++i) {
    const auto res = TP.query(path[i], t(i));
    if (!res->isFeasible) {
      LOG(-1) << "resampled path is not feasible! This should not happen.";
      start_res->writeDetails(cout, C);

      // TP.A.setToTime(TP.C, t0);
      // TP.C.setJointState(q0);
      // TP.C.watch(true);

      // return TaskPart();
    }
  }

  // shortcutting
  const arr new_path = partial_shortcut(TP, path, t0);

  // std::cout << new_path[-1] << "\n" << q1 << std::endl;

  // smoothing and imposing bunch of constraints
  std::cout << "smoothing rrt path using komo" << std::endl;
  arr smooth_path = smoothing(A, C, t, new_path, prefix);

  for (uint i = 0; i < smooth_path.d0; ++i) {
    const auto res = TP.query(smooth_path[i], t(i));
    if (!res->isFeasible && res->coll_y.N > 0 && min(res->coll_y) < -0.01) {
      std::cout << "smoothed path infeasible" << std::endl;
      smooth_path = new_path;
      break;
    }
  }

  // const arr smooth_path = new_path;

  const double max_speed = get_max_speed(smooth_path);
  std::cout << "rrt final time " << t(-1) << std::endl;
  std::cout << "rrt max_speed " << max_speed << std::endl;

  return TaskPart(t, smooth_path);
}

// robust 'time-optimal' planning method
TaskPart plan_in_animation(const rai::Animation &A, rai::Configuration &C,
                           const uint t0, const arr &q0, const arr &q1,
                           const uint time_lb, const std::string prefix,
                           const bool exit_path) {
  // run rrt
  TaskPart rrt_path = plan_in_animation_rrt(A, C, t0, q0, q1, time_lb, prefix);
  rrt_path.algorithm = "rrt";

  /*if(rrt_path.has_solution){
    TimedConfigurationProblem TP(C, A);
    for (uint i = 0; i < rrt_path.t.N; ++i) {
      const auto res = TP.query(rrt_path.path[i], rrt_path.t(i));
      if(res->coll_y.N > 0) std::cout <<min(res->coll_y) << std::endl;
      if (!res->isFeasible){
        std::cout << "B" << std::endl;
        std::cout << "B" << std::endl;
        std::cout << "B" << std::endl;
      }
      if (!res->isFeasible && min(res->coll_y) < -0.01) {
        std::cout << "rrt actually infeasible" << std::endl;
        rrt_path.has_solution = false;
        break;
      }
    }
  }*/

  int time_ub = -1;
  if (rrt_path.has_solution) {
    time_ub = rrt_path.t(-1);
  }

  // attempt komo
  TaskPart komo_path =
      plan_in_animation_komo(A, C, t0, q0, q1, time_lb, prefix, time_ub);
  komo_path.algorithm = "komo";

  /*if(komo_path.has_solution){
    return komo_path;
  }*/

  // ensure that the komo-path does not run into start configurations of future
  // tasks
  if (komo_path.has_solution) {
    TimedConfigurationProblem TP(C, A);
    for (uint i = 0; i < komo_path.t.N; ++i) {
      const auto res = TP.query(komo_path.path[i], komo_path.t(i));
      if (res->coll_y.N > 0)
        std::cout << min(res->coll_y) << std::endl;
      if (!res->isFeasible && min(res->coll_y) < -0.01) {
        std::cout << "komo actually infeasible" << std::endl;
        komo_path.has_solution = false;
        break;
      }
    }
  }

  if (komo_path.has_solution && !rrt_path.has_solution) {
    std::cout << "using komo" << std::endl;
    return komo_path;
  }
  if (!komo_path.has_solution && rrt_path.has_solution) {
    std::cout << "using rrt" << std::endl;
    return rrt_path;
  }

  if (komo_path.has_solution && rrt_path.has_solution &&
      komo_path.t(-1) < rrt_path.t(-1)) {
    std::cout << "using komo" << std::endl;
    return komo_path;
  }

  std::cout << "using rrt" << std::endl;
  return rrt_path;
}

rai::Animation::AnimationPart make_animation_part(rai::Configuration &C,
                                                  const arr &path,
                                                  const FrameL &frames,
                                                  const uint t_start) {
  rai::Animation::AnimationPart anim;

  StringA frameNames;
  for (auto f : frames) {
    frameNames.append(f->name);
  }

  anim.start = t_start;
  anim.frameIDs = framesToIndices(frames);
  anim.frameNames = frameNames;

  const uint dt = path.d0;
  anim.X.resize(dt, frames.N, 7);

  arr q;
  for (uint i = 0; i < path.d0; ++i) {
    q = path[i];
    C.setJointState(q);
    // C.watch(true);
    anim.X[i] = C.getFrameState(frames);
  }
  return anim;
}