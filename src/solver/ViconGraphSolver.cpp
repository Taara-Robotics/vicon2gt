/*
 * The vicon2gt project
 * Copyright (C) 2020 Patrick Geneva
 * Copyright (C) 2020 Guoquan Huang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "ViconGraphSolver.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

ViconGraphSolver::ViconGraphSolver(const Options &options, std::shared_ptr<Propagator> propagator,
                                   std::shared_ptr<Interpolator> interpolator, std::vector<double> timestamp_cameras) {

  // save measurement data
  this->propagator = propagator;
  this->interpolator = interpolator;
  this->timestamp_cameras = timestamp_cameras;

  // Initalize our graphs
  this->graph = new gtsam::NonlinearFactorGraph();
  this->config = std::make_shared<GtsamConfig>();

  init_R_GtoV = options.init_R_GtoV;
  gravity_magnitude = options.gravity_magnitude;
  init_R_BtoI = options.init_R_BtoI;
  init_p_BinI = options.init_p_BinI;
  init_toff_imu_to_vicon = options.init_toff_imu_to_vicon;

  // Debug print to console
  cout << "init_R_GtoV:" << endl << init_R_GtoV << endl;
  cout << "init_R_BtoI:" << endl << init_R_BtoI << endl;
  cout << "init_p_BinI:" << endl << init_p_BinI.transpose() << endl;
  cout << "init_toff_imu_to_vicon:" << endl << init_toff_imu_to_vicon << endl;

  // ================================================================================================
  // ================================================================================================
  // ================================================================================================

  // See if we should estimate time offset
  *config = options.gtsam_config;

  // Number of times we relinearize
  num_loop_relin = options.num_loop_relin;

  // Nice debug print
  cout << "estimate_toff_vicon_to_imu: " << (int)config->estimate_vicon_imu_toff << endl;
  cout << "estimate_ori_vicon_to_imu: " << (int)config->estimate_vicon_imu_ori << endl;
  cout << "estimate_pos_vicon_to_imu: " << (int)config->estimate_vicon_imu_pos << endl;
  cout << "num_loop_relin: " << num_loop_relin << endl;
}

void ViconGraphSolver::build_and_solve() {

  // Ensure we have enough measurements
  if (timestamp_cameras.empty()) {
    throw std::runtime_error("[VICON-GRAPH]: camera timestamp vector is empty");
  }

  // Ensure we have enough measurements after removing invalid
  if (interpolator->get_raw_poses().size() < 2) {
    throw std::runtime_error("[VICON-GRAPH]: not enough vicon poses to optimize with");
  }

  // Delete all camera measurements that occur before our IMU readings
  // Also delete ones before and after the first and last vicon measurements
  cout << "cleaning camera timestamps" << endl;
  int ct_remove_imu = 0;
  int ct_remove_before = 0;
  int ct_remove_after = 0;
  double vicon_first_time_inI = interpolator->get_time_min() + init_toff_imu_to_vicon + 2 * TIME_OFFSET;
  double vicon_last_time_inI = interpolator->get_time_max() + init_toff_imu_to_vicon - 2 * TIME_OFFSET;
  auto it0 = timestamp_cameras.begin();
  while (it0 != timestamp_cameras.end()) {
    if (!propagator->has_bounding_imu(*it0)) {
      it0 = timestamp_cameras.erase(it0);
      ct_remove_imu++;
    } else if ((*it0) < vicon_first_time_inI) {
      it0 = timestamp_cameras.erase(it0);
      ct_remove_before++;
    } else if ((*it0) > vicon_last_time_inI) {
      it0 = timestamp_cameras.erase(it0);
      ct_remove_after++;
    } else {
      it0++;
    }
  }
  cout << "removed " << ct_remove_imu << " imu invalid, " << ct_remove_before << " invalid before vicon, " << ct_remove_after
       << " invalid after vicon" << endl;

  // Ensure we have enough measurements after removing invalid
  if (timestamp_cameras.empty()) {
    throw std::runtime_error("[VICON-GRAPH]: all camera timestamps were outside the IMU/Vicon overlap");
  }

  // Clear old states
  map_states.clear();
  values.clear();

  // Create map of the state timestamps to their IDs
  for (size_t i = 0; i < timestamp_cameras.size(); i++) {
    map_states.insert({timestamp_cameras.at(i), i});
  }

  // Loop a specified number of times, and keep solving the problem
  // One would want this if you want to relinearize the bias estimates in CPI
  for (int i = 0; i <= num_loop_relin; i++) {

    // Build the problem
    build_problem(i == 0);

    // optimize the graph.
    optimize_problem();

    // move values forward in time
    values = values_result;

    // Now print timing statistics
    printf("\033[34m[TIME]: %.4f to build\033[0m\n", (rT2 - rT1).total_microseconds() * 1e-6);
    printf("\033[34m[TIME]: %.4f to optimize\033[0m\n", (rT3 - rT2).total_microseconds() * 1e-6);
    printf("\033[34m[TIME]: %.4f total (loop %d)\033[0m\n", (rT3 - rT1).total_microseconds() * 1e-6, i);

    // Visualize this iteration
    visualize();
  }

  // Debug print results...
  cout << endl << "======================================" << endl;
  cout << "state_0: " << endl << values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(0)])) << endl;
  cout << "state_N: " << endl << values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(timestamp_cameras.size() - 1)])) << endl;
  cout << "R_BtoI: " << endl << quat_2_Rot(values_result.at<JPLQuaternion>(C(0)).q()) << endl << endl;
  cout << "p_BinI: " << endl << values_result.at<Vector3>(C(1)) << endl << endl;
  cout << "R_GtoV: " << endl << values_result.at<RotationXY>(G(0)).rot() << endl << endl;
  cout << "t_off_vicon_to_imu: " << endl << values_result.at<Vector1>(T(0)) << endl << endl;
  cout << "======================================" << endl << endl;
}

void ViconGraphSolver::write_to_file(std::string csvfilepath, std::string infofilepath) {
  write_to_file(std::move(csvfilepath), std::move(infofilepath), ExportOptions{});
}

void ViconGraphSolver::write_to_file(std::string csvfilepath, std::string infofilepath, const ExportOptions &options) {

  // Debug info
  cout << "saving states and info to file" << endl;

  // If the file exists, then delete it
  if (options.overwrite_existing && boost::filesystem::exists(csvfilepath)) {
    boost::filesystem::remove(csvfilepath);
    cout << "    - old state file found, deleted..." << endl;
  }
  if (options.overwrite_existing && boost::filesystem::exists(infofilepath)) {
    boost::filesystem::remove(infofilepath);
    cout << "    - old info file found, deleted..." << endl;
  }
  // Create the directory that we will open the file in
  boost::filesystem::path p1(csvfilepath);
  boost::filesystem::create_directories(p1.parent_path());
  boost::filesystem::path p2(infofilepath);
  boost::filesystem::create_directories(p2.parent_path());

  // Open our state file!
  std::ofstream of_state;
  of_state.open(csvfilepath, std::ofstream::out | std::ofstream::app);
  of_state << "#time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz" << std::endl;

  // Loop through all states, and
  Eigen::Matrix3d R_GtoV = values_result.at<RotationXY>(G(0)).rot();
  Eigen::Vector4d q_GtoV = rot_2_quat(R_GtoV);
  for (size_t i = 0; i < timestamp_cameras.size(); i++) {
    // get this state at this timestep rotated into gravity aligned frame
    JPLNavState state = values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(i)]));
    Eigen::Vector4d q_GtoIi = quat_multiply(state.q(), q_GtoV);
    Eigen::Vector3d p_IiinG = R_GtoV.transpose() * state.p();
    Eigen::Vector3d v_IiinG = R_GtoV.transpose() * state.v();
    // export to file (time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz)
    of_state << std::setprecision(20) << std::floor(1e9 * timestamp_cameras.at(i)) << "," << std::setprecision(6) << p_IiinG(0) << ","
             << p_IiinG(1) << "," << p_IiinG(2) << "," << q_GtoIi(3) << "," << q_GtoIi(0) << "," << q_GtoIi(1) << "," << q_GtoIi(2) << ","
             << v_IiinG(0) << "," << v_IiinG(1) << "," << v_IiinG(2) << "," << state.bg()(0) << "," << state.bg()(1) << "," << state.bg()(2)
             << "," << state.ba()(0) << "," << state.ba()(1) << "," << state.ba()(2) << std::endl;
  }
  of_state.close();

  // Save calibration and the such to file
  std::ofstream of_info;
  of_info.open(infofilepath, std::ofstream::out | std::ofstream::app);
  of_info << "R_BtoI: " << endl << quat_2_Rot(values_result.at<JPLQuaternion>(C(0)).q()) << endl << endl;
  of_info << "q_BtoI: " << endl << values_result.at<JPLQuaternion>(C(0)).q() << endl << endl;
  of_info << "p_BinI: " << endl << values_result.at<Vector3>(C(1)) << endl << endl;
  of_info << "R_GtoV: " << endl << values_result.at<RotationXY>(G(0)).rot() << endl << endl;
  of_info << "R_GtoV (thetax, thetay): " << endl;
  of_info << values_result.at<RotationXY>(G(0)).thetax() << " " << values_result.at<RotationXY>(G(0)).thetax() << endl << endl;
  of_info << "gravity norm: " << endl << gravity_magnitude << endl << endl;
  of_info << "t_off_vicon_to_imu: " << endl << values_result.at<Vector1>(T(0)) << endl << endl;
  of_info.close();
}

void ViconGraphSolver::visualize() {}

void ViconGraphSolver::get_imu_poses(std::vector<double> &times, std::vector<Eigen::Matrix<double, 10, 1>> &poses) {

  // Clear the old data
  times.clear();
  poses.clear();

  // Loop through all states (invalid times will already be removed)
  for (size_t i = 0; i < timestamp_cameras.size(); i++) {

    // get this state at this timestep
    JPLNavState state = values_result.at<JPLNavState>(X(map_states[timestamp_cameras.at(i)]));

    // append to our vectors
    Eigen::Matrix<double, 10, 1> pose;
    pose << state.q(), state.p(), state.v();
    times.push_back(timestamp_cameras.at(i));
    poses.push_back(pose);
  }
}

void ViconGraphSolver::get_calibration(double &toff, Eigen::Matrix3d &R_BtoI, Eigen::Vector3d &p_BinI, Eigen::Matrix3d &R_GtoV) {

  // Get vicon marker body to imu calibration
  R_BtoI = quat_2_Rot(values_result.at<JPLQuaternion>(C(0)).q());
  p_BinI = values_result.at<Vector3>(C(1));

  // Rotation from gravity frame to vicon frame
  R_GtoV = values_result.at<RotationXY>(G(0)).rot();

  // Time offset betwen vicon and imu
  toff = values_result.at<Vector1>(T(0)).matrix()(0);
}

void ViconGraphSolver::build_problem(bool init_states) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Clear the old factors
  cout << "[BUILD]: building the graph (might take a while)" << endl;
  graph->erase(graph->begin(), graph->end());

  // Create gravity and calibration nodes and insert them
  if (init_states) {
    values.insert(C(0), JPLQuaternion(rot_2_quat(init_R_BtoI)));
    values.insert(C(1), Vector3(init_p_BinI));
    Eigen::Vector3d rpy = rot2rpy(init_R_GtoV);
    values.insert(G(0), RotationXY(rpy(0), rpy(1)));
  }
  printf("[BUILD]: initial R_GtoV roll pitch %.4f, %.4f\n", values.at<RotationXY>(G(0)).thetax(), values.at<RotationXY>(G(0)).thetay());

  // If estimating the timeoffset logic
  if (init_states) {
    Vector1 temp;
    temp(0) = init_toff_imu_to_vicon;
    values.insert(T(0), temp);
  }
  // Prior to make time offset stable
  // Vector1 sigma;
  // sigma(0,0) = 0.02; // seconds
  // PriorFactor<Vector1> factor_timemag(T(0), values.at<Vector1>(T(0)), sigma);
  // graph->add(factor_timemag);
  printf("[BUILD]: current time offset is %.4f\n", values.at<Vector1>(T(0))(0));

  // Loop through each camera time and construct the graph
  auto it1 = timestamp_cameras.begin();
  while (it1 != timestamp_cameras.end()) {

    // If ros is wants us to stop, break out
    // Current image time
    double timestamp_inI = *it1;
    double time_from_start = timestamp_inI - timestamp_cameras.at(0);
    double timestamp_inV = timestamp_inI - values.at<Vector1>(T(0))(0);

    // First get the vicon pose at the current time
    Eigen::Vector4d q_VtoB, q_VtoB0, q_VtoB2;
    Eigen::Vector3d p_BinV, p_B0inV, p_B2inV;
    Eigen::Matrix<double, 6, 6> R_vicon;
    bool has_vicon1 = interpolator->get_pose(timestamp_inV - TIME_OFFSET, q_VtoB0, p_B0inV, R_vicon);
    bool has_vicon2 = interpolator->get_pose(timestamp_inV + TIME_OFFSET, q_VtoB2, p_B2inV, R_vicon);
    bool has_vicon3 = interpolator->get_pose(timestamp_inV, q_VtoB, p_BinV, R_vicon);

    // Skip if we don't have a vicon measurement for this pose
    if (!has_vicon1 || !has_vicon2 || !has_vicon3) {
      printf("[WARN]: skipping camera time %.9f - %.2f from beginning (no vicon pose found)\n", timestamp_inI, time_from_start);
      if (values.find(X(map_states[timestamp_inI])) != values.end()) {
        values.erase(X(map_states[timestamp_inI]));
      }
      it1 = timestamp_cameras.erase(it1);
      continue;
    }

    // Check if we can do the inverse
    if (std::isnan(R_vicon.norm()) || std::isnan(R_vicon.inverse().norm())) {
      printf("[WARN]: skipping camera time %.9f - %.2f from beginning (R.norm = %.3f | Rinv.norm = %.3f)\n", timestamp_inI, time_from_start,
             R_vicon.norm(), R_vicon.inverse().norm());
      if (values.find(X(map_states[timestamp_inI])) != values.end()) {
        values.erase(X(map_states[timestamp_inI]));
      }
      it1 = timestamp_cameras.erase(it1);
      continue;
    }

    // Now initialize the current pose of the IMU
    if (init_states) {

      // Orientation and position are the relative to
      Eigen::Vector4d q_VtoI = quat_multiply(rot_2_quat(init_R_BtoI), q_VtoB);
      Eigen::Vector3d p_IinV = p_BinV - quat_2_Rot(Inv(q_VtoB)) * init_R_BtoI.transpose() * init_p_BinI;

      // An initial guess for the velocity is just a sample time derivative from the vicon poses
      // Should be a "Three-point midpoint" numerical derivative
      Eigen::Vector3d p_I0inV = p_B0inV - quat_2_Rot(Inv(q_VtoB0)) * init_R_BtoI.transpose() * init_p_BinI;
      Eigen::Vector3d p_I2inV = p_B2inV - quat_2_Rot(Inv(q_VtoB2)) * init_R_BtoI.transpose() * init_p_BinI;
      Eigen::Vector3d v_IinV = (p_I2inV - p_I0inV) / (2 * TIME_OFFSET);

      // Initialize our biases to zero as the guess here
      Eigen::Vector3d bg = Eigen::Vector3d::Zero();
      Eigen::Vector3d ba = Eigen::Vector3d::Zero();

      // Create the graph node
      JPLNavState imu_state(timestamp_inI, q_VtoI, bg, v_IinV, ba, p_IinV);
      values.insert(X(map_states[timestamp_inI]), imu_state);
    }

    // Add the vicon measurement to this pose
    MeasBased_ViconPoseTimeoffsetFactor factor_vicon(X(map_states[timestamp_inI]), C(0), C(1), T(0), interpolator, config);
    graph->add(factor_vicon);

    // Skip the first ever pose
    if (it1 == timestamp_cameras.begin()) {
      it1++;
      continue;
    }

    // Now add preintegration between this state and the next
    // We do a silly hack since inside of the propagator we create the preintegrator
    // So we just randomly assign noises here which will be overwritten in the propagator
    double time0 = *(it1 - 1);
    double time1 = *(it1);

    // Get the bias of the time0 state
    Bias3 bg = values.at<JPLNavState>(X(map_states[time0])).bg();
    Bias3 ba = values.at<JPLNavState>(X(map_states[time0])).ba();

    // Get the preintegrator (will get recreated with correct noises in propagator)
    CpiV1 preint(0, 0, 0, 0, true);
    bool has_imu = propagator->propagate(time0, time1, bg, ba, preint);
    if (!has_imu || preint.DT != (time1 - time0)) {
      throw std::runtime_error("unable to get IMU readings for preintegration");
    }

    // cout << "dt = " << preint.DT << " | dt_times = " << time1-time0 << endl;
    // cout << "q_k2tau = " << preint.q_k2tau.transpose() << endl;
    // cout << "alpha_tau = " << preint.alpha_tau.transpose() << endl;
    // cout << "beta_tau = " << preint.beta_tau.transpose() << endl;

    // Check if we can do the inverse
    if (std::isnan(preint.P_meas.norm()) || std::isnan(preint.P_meas.inverse().norm())) {
      throw std::runtime_error("IMU preintegration covariance became NaN");
    }

    // Now create the IMU factor
    ImuFactorCPIv1 factor_imu(X(map_states[time0]), X(map_states[time1]), G(0), preint.P_meas, preint.DT, gravity_magnitude,
                              preint.alpha_tau, preint.beta_tau, preint.q_k2tau, preint.b_a_lin, preint.b_w_lin, preint.J_q, preint.J_b,
                              preint.J_a, preint.H_b, preint.H_a);
    graph->add(factor_imu);

    // Finally, move forward in time!
    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();
}

void ViconGraphSolver::optimize_problem() {

  // Debug
  cout << "[VICON-GRAPH]: graph factors - " << (int)graph->nrFactors() << endl;
  cout << "[VICON-GRAPH]: graph nodes - " << (int)graph->keys().size() << endl;

  // Setup the optimizer (levenberg)
  // Use METIS ordering to fix memory issue for large number of nodes
  // See: https://bitbucket.org/gtborg/gtsam/issues/369/segfault-when-running-on-data-sets-with
  LevenbergMarquardtParams opti_config;
  opti_config.verbosity = NonlinearOptimizerParams::Verbosity::TERMINATION;
  // config.verbosityLM = LevenbergMarquardtParams::VerbosityLM::SUMMARY;
  // config.verbosityLM = LevenbergMarquardtParams::VerbosityLM::TERMINATION;
  opti_config.orderingType = Ordering::OrderingType::METIS;
  opti_config.absoluteErrorTol = 1e-30;
  opti_config.relativeErrorTol = 1e-30;
  opti_config.lambdaUpperBound = 1e20;
  opti_config.maxIterations = 30;
  LevenbergMarquardtOptimizer optimizer(*graph, values, opti_config);

  // Setup optimizer (dogleg)
  // DoglegParams params;
  // params.verbosity = NonlinearOptimizerParams::Verbosity::TERMINATION;
  // params.relativeErrorTol = 1e-10;
  // params.absoluteErrorTol = 1e-10;
  // DoglegOptimizer optimizer(*graph, values, params);

  // Perform the optimization
  cout << "[VICON-GRAPH]: begin optimization" << endl;
  values_result = optimizer.optimize();
  cout << "[VICON-GRAPH]: done optimization (" << (int)optimizer.iterations() << " iterations)!" << endl;
  rT3 = boost::posix_time::microsec_clock::local_time();
}
