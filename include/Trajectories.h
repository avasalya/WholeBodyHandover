#pragma once


#include <cmath>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <tuple>
#include <queue>
#include <utility>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <mc_control/mc_controller.h>

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/logging.h>

#include <mc_tasks/TrajectoryTask.h>

#include <Tasks/QPTasks.h>

using namespace std;
using namespace mc_control;
using namespace Eigen;

namespace lipm_walking
{

	struct HandoverTrajectory
	{

		HandoverTrajectory();
		~HandoverTrajectory();


		std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> minJerkZeroBoundary(const Eigen::Vector3d & xi, const Eigen::Vector3d & xf, double tf);

		std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>  minJerkNonZeroBoundary(const Eigen::Vector3d & xi, const Eigen::Vector3d & vi, const Eigen::Vector3d & ai, const Eigen::Vector3d & xc, double tf);

		std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> minJerkPredictPos(const Eigen::Vector3d & xi, const Eigen::Vector3d & xc, double t0, double tc, double tf);

		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> constVelocity(const Eigen::Vector3d & xi, const Eigen::Vector3d & xf, double tf);

		Eigen::Vector3d constVelocityPredictPos(const Eigen::Vector3d & xdot, const Eigen::Vector3d & C, double tf);

		Eigen::MatrixXd diff(Eigen::MatrixXd data);
		Eigen::Vector3d takeAverage(Eigen::MatrixXd m);
	};

} // namespace lipm_walking