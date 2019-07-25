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

namespace mc_handover 
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


	// struct CircularTrajectory
	// {
	// 	public:
	// 		CircularTrajectory();
	// 		CircularTrajectory(double radius, std::size_t nr_points, const Eigen::Vector3d& initial);
	// 		std::pair<Eigen::Vector3d, Eigen::Vector3d> pop();
	// 		void reset();
	// 	private:
	// 	double r;
	// 	std::size_t nr_points;
	// 	Eigen::Vector3d x0;
	// 	std::queue<std::pair<Eigen::Vector3d, Eigen::Vector3d> > queue;
	// };


	// class HandoverTrajectoryTask
	// {
	// 	public:
	// 	HandoverTrajectoryTask(mc_solver::QPSolver & solver);
	// 	~HandoverTrajectoryTask();

	// 	bool update();

	// 	long wp_index =0;

	// 	double gainPos	= 1e3;
	// 	double gainVel	= 1e2;
	// 	double weight	= 1e3;

	// 	int tunParam1{20}; //100ms
	// 	int tunParam2{200}; //1sec

	// 	Eigen::MatrixXd pos;
	// 	Eigen::MatrixXd vel;
	// 	Eigen::MatrixXd ace;


	// 	mc_solver::QPSolver & solver;

	// 	std::shared_ptr<tasks::qp::PositionTask> positionTask;
	// 	std::shared_ptr<tasks::qp::TrajectoryTask> trajTask;

	// 	Eigen::Vector3d initPos, refVel, refAce;
	// };


} // namespace mc_handover