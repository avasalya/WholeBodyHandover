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
#include <time.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_control/mc_global_controller.h>

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/logging.h>

#include <mc_tasks/TrajectoryTask.h>

#include <Tasks/QPTasks.h>

#include <lipm_walking/Controller.h>
#include "Trajectories.h"


using namespace std;
using namespace Eigen;

namespace lipm_walking
{
	struct Controller;

	struct ApproachObject
	{
	public:
		ApproachObject(Controller & controller_);
		~ApproachObject();

		void initials();

		bool checkFrameOfData(std::vector<Eigen::Vector3d>);

		bool handoverRun();

		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> predictionController(
			const Eigen::Vector3d& curPosEf,
			const Eigen::Matrix3d & constRotLink6,
			std::vector<std::string> subjMarkersName);

		void goToHandoverPose(
			double Xmax,
			double Ymin,
			double Ymax,
			bool& enableHand,
			Eigen::Vector3d& curPosEf,
			std::shared_ptr<mc_tasks::PositionTask>& posTask,
			std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
			std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> handPredict,
			Eigen::Vector3d offsetPos);

		bool forceControllerIndividual(
			bool& enableHand,
			Eigen::Vector3d relaxPos,
			Eigen::Vector3d initPos,
			Eigen::Matrix3d initRot,
			Eigen::Vector3d handForce,
			Eigen::Vector3d ForceSurf,
			Eigen::Vector3d thresh,
			Eigen::Vector3d efAce,
			std::shared_ptr<mc_tasks::PositionTask>& posTask,
			std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
			std::string gripperName,
			std::vector<std::string> robotMarkersName,
			std::vector<std::string> lShpMarkersName,
			double obj_rel_robotHand);


		bool forceControllerTogether(
			bool& enableHand,
			sva::PTransformd X_0_rel,
			Eigen::Vector3d initPosR, Eigen::Matrix3d initRotR,
			Eigen::Vector3d initPosL, Eigen::Matrix3d initRotL,
			Eigen::Vector3d relaxPosR, Eigen::Matrix3d relaxRotR,
			Eigen::Vector3d relaxPosL, Eigen::Matrix3d relaxRotL,
			Eigen::VectorXd thresh,
			Eigen::Vector3d leftForce, Eigen::Vector3d rightForce,
			Eigen::Vector3d leftForceLo, Eigen::Vector3d rightForceLo,
			Eigen::Vector3d efLAce, Eigen::Vector3d efRAce,
			std::shared_ptr<mc_tasks::PositionTask>& posTaskL,
			std::shared_ptr<mc_tasks::OrientationTask>& oriTaskL,
			std::shared_ptr<mc_tasks::PositionTask>& posTaskR,
			std::shared_ptr<mc_tasks::OrientationTask>& oriTaskR);


	    Controller & ctl;


		bool Flag_withoutRobot{false}; //TRUE, otherwise use ROBOT_Markers

		bool Flag_prediction{false}; //TRUE otherwise, use finger Position

		bool FlAG_INDIVIDUAL{false}; // TRUE for using individual hand, otherwise use both hands together

		bool Flag_Walk{true}; //TRUE for walking, else only stabilizer

		bool finishedWalk_{false};
		bool walkFwd{false};
		bool walkFwdAgain{false};
		bool walkBack{false};


		Eigen::Vector3d tuner;

		int fps{200};
		int t_predict;
		int t_observe;
		int it;

		int i{1};
		int e{1};

		int totalMarkers;

		int count_hr_success{0};
		int count_rh_success{0};
		int count_hr_fail{0};
		int count_rh_fail{0};
		int count_reset{0};

		time_t start;

		double t1{0.0};
		double t2{0.0};
		double t3{0.0};
		double t4{0.0};
		double t5{0.0};
		double t6{0.0};
		double t7{0.0};
		double t8{0.0};
		double t9{0.0};
		double t_falseClose{0.0};

		bool bool_t1{true};
		bool bool_t6{true};
		bool rh_fail{true};


		double GlobalAvgVelSubjNorm;

		std::vector<Eigen::Vector3d> Markers;
		std::vector<Eigen::MatrixXd> markersPos;

		std::map<std::string, double> markers_name_index;
		std::vector<std::string> strMarkersBodyName, strMarkersName;
		std::vector<std::string> robotLtMarkers, robotRtMarkers;
		std::vector<std::string> objMarkers, subjRtMarkers, subjLtMarkers, subjMarkers, subjHeadMarkers;

		Eigen::Matrix3d idtMat = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d handRot= idtMat;
		Eigen::Matrix3d subjLHandRot, subjRHandRot, objRot;

		Eigen::Vector3d efLPosOfHandover = Eigen::Vector3d::Zero();
		Eigen::Vector3d efRPosOfHandover = Eigen::Vector3d::Zero();
		Eigen::Vector3d hLPosOfHandover  = Eigen::Vector3d::Zero();
		Eigen::Vector3d hRPosOfHandover  = Eigen::Vector3d::Zero();

		Eigen::Vector3d predictPosL = Eigen::Vector3d::Zero();
		Eigen::Vector3d predictPosR = Eigen::Vector3d::Zero();

		Eigen::Vector3d gripperEfL = Eigen::Vector3d::Zero();
		Eigen::Vector3d gripperEfR = Eigen::Vector3d::Zero();

		Eigen::Vector3d gripperLtEfA, gripperRtEfA;
		Eigen::Vector3d gripperLtEfB, gripperRtEfB;

		Eigen::Vector3d headPos, headPos1, headPos2;
		Eigen::Vector3d fingerPosL = Eigen::Vector3d::Zero();
		Eigen::Vector3d fingerPosR = Eigen::Vector3d::Zero();
		Eigen::Vector3d objectPosC = Eigen::Vector3d::Zero();
		Eigen::Vector3d objectPosCx = Eigen::Vector3d::Zero();
		Eigen::Vector3d objectPosCy = Eigen::Vector3d::Zero();

		sva::PTransformd virObjLeft, virObjRight;

		double objAboveWaist{1.0}; //for Ashesh
		double finR_rel_efL, finL_rel_efR;
		double obj_rel_subjLtHand, obj_rel_subjRtHand, obj_rel_robotLtHand, obj_rel_robotRtHand;
		double virObj_rel_subjLtHand, virObj_rel_subjRtHand, virObj_rel_robotLtHand, virObj_rel_robotRtHand;

		std::shared_ptr<lipm_walking::HandoverTrajectory> handoverTraj;

		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> lHandPredict, rHandPredict;

		bool gOpen{false};
		bool gClose{false};
		bool openGripper{false};
		bool closeGripper{false};

		bool graspObject{true};
		bool takeBackObject{false};

		bool goBackInitPose{true};
		bool restartHandover{false};

		bool startNow{false};

		double objMass{0.0};

	public: // individual hand

		bool useLtEf{true};
		bool stopLtEf{true};

		bool useRtEf{true};
		bool stopRtEf{true};

		bool enableLHand{true};
		bool enableRHand{true};

		bool pickaHand{false};

		std::vector<double> Floadx, Floady, Floadz;
		Eigen::Vector3d localSurf_Fzero = Eigen::Vector3d::Zero();
		Eigen::Vector3d newTh = Eigen::Vector3d::Zero();
		Eigen::Vector3d Finert = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fzero = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fclose = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fload = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fpull = Eigen::Vector3d::Zero();

	public: // together hands

		bool addContacts{false};
		bool removeContacts{false};
		bool objHasContacts{false};

		bool useLeftEf{false};
		bool useRightEf{false};

		bool subjHasObject{true};
		bool robotHasObject{false};

		bool enableHand{true};

		bool pickNearestHand{true};

		bool handoverComplete{false};

		std::vector<double> FloadLx, FloadLy, FloadLz;
		std::vector<double> FloadRx, FloadRy, FloadRz;

		Eigen::Vector3d localSurf_FzeroL = Eigen::Vector3d::Zero();
		Eigen::Vector3d newThL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FinertL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FzeroL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FcloseL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FloadL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FpullL = Eigen::Vector3d::Zero();

		Eigen::Vector3d localSurf_FzeroR = Eigen::Vector3d::Zero();
		Eigen::Vector3d newThR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FinertR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FzeroR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FcloseR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FloadR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FpullR = Eigen::Vector3d::Zero();

	};//strcut ApproachObject

}//namespace lipm_walking
