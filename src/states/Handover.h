#pragma once

// global std
#include <iostream>
#include <fstream>
#include <ctime>

// FSM realated
#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

// Walking controller
#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>
#include <lipm_walking/utils/stats.h>

// MC control
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/LookAtTask.h>

// Tasks
#include "Tasks/QPTasks.h"

// RBD
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/rpy_utils.h>


// ROS
#include <ros/ros.h>
#include <thread>
#include <mc_rtc/ros.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>

// Cortex_ROS_bridge
#include "cortex/cortex.h"
#include <cortex_ros_bridge_msgs/Marker.h>
#include <cortex_ros_bridge_msgs/Markers.h>

// handover related
#include "ApproachObject.h"


using namespace sva;

namespace lipm_walking
{
	namespace states
	{
		struct Handover : State
		{
			// EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		public:

			void start() override;
			void runState() override;
			bool checkTransitions() override;
			void teardown() override;

			void ros_spinner();
			void cortexCallback(const cortex_ros_bridge_msgs::Markers & msg);

			bool Flag_HandoverInit{true};
			bool Flag_HandoverTasks{true};
			bool Flag_HandoverGUI{true};
			bool Flag_HandoverLogs{true};
			bool Flag_HandoverRun{true};
			bool Flag_HeadTracking{true};


			double pi{3.14};
			double DegToRad{pi/180};
			double RadToDeg{180/pi};
			double closeGrippers{0.13};
			double openGrippers{0.6};

			std::vector<std::string> activeJointsName = {"HEAD_JOINT0", "HEAD_JOINT1"};

			/*mocap_simulaton*/
			double pt;
			std::vector<double> pts;
			std::vector<Eigen::MatrixXd> pos;
			std::string name;

			int fps{200};
			int i{0};
			int c{0};
			int runCount{0};
			double timeStep{0.005};

			double leftForce_Xabs{0.0};
			double leftForce_Yabs{0.0};
			double leftForce_Zabs{0.0};
			double rightForce_Xabs{0.0};
			double rightForce_Yabs{0.0};
			double rightForce_Zabs{0.0};


			sva::PTransformd X_0_rel;

			Eigen::Vector3d headVector, headTarget;

			Eigen::Vector3d ltPosW, rtPosW;
			std::vector<Eigen::Vector3d> efLPos, efLVel;
			std::vector<Eigen::Vector3d> efRPos, efRVel;
			Eigen::Vector3d efLAce, efRAce;
			int g{1};

			Eigen::Vector3d fingerPos = Eigen::Vector3d::Zero();

			Eigen::Matrix3d ltRotW, rtRotW;
			Eigen::Matrix3d initRotL, initRotR;
			Eigen::Vector3d initPosL, initPosR;

			Eigen::Vector6d relaxPos;
			Eigen::Matrix3d relaxRotL, relaxRotR;
			Eigen::Vector3d relaxPosL, relaxPosR;

			Eigen::Vector3d bodyPosR, bodyPosS;
			double Xmax{0.8};
			double ID{0.0}; //interpersonal distance

			double logStepSize{0.0};
			std::string walkThisDir = "can move either forward or backward";


			double objBody_rel_robotBody{0.0};

			/*offsets for robot grippers to grasp object*/
			double objLen;
			double objLenLt;
			double al;
			double bl;
			double objLenRt;
			double ar;
			double br;

			Eigen::Vector3d offsetLt, offsetRt;
			Eigen::Vector3d updateOffsetPosL, updateOffsetPosR;

			Eigen::Vector3d vecOffsetL, vecOffsetR;
			sva::PTransformd X_M_offset, X_M_offsetL, X_M_offsetR;
			sva::PTransformd X_M_Obj0;
			sva::PTransformd X_Obj0_offsetEf; //when subj has object
			sva::PTransformd X_Obj0_offsetS; // when robot has object

			Eigen::VectorXd thresh = Eigen::VectorXd::Zero(12);
			Eigen::Vector3d leftTh, rightTh;
			Eigen::Vector3d leftForce, rightForce;
			Eigen::Vector3d leftForceSurf, rightForceSurf;

			std::shared_ptr<mc_tasks::PositionTask> posTaskL;
			std::shared_ptr<mc_tasks::PositionTask> posTaskR;

			std::shared_ptr<mc_tasks::OrientationTask> oriTaskL;
			std::shared_ptr<mc_tasks::OrientationTask> oriTaskR;

			std::shared_ptr<mc_tasks::EndEffectorTask>objEfTask;
			std::shared_ptr<mc_tasks::LookAtTask> headTask;

			std::shared_ptr<lipm_walking::ApproachObject> approachObj;
			std::vector<std::string> subjMarkersName, robotMarkersName;


		public:
			bool startCapture{false};
			bool resetFlags_and_efPose{false};

			bool stepFwd;
			bool stepBack;

		public://Cortex_ROS_Bridge
			std::shared_ptr<ros::NodeHandle> m_nh_;
			std::thread m_ros_spinner_;
			ros::Subscriber l_shape_sub_;

		};

	} // namespace states

} // namespace lipm_walking