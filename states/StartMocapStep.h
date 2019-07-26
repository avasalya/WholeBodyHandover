#pragma once

#include <iostream>
#include <fstream>
#include <ctime>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>


#include "Tasks/QPTasks.h"

#include <mc_rbdyn/Robot.h>

#include "../cortex/cortex.h"

#include "handover_approachObject.h"


//Cortex_ROS_bridge
#include <cortex_ros_bridge_msgs/Marker.h>
#include <cortex_ros_bridge_msgs/Markers.h>


// ros
#include <ros/ros.h>
#include <thread>
#include <mc_rtc/ros.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>


using namespace sva;

namespace mc_handover
{
	namespace states
	{
		struct StartMocapStep : mc_control::fsm::State
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		public:
			void configure(const mc_rtc::Configuration&) override {}
			void start(mc_control::fsm::Controller&) override;
			bool run(mc_control::fsm::Controller&) override;
			void teardown(mc_control::fsm::Controller&) override;

			void ros_spinner();
			void cortexCallback(const cortex_ros_bridge_msgs::Markers & msg);

			double pi = 3.14;
			double DegToRad = pi/180;
			double RadToDeg = 180/pi;
			double closeGrippers{0.13};
			double openGrippers{0.5};


			std::vector<std::string> activeJointsName = {"HEAD_JOINT0", "HEAD_JOINT1"};

			bool Flag_CORTEX{false};//TRUE, otherwise use Cortex_ROS_bridge

			bool Flag_oneHand{false};//TRUE, otherwise use both hands


			/*mocap_simulaton*/
			double pt;
			std::vector<double> pts;
			std::vector<Eigen::MatrixXd> pos;
			std::string name;


			int fps{200};
			int g{1};
			int dt{1};
			int maxMarkers, markersCount, until, b_;

			double leftForce_Xabs{0.0};
			double leftForce_Yabs{0.0};
			double leftForce_Zabs{0.0};
			double rightForce_Xabs{0.0};
			double rightForce_Yabs{0.0};
			double rightForce_Zabs{0.0};


			double bodiesDiffX{0.0};
			Eigen::Vector3d bodyPosR, bodyPosS;

			Eigen::Vector3d move, target, initialCom = Eigen::Vector3d::Zero();

			Eigen::Vector3d headVector, headTarget;

			Eigen::Vector3d ltPosW, rtPosW;

			sva::MotionVecd ltBodyAccW, rtBodyAccW;
			std::vector<Eigen::Vector3d> efLPos, efLVel;
			std::vector<Eigen::Vector3d> efRPos, efRVel;
			Eigen::Vector3d efLAce, efRAce;


			Eigen::Vector3d initPosL, initPosR, fingerPos;
			Eigen::Vector3d constPosL, constPosR;
			Eigen::Matrix3d constRotL, constRotR;

			Eigen::Matrix3d ltRotW, rtRotW;
			Eigen::Matrix3d initRotL, initRotR;

			Eigen::VectorXd thresh = Eigen::VectorXd::Zero(12);
			Eigen::Vector3d leftTh, rightTh;
			Eigen::Vector3d leftForce, rightForce;
			Eigen::Vector3d leftForceLo, rightForceLo;


			std::shared_ptr<mc_tasks::PositionTask> posTaskL;
			std::shared_ptr<mc_tasks::OrientationTask> oriTaskL;

			std::shared_ptr<mc_tasks::PositionTask> posTaskR;
			std::shared_ptr<mc_tasks::OrientationTask> oriTaskR;

			std::shared_ptr<mc_tasks::OrientationTask> chestOriTask;
			std::shared_ptr<mc_tasks::PositionTask> chestPosTask;

			std::shared_ptr<mc_tasks::PositionTask> bodyPosTask;
			std::shared_ptr<mc_tasks::OrientationTask> bodyOriTask;

			std::shared_ptr<mc_tasks::EndEffectorTask>objEfTask;

			std::shared_ptr<mc_tasks::LookAtTask> headTask;

			std::shared_ptr<mc_tasks::CoMTask> comTask;

			std::shared_ptr<mc_handover::ApproachObject> approachObj;

			std::vector<std::string> subjMarkersName, robotMarkersName;


		private:
			sBodyDefs* pBodyDefs{NULL};
			sBodyDef* pBody{NULL};
			sFrameOfData* getCurFrame{NULL};
			sFrameOfData FrameofData;

			std::vector<int> bodyMarkers;

			void *pResponse;

			int nBytes;
			int totalBodies;
			int retval = RC_Okay;
			int c{0};

			double del{0};

			bool startCapture{false};
			bool startHandover{false};

			bool taskOK{false};

			bool restartEverything{false};


		public://Cortex_ROS_Bridge
			std::shared_ptr<ros::NodeHandle> m_nh_;
			std::thread m_ros_spinner_;
			ros::Subscriber l_shape_sub_;

		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("StartMocapStep", mc_handover::states::StartMocapStep)