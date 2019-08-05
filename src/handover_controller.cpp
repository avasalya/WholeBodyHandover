//
/// ... handovercontroller ...
///

#include "handover_controller.h"

namespace mc_handover
{
	//////////////
	//
	// Handover Controller constructor
	//
	//////////////
	HandoverController::HandoverController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
		double dt, const mc_rtc::Configuration & config)
	:mc_control::fsm::Controller(robot_module, dt, config)
	{
		selfCollisionConstraint.reset();
		selfCollisionConstraint.addCollisions(solver(), {
			mc_rbdyn::Collision("LARM_LINK2", "BODY", 0.15, 0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK3", "BODY", 0.15, 0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK4", "BODY", 0.15, 0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK5", "BODY", 0.15, 0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK6", "BODY", 0.15, 0.10, 0.),

			mc_rbdyn::Collision("RARM_LINK2", "BODY", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK3", "BODY", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK4", "BODY", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK5", "BODY", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK6", "BODY", 0.1, 0.05, 0.),

			mc_rbdyn::Collision("LARM_LINK4", "LLEG_LINK2", 0.15, 0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK5", "LLEG_LINK2", 0.15, 0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK6", "LLEG_LINK2", 0.15, 0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK7", "LLEG_LINK2", 0.15, 0.10, 0.),

			mc_rbdyn::Collision("RARM_LINK4", "RLEG_LINK2", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK5", "RLEG_LINK2", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK6", "RLEG_LINK2", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK7", "RLEG_LINK2", 0.1, 0.05, 0.),

			mc_rbdyn::Collision("LARM_LINK2", "CHEST_LINK0", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("LARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),

			mc_rbdyn::Collision("RARM_LINK2", "CHEST_LINK0", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),

			mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.),

			mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
			mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.),

			mc_rbdyn::Collision("LARM_LINK6", "RARM_LINK6", 0.15,0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK7", "RARM_LINK6", 0.15,0.10, 0.),

			mc_rbdyn::Collision("LARM_LINK6", "RARM_LINK7", 0.15,0.10, 0.),
			mc_rbdyn::Collision("LARM_LINK7", "RARM_LINK7", 0.15,0.10, 0.),
		});
		qpsolver->addConstraintSet(selfCollisionConstraint);


		// contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Position);
		// qpsolver->addConstraintSet(contactConstraint);

		// std::array<double, 3> damper = {{0.01, 0.001, 0.01}};
		// // KinematicsConstraint(robots, robotIndex, timeStep, damper, velocityPercent)
		// kinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 0, timeStep, damper, 1.0);
		// qpsolver->addConstraintSet(kinematicsConstraint);


		LOG_SUCCESS("handoverController init done")
	}



	//////////////
	//
	// Handover Controller reset
	//
	//////////////
	void HandoverController::reset(const ControllerResetData & reset_data)
	{
		mc_control::fsm::Controller::reset(reset_data);

		/*getHostInfo*/
		getHostInfo();

		auto q = reset_data.q;
		MCController::reset({q});

				/* gripper control */
		gui()->addElement({"Handover", "Grippers"},

				mc_rtc::gui::Button("open_Grippers", [this]() { std::string msg = "openGrippers"; read_msg(msg);
					cout<<"opening both grippers via GUI\n";
							// std::cout << "at grippers opening: right hand Forces " << wrenches.at("RightHandForceSensor").force().transpose() << endl;
							// std::cout << "at grippers opening: left hand Forces " << wrenches.at("LeftHandForceSensor").force().transpose() << endl;
				}),

				mc_rtc::gui::Button("close_Grippers",[this]() { std::string msg = "closeGrippers"; read_msg(msg);
					cout<<"closing both grippers via GUI\n";
							// std::cout << "at grippers closing: right hand Forces " << wrenches.at("RightHandForceSensor").force().transpose() << endl;
							// std::cout << "at grippers closing: left hand Forces " << wrenches.at("LeftHandForceSensor").force().transpose() << endl;
				}),

				mc_rtc::gui::Button("open_Right_Gripper",[this]() { std::string msg = "openGripperR"; read_msg(msg);
					cout<<"opening right gripper via GUI\n";
							// std::cout << "at right gripper opening: right hand Forces " << wrenches.at("RightHandForceSensor").force().transpose() << endl;
				}),

				mc_rtc::gui::Button("close_Right_Gripper",[this]() { std::string msg = "closeGripperR"; read_msg(msg);
					cout<<"closing right gripper via GUI\n";
							// std::cout << "at right gripper closing: right hand Forces " << wrenches.at("RightHandForceSensor").force().transpose() << endl;
				}),

				mc_rtc::gui::Button("open_Left_Gripper",[this]() { std::string msg = "openGripperL"; read_msg(msg);
					cout<<"opening left gripper via GUI\n";
					// std::cout << "at left gripper opening: left hand Forces " << wrenches.at("LeftHandForceSensor").force().transpose() << endl;
				}),

				mc_rtc::gui::Button("close_Left_Gripper",[this]() { std::string msg = "closeGripperL"; read_msg(msg);
					cout<<"closing left gripper via GUI\n";
					// std::cout << "at left gripper closing: left hand Forces " << wrenches.at("LeftHandForceSensor").force().transpose() << endl;
				})
			);
	}



	//////////////
	//
	// Handover Controller run
	//
	//////////////
	bool HandoverController::run()
	{
		bool ret = mc_control::fsm::Controller::run();

		if(ret)
		{
			/* Force Sensor*/
			wrenches["LeftHandForceSensor"] =
			// this->robot().forceSensor("LeftHandForceSensor").wrenchWithoutGravity(this->robot());
			this->robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(this->robot());


			wrenches["RightHandForceSensor"] =
			// this->robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(this->robot());
			this->robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(this->robot());
		}
		return ret;
	}



	//////////////
	//
	// getHostInfo
	//
	//////////////
	bool HandoverController::getHostInfo()
	{
		char username[LOGIN_NAME_MAX];
		char hostname[HOST_NAME_MAX];
		int result;

		result = gethostname(hostname, HOST_NAME_MAX);
		if (result)
			{	perror("gethostname");	return EXIT_FAILURE;	}

		result = getlogin_r(username, LOGIN_NAME_MAX);
		if (result)
			{	perror("getlogin_r");	return EXIT_FAILURE;	}

		if( strcmp(username, "hrp2user")==0 && strcmp(hostname, "hrp2012c")==0 )
			{	Flag_ROBOT = true;	}
		else if ( strcmp(username, "avasalya")==0 && strcmp(hostname, "vasalya-xps15")==0 )
			{	Flag_ROBOT = false;	}

		result = printf("Hello %s, you are logged in to %s.\n", username, hostname);
		if (result < 0)
			{	perror("printf");	return EXIT_FAILURE;	}

		return EXIT_SUCCESS;
	}



	//////////////
	//
	// Handover Controller read_msg
	//
	//////////////
	bool  HandoverController::read_msg(std::string & msg)
	{
		std::stringstream ss;
		std::string token;

		ss << msg;
		ss >> token;

		// if(token == "step1")
		// {

		// 	MCController::set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down
		// 	Eigen::Vector3d initPosR, initPosL;
		// 	sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];

		// 	initPosR <<  0.30, -0.35, 0.3;
		// 	relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
		// 	solver().addTask(relEfTaskR);


		// 	initPosL <<  0.30, 0.35, 0.3;
		// 	relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosL));
		// 	solver().addTask(relEfTaskL);

		// 	return true;
		// }



		// if(token == "step2")
		// {
		// 	//set ef pose
		// 	Eigen::Vector3d tL( 0.7, 0.35, .3 );
		// 	Eigen::Matrix3d getCurRotL =  relEfTaskL->get_ef_pose().rotation();
		// 	sva::PTransformd dtrL(getCurRotL, tL);
		// 	relEfTaskL->set_ef_pose(dtrL);


		// 	Eigen::Vector3d tR( 0.7, -0.35, .3 );
		// 	Eigen::Matrix3d getCurRotR =  relEfTaskR->get_ef_pose().rotation();
		// 	sva::PTransformd dtrR(getCurRotR, tR);
		// 	relEfTaskR->set_ef_pose(dtrR);

		// 	return true;
		// }



		// if(token == "robots")
		// {
		// 	std::string robotName =  this->robot().name();

		// 	cout << robotName <<  endl;

		// 	return true;
		// }



		// if(token == "surfaces")
		// {
		// 	surf =  this->robot().surfaces();

		// 	for(auto elem : surf)
		// 	{
		// 		std::cout << elem.first << " " << elem.second << endl;
		// 	}
		// 	return true;
		// }

		// gripper control actions //

		if(token == "openGripperR")
		{
			auto gripper = grippers["r_gripper"].get();
			gripper->setTargetQ({openG});

			return true;
		}
		if(token == "closeGripperR")
		{
			auto gripper = grippers["r_gripper"].get();
			gripper->setTargetQ({closeG});
			return true;
		}



		if(token == "openGripperL")
		{
			auto gripper = grippers["l_gripper"].get();
			gripper->setTargetQ({openG});
			return true;
		}
		if(token == "closeGripperL")
		{
			auto gripper = grippers["l_gripper"].get();
			gripper->setTargetQ({closeG});
			return true;
		}



		if(token == "openGrippers")
		{
			auto gripper = grippers["l_gripper"].get();
			gripper->setTargetQ({openG});
			gripper = grippers["r_gripper"].get();
			gripper->setTargetQ({openG});
			return true;
		}
		if(token == "closeGrippers")
		{
			auto gripper = grippers["l_gripper"].get();
			gripper->setTargetQ({closeG});
			gripper = grippers["r_gripper"].get();
			gripper->setTargetQ({closeG});
			return true;
		}


		// get LARM JOINTs //
		std::vector<double>  get_LArm_Joints(8);
		if(token == "getRtPose")
		{
			for (int i = 0; i<8; ++i)
			{
				std::stringstream ss;
				ss << "RARM_JOINT" << i;
				std::cout << robot().mbc().q[robot().jointIndexByName(ss.str())][0] << ", ";
			}
			std::cout<<std::endl;
			return true;
		}

		// get RARM JOINTs //
		std::vector<double>  get_RArm_Joints(8);
		if(token == "getLtPose")
		{
			for (int i = 0; i<8; ++i)
			{
				std::stringstream ss;
				ss << "LARM_JOINT" << i;
				std::cout << robot().mbc().q[robot().jointIndexByName(ss.str())][0] << ", ";
			}
			std::cout<<std::endl;
			return true;
		}

		LOG_WARNING("Cannot handle " << msg)
		return mc_control::fsm::Controller::read_msg(msg);
	}



	//////////////
	//
	// Handover Controller read_write_msg
	//
	//////////////
	bool HandoverController::read_write_msg(std::string & msg, std::string & out)
	{
		// out = msg;
		// return true;
		return mc_control::fsm::Controller::read_write_msg(msg, out);
	}

} //namespace mc_control

CONTROLLER_CONSTRUCTOR("Handover", mc_handover::HandoverController)
