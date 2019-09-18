#include "Handover.h"

namespace lipm_walking
{

	namespace states
	{

		void states::Handover::ros_spinner()
		{ ros::spin(); }


		void states::Handover::cortexCallback(const cortex_ros_bridge_msgs::Markers & msg)
		{

			c = 0;
			for(unsigned int d=0; d<msg.markers.size(); d++)
			{

				if(msg.markers.at(d).marker_name== "dummy")
				{}
				else if(msg.markers.at(d).marker_name== "dummyleft")
				{}
				else if(msg.markers.at(d).marker_name== "dummyHead")
				{}
				else if( c < approachObj->totalMarkers)/*used markers count*/
				{
					approachObj->Markers[c] <<
					msg.markers.at(d).translation.x, msg.markers.at(d).translation.y, msg.markers.at(d).translation.z;
					// LOG_INFO(msg.markers.at(d).marker_name <<" "<<c <<"    "<< approachObj->Markers[c].transpose())
					c+=1;
				}

			}

		}


		void states::Handover::start()
		{

			auto & ctl = controller();

			if(Flag_HandoverInit)
			{

				/*current time*/
				time_t now = time(0);
				char* dt = ctime(&now);


				/*configure MOCAP*/
				m_nh_ = mc_rtc::ROSBridge::get_node_handle();
				if(!m_nh_)
				{
				LOG_ERROR_AND_THROW(std::runtime_error, "This controller does not work withtout ROS")
				}
				m_ros_spinner_ = std::thread{[this](){ this->ros_spinner(); }};
				l_shape_sub_ = m_nh_->subscribe("novis_markers", 1, & lipm_walking::states::Handover::cortexCallback, this);


				/*allocate memory*/
				approachObj = std::make_shared<lipm_walking::ApproachObject>(ctl);
				approachObj->initials();

				if(approachObj->FlAG_INDIVIDUAL)
				{
					LOG_ERROR("Starting **INDIVIDUAL** hands scenario at " <<  dt)
				}
				else //TOGETHER
				{
					LOG_ERROR("Starting **TOGETHER** hands scenario at " << dt)
				}


				stepFwd = true;
				stepBack = true;


				/*close grippers for safety*/
				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();
				gripperL->setTargetQ({closeGrippers});
				gripperR->setTargetQ({closeGrippers});


				if(approachObj->FlAG_INDIVIDUAL)
				{
					relaxRotL<<
						-0.1624, -0.0616,  0.974,
						 0.092,   0.9926,  0.0784,
						-0.9716,  0.104,  -0.1554;
					relaxRotR<<
						0.013618,  0.0805374,   0.996659,
						-0.243029,   0.967128, -0.0748305,
						-0.969923,  -0.241198,  0.0327433;

			 			thresh << 10, 10, 10, 5, 5, 5, 10, 10, 10, 5, 5, 5;
			 			// thresh << 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10;
						leftTh = thresh.segment(3,3);
						rightTh = thresh.segment(9,3);
				}
				else //TOGETHER
				{
					relaxRotL<<
						-0.0267968,  -0.999573,  0.0116803,
						-0.141427,  0.0153579,    0.98983,
						-0.989586,  0.0248723,  -0.141778;
					relaxRotR<<
						-0.181998,  0.982828, -0.0304213,
						-0.0267631, -0.0358778,  -0.998998,
						 -0.982935,  -0.181001,  0.0328332;

					thresh << 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10;
				}

				relaxPos << 0.20, 0.35, 1.0, 0.20, -0.35, 1.0;
				relaxPosL << relaxPos.segment(0,3);
				relaxPosR << relaxPos.segment(3,3);


				efLPos.resize(3);
				efLVel.resize(2);

				efRPos.resize(3);
				efRVel.resize(2);

			}

			if(Flag_HandoverTasks)
			{

				/*HeadTask*/
				headVector<<1., 0., 0.;
				headTarget<<1., 0., 0.;
				headTask.reset(new mc_tasks::LookAtTask("HEAD_LINK1", headVector, headTarget, ctl.robots(), ctl.robots().robotIndex(), 2., 500.));
				ctl.solver().addTask(headTask);
				headTask->target(Eigen::Vector3d(0.5, 0.2, 1.));
				headTask->selectActiveJoints(ctl.solver(), activeJointsName);


				/*Ef pos Tasks*/
				posTaskL = make_shared<mc_tasks::PositionTask>("LARM_LINK7", ctl.robots(), 0, 4.0, 1e3);
				ctl.solver().addTask(posTaskL);
				initPosL = posTaskL->position();

				posTaskR = make_shared<mc_tasks::PositionTask>("RARM_LINK7", ctl.robots(), 0, 4.0, 1e3);
				ctl.solver().addTask(posTaskR);
				initPosR = posTaskR->position();


				/*Ef ori Task*/
				oriTaskL = make_shared<mc_tasks::OrientationTask>("LARM_LINK6", ctl.robots(), 0, 4.0, 500);
				ctl.solver().addTask(oriTaskL);
				initRotL = oriTaskL->orientation();

				oriTaskR = make_shared<mc_tasks::OrientationTask>("RARM_LINK6", ctl.robots(), 0, 4.0, 500);
				ctl.solver().addTask(oriTaskR);
				initRotR = oriTaskR->orientation();


				/*handover endEffectorTask*/
				objEfTask = make_shared<mc_tasks::EndEffectorTask>("base_link", ctl.robots(), 2, 2.0, 1e3);
				ctl.solver().addTask(objEfTask);
				objEfTask->set_ef_pose(Eigen::Vector3d(0.35, 0.0, 0.845));

			}

			if(Flag_HandoverLogs)
			{

				if(approachObj->FlAG_INDIVIDUAL)
				{

					ctl.logger().addLogEntry("HANDOVER_indiv_enableLHand",[this]() -> double { return approachObj->enableLHand; });
					ctl.logger().addLogEntry("HANDOVER_indiv_enableRHand",[this]() -> double { return approachObj->enableRHand; });

					ctl.logger().addLogEntry("HANDOVER_indiv_tryToPull",[this]() -> double { return approachObj->tryToPull; });

					ctl.logger().addLogEntry("HANDOVER_indiv_useLtEf",[this]() -> double { return approachObj->useLtEf; });
					ctl.logger().addLogEntry("HANDOVER_indiv_stopLtEf",[this]() -> double { return approachObj->stopLtEf; });

					ctl.logger().addLogEntry("HANDOVER_indiv_useRtEf",[this]() -> double { return approachObj->useRtEf; });
					ctl.logger().addLogEntry("HANDOVER_indiv_stopRtEf",[this]() -> double { return approachObj->stopRtEf; });

					ctl.logger().addLogEntry("HANDOVER_indiv_localSurfF0",[this]() -> Eigen::Vector3d { return approachObj->localSurf_Fzero; });
					ctl.logger().addLogEntry("HANDOVER_indiv_Fzero",[this]() -> Eigen::Vector3d { return approachObj->Fzero; });
					ctl.logger().addLogEntry("HANDOVER_indiv_Fclose",[this]() -> Eigen::Vector3d { return approachObj->Fclose; });
					ctl.logger().addLogEntry("HANDOVER_indiv_Finert",[this]() -> Eigen::Vector3d { return approachObj->Finert; });
					ctl.logger().addLogEntry("HANDOVER_indiv_Fload",[this]() -> Eigen::Vector3d { return approachObj->Fload; });
					ctl.logger().addLogEntry("HANDOVER_indiv_Fpull",[this]() -> Eigen::Vector3d { return approachObj->Fpull; });
					ctl.logger().addLogEntry("HANDOVER_indiv_newTh",[this]() -> Eigen::Vector3d { return approachObj->newTh; });

					ctl.logger().addLogEntry("HANDOVER_efPosOfHandover",[this]() -> Eigen::Vector3d {return approachObj->efPosOfHandover; });
					ctl.logger().addLogEntry("HANDOVER_hPosOfHandover",[this]() -> Eigen::Vector3d {return approachObj->hPosOfHandover; });

				}
				else // TOGETHER
				{

					ctl.logger().addLogEntry("HANDOVER_together_robotHasObject",[this]() -> double { return approachObj->robotHasObject; });
					ctl.logger().addLogEntry("HANDOVER_together_subjHasObject",[this]() -> double { return approachObj->subjHasObject; });
					ctl.logger().addLogEntry("HANDOVER_together_pickNearestHand",[this]() -> double { return approachObj->pickNearestHand; });

					ctl.logger().addLogEntry("HANDOVER_together_enableHand",[this]() -> double { return approachObj->enableHand; });


					ctl.logger().addLogEntry("HANDOVER_indiv_localSurfFL",[this]() -> Eigen::Vector3d { return approachObj->localSurf_FzeroL; });
					ctl.logger().addLogEntry("HANDOVER_together_FzeroL",[this]() -> Eigen::Vector3d { return approachObj->FzeroL; });
					ctl.logger().addLogEntry("HANDOVER_together_FcloseL",[this]() -> Eigen::Vector3d { return approachObj->FcloseL; });
					ctl.logger().addLogEntry("HANDOVER_together_FinertL",[this]() -> Eigen::Vector3d { return approachObj->FinertL; });
					ctl.logger().addLogEntry("HANDOVER_together_FloadL",[this]() -> Eigen::Vector3d { return approachObj->FloadL; });
					ctl.logger().addLogEntry("HANDOVER_together_FpullL",[this]() -> Eigen::Vector3d { return approachObj->FpullL; });
					ctl.logger().addLogEntry("HANDOVER_together_newThL",[this]() -> Eigen::Vector3d { return approachObj->newThL; });


					ctl.logger().addLogEntry("HANDOVER_indiv_localSurfFR",[this]() -> Eigen::Vector3d { return approachObj->localSurf_FzeroR; });
					ctl.logger().addLogEntry("HANDOVER_together_FzeroR",[this]() -> Eigen::Vector3d { return approachObj->FzeroR; });
					ctl.logger().addLogEntry("HANDOVER_together_FcloseR",[this]() -> Eigen::Vector3d { return approachObj->FcloseR; });
					ctl.logger().addLogEntry("HANDOVER_together_FinertR",[this]() -> Eigen::Vector3d { return approachObj->FinertR; });
					ctl.logger().addLogEntry("HANDOVER_together_FloadR",[this]() -> Eigen::Vector3d { return approachObj->FloadR; });
					ctl.logger().addLogEntry("HANDOVER_together_FpullR",[this]() -> Eigen::Vector3d { return approachObj->FpullR; });
					ctl.logger().addLogEntry("HANDOVER_together_newThR",[this]() -> Eigen::Vector3d { return approachObj->newThR; });

					ctl.logger().addLogEntry("HANDOVER_together_updateOffsetPosL",[this]() -> Eigen::Vector3d { return updateOffsetPosL; });
					ctl.logger().addLogEntry("HANDOVER_together_updateOffsetPosR",[this]() -> Eigen::Vector3d { return updateOffsetPosR; });


					ctl.logger().addLogEntry("HANDOVER_efLPosOfHandover",[this]() -> Eigen::Vector3d {return approachObj->efLPosOfHandover; });
					ctl.logger().addLogEntry("HANDOVER_hRPosOfHandover",[this]() -> Eigen::Vector3d {return approachObj->hRPosOfHandover ; });

					ctl.logger().addLogEntry("HANDOVER_efRPosOfHandover",[this]() -> Eigen::Vector3d {return approachObj->efRPosOfHandover; });
					ctl.logger().addLogEntry("HANDOVER_hLPosOfHandover",[this]() -> Eigen::Vector3d {return approachObj->hLPosOfHandover ; });

				}


				ctl.logger().addLogEntry("HANDOVER_together_efLAce",[this]() -> Eigen::Vector3d { return efLAce; });
				ctl.logger().addLogEntry("HANDOVER_together_efRAce",[this]() -> Eigen::Vector3d { return efRAce; });

				ctl.logger().addLogEntry("HANDOVER_cycle_1st",[this]() -> double { return approachObj->cycle_1st; });
				ctl.logger().addLogEntry("HANDOVER_cycle_2nd",[this]() -> double { return approachObj->cycle_2nd; });

				ctl.logger().addLogEntry("HANDOVER_human_isReady",[this]() -> double { return isHumanReady; });
				ctl.logger().addLogEntry("HANDOVER_human_near",[this]() -> double { return approachObj->human_near; });
				ctl.logger().addLogEntry("HANDOVER_human_far",[this]() -> double { return approachObj->human_far; });

				ctl.logger().addLogEntry("HANDOVER_Flag_Individual",[this]() -> double { return approachObj->FlAG_INDIVIDUAL; });
				ctl.logger().addLogEntry("HANDOVER_Flag_walk",[this]() -> double { return approachObj->Flag_WALK; });
				ctl.logger().addLogEntry("HANDOVER_Flag_enableWalk",[this]() -> double { return approachObj->enableWalk; });
				ctl.logger().addLogEntry("HANDOVER_Flag_walkFwd",[this]() -> double { return approachObj->walkFwd; });
				ctl.logger().addLogEntry("HANDOVER_Flag_walkFwdAgain",[this]() -> double { return approachObj->walkFwdAgain; });
				ctl.logger().addLogEntry("HANDOVER_Flag_walkBack",[this]() -> double { return approachObj->walkBack; });
				ctl.logger().addLogEntry("HANDOVER_Flag_finishedWalk_",[this]() -> double { return approachObj->finishedWalk_; });

				ctl.logger().addLogEntry("HANDOVER_RoutineCompleted",[this]() -> double { return approachObj->handoverComplete; });

				ctl.logger().addLogEntry("HANDOVER_MaxAllowedDist",[this]() -> double { return MAX_ALLOWED_DIST; });

				ctl.logger().addLogEntry("HANDOVER_thresh",[this]() -> Eigen::VectorXd { return thresh; });
				ctl.logger().addLogEntry("HANDOVER_leftForce_Xabs",[this]() -> double { return leftForce_Xabs; });
				ctl.logger().addLogEntry("HANDOVER_leftForce_Yabs",[this]() -> double { return leftForce_Yabs; });
				ctl.logger().addLogEntry("HANDOVER_leftForce_Zabs",[this]() -> double { return leftForce_Zabs; });
				ctl.logger().addLogEntry("HANDOVER_rightForce_Xabs",[this]() -> double { return rightForce_Xabs; });
				ctl.logger().addLogEntry("HANDOVER_rightForce_Yabs",[this]() -> double { return rightForce_Yabs; });
				ctl.logger().addLogEntry("HANDOVER_rightForce_Zabs",[this]() -> double { return rightForce_Zabs; });
				ctl.logger().addLogEntry("HANDOVER_leftForceSurf",[this]() -> Eigen::Vector3d { return leftForceSurf; });
				ctl.logger().addLogEntry("HANDOVER_rightForceSurf",[this]() -> Eigen::Vector3d { return rightForceSurf; });

				ctl.logger().addLogEntry("HANDOVER_posTaskL", [this]() -> Eigen::Vector3d { return posTaskL->position(); });
				ctl.logger().addLogEntry("HANDOVER_posTaskR", [this]() -> Eigen::Vector3d { return posTaskR->position(); });
				ctl.logger().addLogEntry("HANDOVER_rpyFrom_oriTaskL", [this]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(oriTaskL->orientation()); });
				ctl.logger().addLogEntry("HANDOVER_rpyFrom_oriTaskR", [this]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(oriTaskR->orientation()); });

				ctl.logger().addLogEntry("HANDOVER_posTaskLEval", [this]() -> Eigen::Vector3d { return posTaskL->eval(); });
				ctl.logger().addLogEntry("HANDOVER_posTaskREval", [this]() -> Eigen::Vector3d { return posTaskR->eval(); });
				ctl.logger().addLogEntry("HANDOVER_oriTaskLEval", [this]() -> Eigen::Vector3d { return oriTaskL->eval(); });
				ctl.logger().addLogEntry("HANDOVER_oriTaskREval", [this]() -> Eigen::Vector3d { return oriTaskR->eval(); });

				ctl.logger().addLogEntry("HANDOVER_fingerPosL",[this]() -> Eigen::Vector3d { return approachObj->fingerPosL; });
				ctl.logger().addLogEntry("HANDOVER_fingerPosR",[this]() -> Eigen::Vector3d { return approachObj->fingerPosR; });

				ctl.logger().addLogEntry("HANDOVER_rpyFrom_subjLHandRot", [this]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(approachObj->subjLHandRot); });
				ctl.logger().addLogEntry("HANDOVER_rpyFrom_subjRHandRot", [this]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(approachObj->subjRHandRot); });

				ctl.logger().addLogEntry("HANDOVER_objmassZ",[this]() -> double { return approachObj->objMassZ; });
				ctl.logger().addLogEntry("HANDOVER_objmassNorm",[this]() -> double { return approachObj->objMassNorm; });

				ctl.logger().addLogEntry("HANDOVER_objAboveWaist",[this]() -> double { return approachObj->objAboveWaist; });
				ctl.logger().addLogEntry("HANDOVER_objectPosC",[this]()-> Eigen::Vector3d { return approachObj->objectPosC; });
				ctl.logger().addLogEntry("HANDOVER_objectPosCx",[this]()-> Eigen::Vector3d { return approachObj->objectPosCx; });
				ctl.logger().addLogEntry("HANDOVER_objectPosCy",[this]()-> Eigen::Vector3d { return approachObj->objectPosCy; });

				ctl.logger().addLogEntry("HANDOVER_rpyFrom_objRot", [this]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(approachObj->objRot); });

				ctl.logger().addLogEntry("HANDOVER_objRel_subjLtHand",[this]() -> double { return approachObj->obj_rel_subjLtHand; });
				ctl.logger().addLogEntry("HANDOVER_objRel_subjRtHand",[this]() -> double { return approachObj->obj_rel_subjRtHand; });
				ctl.logger().addLogEntry("HANDOVER_objRel_robotLtHand",[this]() -> double { return approachObj->obj_rel_robotLtHand; });
				ctl.logger().addLogEntry("HANDOVER_objRel_robotRtHand",[this]() -> double { return approachObj->obj_rel_robotRtHand; });
				ctl.logger().addLogEntry("HANDOVER_objRel_RobotBodyDist",[this]() -> double { return objBody_rel_robotBody; });

				ctl.logger().addLogEntry("HANDOVER_X-0-rel",[this]() -> Eigen::Vector3d { return X_0_rel.translation(); });
				ctl.logger().addLogEntry("HANDOVER_bodyPosR",[this]() -> Eigen::Vector3d { return bodyPosR; });
				ctl.logger().addLogEntry("HANDOVER_bodyPosS",[this]() -> Eigen::Vector3d { return bodyPosS; });
				ctl.logger().addLogEntry("HANDOVER_b2bDist",[this]() -> double { return ID; });
				ctl.logger().addLogEntry("HANDOVER_stepSize",[this]() -> double { return logStepSize; });

				ctl.logger().addLogEntry("HANDOVER_elapsed_t1",[this]() -> double { return approachObj->t1; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t2",[this]() -> double { return approachObj->t2; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t3",[this]() -> double { return approachObj->t3; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t4",[this]() -> double { return approachObj->t4; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t5",[this]() -> double { return approachObj->t5; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t6",[this]() -> double { return approachObj->t6; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t7",[this]() -> double { return approachObj->t7; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t8",[this]() -> double { return approachObj->t8; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_t9",[this]() -> double { return approachObj->t9; });
				ctl.logger().addLogEntry("HANDOVER_elapsed_tFalseClose",[this]() -> double { return approachObj->t_falseClose; });

				ctl.logger().addLogEntry("HANDOVER_trials_hr_success",[this]() -> double { return approachObj->count_hr_success; });
				ctl.logger().addLogEntry("HANDOVER_trials_hr_fail",[this]() -> double { return approachObj->count_hr_fail; });
				ctl.logger().addLogEntry("HANDOVER_trials_rh_success",[this]() -> double { return approachObj->count_rh_success; });
				ctl.logger().addLogEntry("HANDOVER_trials_rh_fail",[this]() -> double { return approachObj->count_rh_fail; });
				ctl.logger().addLogEntry("HANDOVER_trials_reset",[this]() -> double { return approachObj->count_reset; });

				ctl.logger().addLogEntry("HANDOVER_gOpen",[this]() -> double { return approachObj->gOpen; });
				ctl.logger().addLogEntry("HANDOVER_gClose",[this]() -> double { return approachObj->gClose; });

				ctl.logger().addLogEntry("HANDOVER_OpenGripper",[this]() -> double { return approachObj->openGripper; });
				ctl.logger().addLogEntry("HANDOVER_CloseGripper",[this]() -> double { return approachObj->closeGripper; });

				ctl.logger().addLogEntry("HANDOVER_gripperEfL",[this]() -> Eigen::Vector3d { return approachObj->gripperEfL; });
				ctl.logger().addLogEntry("HANDOVER_gripperEfR",[this]() -> Eigen::Vector3d { return approachObj->gripperEfR; });

				ctl.logger().addLogEntry("HANDOVER_predictPosL",[this]() -> Eigen::Vector3d { return approachObj->predictPosL; });
				ctl.logger().addLogEntry("HANDOVER_predictPosR",[this]() -> Eigen::Vector3d { return approachObj->predictPosR; });

			}

			if(Flag_HandoverGUI)
			{

				/*change prediction_ settings*/
				ctl.gui()->addElement({"Handover", "tuner"},
					mc_rtc::gui::ArrayInput("t_predict/t_observe", {"t_predict", "t_observe", "it"},
						[this]() { return approachObj->tuner; },
						[this](const Eigen::Vector3d & to){approachObj->tuner = to;cout<< "t_predict = " << approachObj->tuner(0)*1/fps<< "sec, t_observe = "<<approachObj->tuner(1)*1/fps<< "sec"<<endl;}),

					mc_rtc::gui::NumberInput("subject waist height",
						[this]() { return approachObj->objAboveWaist; },
						[this](double waist)
						{	approachObj->objAboveWaist = waist;
						cout << "current subject waist height is " << approachObj->objAboveWaist << endl; }));

				/*add remove contact*/
				ctl.gui()->addElement({"Handover", "Contacts"},

					mc_rtc::gui::Button("Add ground contact", [this, &ctl]()
					{
						ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
					}),

					mc_rtc::gui::Button("Remove ground contact", [this, &ctl]()
					{
						ctl.removeContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
					}),

					mc_rtc::gui::Button("Add object/gripper contacts", [this, &ctl]()
					{
						approachObj->robotHasObject = true;
						approachObj->subjHasObject = false;

						approachObj->pickNearestHand = false;
						subjMarkersName = approachObj->subjRtMarkers;

						ctl.solver().removeTask(objEfTask);

						ctl.solver().removeTask(posTaskR);
						ctl.solver().removeTask(oriTaskR);

						ctl.addContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
						ctl.addContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
					}),

					mc_rtc::gui::Button("Remove object/gripper contacts", [this, &ctl]()
					{
						approachObj->subjHasObject = true;
						approachObj->robotHasObject = false;

						approachObj->pickNearestHand = true;

						ctl.solver().addTask(objEfTask);

						ctl.solver().addTask(posTaskR);
						ctl.solver().addTask(oriTaskR);

						ctl.solver().addTask(posTaskL);
						ctl.solver().addTask(oriTaskL);

						oriTaskL->reset();
						posTaskL->reset();

						oriTaskR->reset();
						posTaskR->reset();

						ctl.removeContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
						ctl.removeContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
					})

					);

				/*restart mocapStep*/
				ctl.gui()->addElement({"Handover", "Restart"},
					mc_rtc::gui::Button("resetFlags_and_efPose",
						[this]() { resetFlags_and_efPose = true; }));

				ctl.gui()->addElement({"Handover", "Routine"},
					mc_rtc::gui::Button("TWO handed handover",
						[this]() { approachObj->FlAG_INDIVIDUAL = false; LOG_ERROR("Switching to TWO-hand handover") }),
					mc_rtc::gui::Button("ONE hand handover",
						[this]() { approachObj->FlAG_INDIVIDUAL = true; LOG_ERROR("Switching to ONE-hand handover") }));

				/*reset robot pose*/
				ctl.gui()->addElement({"Handover", "Reset Pose"},

					mc_rtc::gui::Button("openGrippers", [this, &ctl]()
					{
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({openGrippers});

						gripper = ctl.grippers["r_gripper"].get();
						gripper->setTargetQ({openGrippers});
					}),

					mc_rtc::gui::Button("closeGrippers", [this, &ctl]()
					{
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({closeGrippers});

						gripper = ctl.grippers["r_gripper"].get();
						gripper->setTargetQ({closeGrippers});
					}),

					mc_rtc::gui::Button("open LEFT Gripper", [this, &ctl]()
					{
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({openGrippers});
					}),

					mc_rtc::gui::Button("close LEFT Gripper", [this, &ctl]()
					{
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({closeGrippers});
					}),


					mc_rtc::gui::Button("display LEFT hand wrench", [this, &ctl]()
					{
						LOG_ERROR("LEFT wrench " << ctl.robot().forceSensor("LeftHandForceSensor").wrench().force().transpose()
							<< "       ---- NORM ---- "<< ctl.robot().forceSensor("LeftHandForceSensor").wrench().force().norm())

						LOG_SUCCESS("LEFT wrenchWithoutGravity " <<
						ctl.robot().forceSensor("LeftHandForceSensor").wrenchWithoutGravity(ctl.robot()).force().transpose()
						<< "       ---- NORM ---- "<< ctl.robot().forceSensor("LeftHandForceSensor").wrenchWithoutGravity(ctl.robot()).force().norm())

						LOG_ERROR("LEFT worldWrench " <<
						ctl.robot().forceSensor("LeftHandForceSensor").worldWrench(ctl.robot()).force().transpose()
						<< "       ---- NORM ---- "<< ctl.robot().forceSensor("LeftHandForceSensor").worldWrench(ctl.robot()).force().norm())

						LOG_SUCCESS("LEFT worldWrenchWithoutGravity " <<
						ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()
						<< "       ---- NORM ---- "<< ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().norm()<<"\n")

					}),

					mc_rtc::gui::Button("open RIGHT Gripper", [this, &ctl]()
					{
						auto gripper = ctl.grippers["r_gripper"].get();
						gripper->setTargetQ({openGrippers});

						cout << "RIGHT worldWrenchWithoutGravity " <<
						ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
					}),

					mc_rtc::gui::Button("close RIGHT Gripper", [this, &ctl]()
					{
						auto gripper = ctl.grippers["r_gripper"].get();
						gripper->setTargetQ({closeGrippers});
					}),


					mc_rtc::gui::Button("display RIGHT hand wrench", [this, &ctl]()
					{
						LOG_ERROR("RIGHT wrench " << ctl.robot().forceSensor("RightHandForceSensor").wrench().force().transpose()
							<< "       ---- NORM ---- "<< ctl.robot().forceSensor("RightHandForceSensor").wrench().force().norm())

						LOG_SUCCESS("RIGHT wrenchWithoutGravity " <<
						ctl.robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(ctl.robot()).force().transpose()
						<< "       ---- NORM ---- "<< ctl.robot().forceSensor("RightHandForceSensor").wrenchWithoutGravity(ctl.robot()).force().norm())

						LOG_ERROR("RIGHT worldWrench " <<
						ctl.robot().forceSensor("RightHandForceSensor").worldWrench(ctl.robot()).force().transpose()
						<< "       ---- NORM ---- "<< ctl.robot().forceSensor("RightHandForceSensor").worldWrench(ctl.robot()).force().norm())

						LOG_SUCCESS("RIGHT worldWrenchWithoutGravity " <<
						ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()
						<< "       ---- NORM ---- "<< ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().norm()<<"\n")
					}),


					mc_rtc::gui::Button("LEFT ARM orientation", [this, &ctl]()
					{
						oriTaskL->orientation(relaxRotL);
					}),

					mc_rtc::gui::Button("RIGHT ARM orientation", [this, &ctl]()
					{
						oriTaskR->orientation(relaxRotR);
					}),

					mc_rtc::gui::Button("LEFT ARM object-rest pose", [this, &ctl]()
					{
						// posTaskL->position(relaxPosL + X_0_rel.translation());
						posTaskL->position(Eigen::Vector3d(X_0_rel.translation()(0)+0.2, relaxPosL(1), relaxPosL(2)));
						oriTaskL->orientation(relaxRotL);
					}),

					mc_rtc::gui::Button("RIGHT ARM object-rest pose", [this, &ctl]()
					{
						// posTaskR->position(relaxPosR + X_0_rel.translation());
						posTaskR->position(Eigen::Vector3d(X_0_rel.translation()(0)+0.2, relaxPosR(1), relaxPosR(2)));

						oriTaskR->orientation(relaxRotR);
					}),

					mc_rtc::gui::Button("LEFT ARM half-sit pose", [this, &ctl]()
					{
						posTaskL->position(initPosL + X_0_rel.translation());
						oriTaskL->orientation(initRotL);
					}),

					mc_rtc::gui::Button("RIGHT ARM half-sit pose", [this, &ctl]()
					{
						posTaskR->position(initPosR + X_0_rel.translation());
						oriTaskR->orientation(initRotR);
					})

					);

				/*manually move ef with walking*/
				ctl.gui()->addElement({"Handover", "step Walk"},

					mc_rtc::gui::Label("move this direction->", [this]() { return walkThisDir; }),

					mc_rtc::gui::ArrayInput("move Forward 30cm", {"efLx", "efLy", "efLz", "efRx", "efRy", "efRz"},
						[this]() { return relaxPos; },
						[this, &ctl](const Eigen::Vector6d & to){

							if(stepFwd)
							{
								walkThisDir  = "only BACKWARD allowed";
								relaxPos << to;

								relaxPosL = to.head(3);
								posTaskL->position(relaxPosL);
								oriTaskL->orientation(relaxRotL);

								relaxPosR = to.tail(3);
								posTaskR->position(relaxPosR);
								oriTaskR->orientation(relaxRotR);

								stepFwd = false;
								stepBack = true;

								approachObj->stepSize = "30";
								logStepSize = 30;
								approachObj->walkPlan = "HANDOVER_1stepCycle_fwd_" + approachObj->stepSize + "cm";
								ctl.loadFootstepPlan(approachObj->walkPlan);
								ctl.config().add("triggerWalk", true);
							}
						}),

					mc_rtc::gui::ArrayInput("move Backward 30cm", {"efLx", "efLy", "efLz", "efRx", "efRy", "efRz"},
						[this]() { return relaxPos; },
						[this, &ctl](const Eigen::Vector6d & to){


							if(stepBack)
							{
								walkThisDir  = "only FORWARD allowed";
								relaxPos << to;

								relaxPosL = to.head(3);
								posTaskL->position(relaxPosL);
								oriTaskL->orientation(relaxRotL);

								relaxPosR = to.tail(3);
								posTaskR->position(relaxPosR);
								oriTaskR->orientation(relaxRotR);

								stepBack = false;
								stepFwd = true;

								approachObj->stepSize = "30";
								logStepSize = 30;
								approachObj->walkPlan = "HANDOVER_1stepCycle_back_" + approachObj->stepSize + "cm";
								ctl.loadFootstepPlan(approachObj->walkPlan);
								ctl.config().add("triggerWalk", true);
							}
						}),



					mc_rtc::gui::ArrayInput("move Forward 40cm", {"efLx", "efLy", "efLz", "efRx", "efRy", "efRz"},
						[this]() { return relaxPos; },
						[this, &ctl](const Eigen::Vector6d & to){

							if(stepFwd)
							{
								walkThisDir  = "only BACKWARD allowed";
								relaxPos << to;

								relaxPosL = to.head(3);
								posTaskL->position(relaxPosL);
								oriTaskL->orientation(relaxRotL);

								relaxPosR = to.tail(3);
								posTaskR->position(relaxPosR);
								oriTaskR->orientation(relaxRotR);

								stepFwd = false;
								stepBack = true;

								approachObj->stepSize = "40";
								logStepSize = 40;
								approachObj->walkPlan = "HANDOVER_1stepCycle_fwd_" + approachObj->stepSize + "cm";
								ctl.loadFootstepPlan(approachObj->walkPlan);
								ctl.config().add("triggerWalk", true);
							}
						}),

					mc_rtc::gui::ArrayInput("move Backward 40cm", {"efLx", "efLy", "efLz", "efRx", "efRy", "efRz"},
						[this]() { return relaxPos; },
						[this, &ctl](const Eigen::Vector6d & to){


							if(stepBack)
							{
								walkThisDir  = "only FORWARD allowed";
								relaxPos << to;

								relaxPosL = to.head(3);
								posTaskL->position(relaxPosL);
								oriTaskL->orientation(relaxRotL);

								relaxPosR = to.tail(3);
								posTaskR->position(relaxPosR);
								oriTaskR->orientation(relaxRotR);

								stepBack = false;
								stepFwd = true;

								approachObj->stepSize = "40";
								logStepSize = 40;
								approachObj->walkPlan = "HANDOVER_1stepCycle_back_" + approachObj->stepSize + "cm";
								ctl.loadFootstepPlan(approachObj->walkPlan);
								ctl.config().add("triggerWalk", true);
							}
						}));

				/*publish wrench*/
				ctl.gui()->addElement({"Handover", "wrench"},

					mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {
						cout << "left hand Forces " <<
						ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
						cout << "right hand Forces " <<
						ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
					}),

					mc_rtc::gui::Button("Norm_Force",[this, &ctl](){
						Eigen::Vector3d v = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
						Eigen::Vector3d v2 = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
						cout<<"Norm Fl "<< v.norm() <<endl;
						cout<<"Norm Fr "<< v2.norm() <<endl; }),

					mc_rtc::gui::ArrayInput("set Threshold %",
						{"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
						[this]() { return thresh; },
						[this](const Eigen::VectorXd & t)
						{
							LOG_INFO("Changed threshold to:\nLeft: " << t.head(6).transpose() << "\nRight: " << t.tail(6).transpose() << "\n")
							thresh = t;
						}));

			}

		}// start



		void states::Handover::runState()
		{
			auto & ctl = controller();
			auto & pendulum_ = ctl.pendulum();

			relaxPosL << relaxPos.segment(0,3);
			relaxPosR << relaxPos.segment(3,3);

			/*relative to pelvis*/
			sva::PTransformd X_0_body = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("BODY")];
			X_0_rel = sva::PTransformd( X_0_body.rotation(), X_0_body.translation() - Eigen::Vector3d(0, 0, 0.772319) );


			ltPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")].translation();
			ltRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].rotation();

			rtPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK7")].translation();
			rtRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")].rotation();


			/*get ef(s) acceleration*/
			if( approachObj->i > 3 )
			{
				efLPos[3-g] = ltPosW;
				efRPos[3-g] = rtPosW;
				g++;

				if(g>3)
				{
					g = 1;
					efLVel[0] = (efLPos[1]-efLPos[2])/timeStep;
					efLVel[1] = (efLPos[0]-efLPos[1])/timeStep;
					efLAce = (efLVel[1]-efLVel[0])/timeStep;


					efRVel[0] = (efRPos[1]-efRPos[2])/timeStep;
					efRVel[1] = (efRPos[0]-efRPos[1])/timeStep;
					efRAce = (efRVel[1]-efRVel[0])/timeStep;
				}
			}


			/*Force sensor*/
			leftForce = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
			rightForce = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();


			leftForce_Xabs = abs(leftForce(0));
			leftForce_Yabs = abs(leftForce(1));
			leftForce_Zabs = abs(leftForce(2));


			rightForce_Xabs = abs(rightForce(0));
			rightForce_Yabs = abs(rightForce(1));
			rightForce_Zabs = abs(rightForce(2));


			leftForceSurf = ctl.robot().surfaceWrench("InternLeftHand").force();
			rightForceSurf = ctl.robot().surfaceWrench("InternRightHand").force();


			/*robot object contacts*/
			auto addContact_object_Gripper = [&](std::string gripperName)
			{ ctl.addContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"}); };

			auto removeContact_object_Gripper = [&](std::string gripperName)
			{ ctl.removeContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"}); };

			auto objHasContact = [&](std::string gripperName) -> bool
			{
				auto b = ctl.hasContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"});
				return b;
			};


			if(approachObj->addContacts)
			{
				approachObj->addContacts = false;
				addContact_object_Gripper("LeftGripper");
				addContact_object_Gripper("RightGripper");

				ctl.solver().removeTask(objEfTask);

				ctl.solver().removeTask(posTaskR);
				ctl.solver().removeTask(oriTaskR);
			}


			if(approachObj->removeContacts)
			{
				approachObj->removeContacts = false;
				removeContact_object_Gripper("LeftGripper");
				removeContact_object_Gripper("RightGripper");

				ctl.solver().addTask(objEfTask);

				ctl.solver().addTask(posTaskL);
				ctl.solver().addTask(oriTaskL);

				ctl.solver().addTask(posTaskR);
				ctl.solver().addTask(oriTaskR);

				ctl.postureTask->reset();

				posTaskL->reset();
				oriTaskL->reset();

				posTaskR->reset();
				oriTaskR->reset();
			}

			approachObj->objHasContacts = objHasContact("LeftGripper") && objHasContact("RightGripper");



			auto open_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({openGrippers});

				if(approachObj->FlAG_INDIVIDUAL)
				{
					approachObj->gOpen = false;
					approachObj->openGripper = true;
				}
			};


			auto close_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({closeGrippers});

				if(approachObj->FlAG_INDIVIDUAL)
				{
					approachObj->gClose = false;
				}
			};


			auto gripperControl = [&](std::string gripperName)
			{
				if(approachObj->FlAG_INDIVIDUAL)
				{
					if(approachObj->gOpen)
					{
						open_gripper(gripperName);
					}

					if(approachObj->gClose)
					{
						close_gripper(gripperName);
					}
				}
				else //TOGETHER
				{
					if(approachObj->gOpen)
					{
						open_gripper("l_gripper");
						open_gripper("r_gripper");
						approachObj->openGripper = true;
						approachObj->gOpen = false;
					}

					if(approachObj->gClose)
					{
						close_gripper("l_gripper");
						close_gripper("r_gripper");
						approachObj->gClose = false;
					}
				}
			};


			if(Flag_HandoverRun)
			{

				/*everything dependent on markers goes inside here*/
				if( approachObj->handoverRun() )
				{

					/*object geometric properties*/
					objLen = 0.9;
					objLenLt = objLen/2;
					objLenRt = objLenLt - (approachObj->objectPosC - approachObj->objectPosCy).norm();


					/*object body relative to robot body*/
					objBody_rel_robotBody = abs( approachObj->objectPosC(0) - X_0_rel.translation()(0) );


					/*human robot body distance*/
					bodyPosR = X_0_rel.translation();
					bodyPosS = approachObj->headPos;
					ID = bodyPosS(0)-bodyPosR(0);




					auto robotRtHandOnObj = [&]()-> sva::PTransformd
					{
						al = (approachObj->objectPosC - approachObj->gripperEfR).norm();
						bl = abs(objLenLt - al);

						if(al <= objLenLt)
						{
							if(al>=bl)
							{
								offsetLt << 0.0, -al/2, 0.0;
							}
							else
							{
								offsetLt << 0.0, bl/2, 0.0;
							}

							X_Obj0_offsetS = sva::PTransformd(offsetLt);
							X_M_Obj0 = sva::PTransformd(approachObj->subjLHandRot.transpose(), approachObj->fingerPosL);
							X_M_offsetR = X_Obj0_offsetS * X_M_Obj0;
							return X_M_offsetR;
						}
					};

					auto robotLtHandOnObj = [&]()-> sva::PTransformd
					{
						ar = (approachObj->objectPosCy - approachObj->gripperEfL).norm();
						br = abs(objLenRt - ar);

						if(ar <= objLenRt)
						{
							if(ar>=br)
							{
								offsetRt << 0.0, ar/2, 0.0;
							}
							else
							{
								offsetRt << 0.0, -br/2, 0.0;
							}

							X_Obj0_offsetS = sva::PTransformd(offsetRt);
							X_M_Obj0 = sva::PTransformd(approachObj->subjRHandRot.transpose(), approachObj->fingerPosR);
							X_M_offsetL = X_Obj0_offsetS * X_M_Obj0;
							return X_M_offsetL;
						}
					};



					auto subjLtHandOnObj = [&]()-> sva::PTransformd
					{
						al = (approachObj->objectPosC - approachObj->fingerPosL).norm();
						bl = abs(objLenLt - al);

						if(al <= objLenLt)
						{
							if(al>=bl)
							{
								offsetLt << 0.0, 2*al/3, 0.0;
							}
							else
							{
								offsetLt << 0.0, -2*bl/3, 0.0;
							}
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->fingerPosL);
						}
						else
						{
							//move to center pos of objLenLt/2
							offsetLt << 0.0, -objLenLt/2, 0.0;
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosC);
						}

						X_Obj0_offsetEf = sva::PTransformd(offsetLt);
						X_M_offsetR = X_Obj0_offsetEf * X_M_Obj0;
						return X_M_offsetR;
					};

					auto subjRtHandOnObj = [&]()-> sva::PTransformd
					{
						ar = (approachObj->objectPosCy - approachObj->fingerPosR).norm();
						br = abs(objLenRt - ar);

						if(ar <= objLenRt)
						{
							if(ar>=br)
							{
								offsetRt << 0.0, -2*ar/3, 0.0;
							}
							else
							{
								offsetRt << 0.0, 2*br/3, 0.0;
							}
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->fingerPosR);
						}
						else
						{
							//move to center pos of objLenRt/2
							offsetRt << 0.0, objLenRt/2, 0.0;
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosC);
						}

						X_Obj0_offsetEf = sva::PTransformd(offsetRt);
						X_M_offsetL = X_Obj0_offsetEf * X_M_Obj0;
						return X_M_offsetL;
					};



					/*
					*
					* 4th,
					*
					* check object relative to human hands
					*
					*/
					auto obj_rel_subj = [&]()
					{
						if(approachObj->FlAG_INDIVIDUAL)
						{
							if(approachObj->obj_rel_subjRtHand < approachObj->obj_rel_subjLtHand)
							{
								subjMarkersName = approachObj->subjRtMarkers;
								fingerPos = approachObj->fingerPosR;
							}
							else
							{
								subjMarkersName = approachObj->subjLtMarkers;
								fingerPos = approachObj->fingerPosL;
							}

							objEfTask->set_ef_pose(sva::PTransformd(approachObj->handRot, approachObj->objectPosC));
						}
						else // TOGETHER
						{
							if(approachObj->subjHasObject)
							{
								if(approachObj->bool_t1)
								{
									approachObj->bool_t1 = false;
									approachObj->t1 = difftime( time(0), approachObj->start);
								}

								objEfTask->set_ef_pose(sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosC));

								subjLtHandOnObj();
								subjRtHandOnObj();
							}
							else if( approachObj->robotHasObject && approachObj->pickNearestHand &&
								( (approachObj->finR_rel_efL < MAX_ALLOWED_DIST) || (approachObj->finL_rel_efR < MAX_ALLOWED_DIST) ) )
							{
								if(approachObj->bool_t6)
								{
									approachObj->bool_t6 = false;
									approachObj->t6 = difftime( time(0), approachObj->start);
								}
								subjMarkersName = approachObj->subjRtMarkers;
								approachObj->pickNearestHand = false;
							}
						}
					};




					/*
					* 3rd,
					*
					* track only subj right hand when robot carries the object
					*
					*/
					auto obj_rel_robot = [&]() -> bool
					{
						obj_rel_subj();

						if(approachObj->FlAG_INDIVIDUAL)
						{
							/*this should be relative to robot body*/
							if( ( fingerPos(0) - abs(X_0_rel.translation()(0)) ) < MAX_ALLOWED_DIST /*- 0.2*/ ) //1.2 //old was 0.7
							{

								if( fingerPos(1) >= 0.11 )
								{
									if( (approachObj->stopRtEf) && (approachObj->useLtEf) && (approachObj->selectRobotHand) )
									{
										LOG_SUCCESS("------------------------------> robot 'LEFT' Hand in use")
										approachObj->selectRobotHand = false;

										approachObj->stopRtEf = false;

										// posTaskR->position(initPosR);
										// oriTaskR->orientation(initRotR);
										ctl.solver().removeTask(posTaskR);
										ctl.solver().removeTask(oriTaskR);


										approachObj->stopLtEf = true;
									}

									if(!approachObj->selectRobotHand)
									{
										approachObj->lHandPredict = approachObj->predictionController(
											ltPosW,
											relaxRotL,
											subjMarkersName);

										approachObj->predictPosL = get<4>(approachObj->lHandPredict);
									}
								}
								else
								{
									if( (approachObj->stopLtEf) && (approachObj->useRtEf) && (approachObj->selectRobotHand) )
									{
										LOG_SUCCESS("------------------------------> robot 'RIGHT' Hand in use")
										approachObj->selectRobotHand = false;

										approachObj->stopLtEf = false;

										// posTaskL->position(initPosL);
										// oriTaskL->orientation(initRotL);
										ctl.solver().removeTask(posTaskL);
										ctl.solver().removeTask(oriTaskL);

										approachObj->stopRtEf = true;
									}

									if(!approachObj->selectRobotHand)
									{
										approachObj->rHandPredict = approachObj->predictionController(
											rtPosW,
											relaxRotR,
											subjMarkersName);

										approachObj->predictPosR = get<4>(approachObj->rHandPredict);
									}
								}


								if( (approachObj->bool_t1) && (!approachObj->selectRobotHand) )
								{
									approachObj->bool_t1 = false;
									approachObj->t1 = difftime( time(0), approachObj->start);
								}

							}

							return false;
						}
						else // TOGETHER
						{
							if(approachObj->subjHasObject)
							{
								/*CHECK once every prediction cycle*/
								if( (approachObj->i)%(approachObj->t_observe) == 0 )
								{
									approachObj->rHandPredict = approachObj->predictionController(
										rtPosW,
										relaxRotR,
										approachObj->subjLtMarkers);

									approachObj->useRightEf = get<0>(approachObj->rHandPredict);

									approachObj->lHandPredict = approachObj->predictionController(
										ltPosW,
										relaxRotL,
										approachObj->subjRtMarkers);

									approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
								}
							}
							else if( approachObj->robotHasObject && (!approachObj->pickNearestHand) )
							{
								if( subjMarkersName[0] == "lShapeRtA" )
								{
									approachObj->lHandPredict = approachObj->predictionController(
										ltPosW,
										relaxRotL,
										subjMarkersName);

									approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
									approachObj->useRightEf = false;
								}
								else
								{
									approachObj->rHandPredict = approachObj->predictionController(
										rtPosW,
										relaxRotR,
										subjMarkersName);

									approachObj->useRightEf = get<0>(approachObj->rHandPredict);
									approachObj->useLeftEf = false;
								}
							}

							approachObj->predictPosL = get<4>(approachObj->lHandPredict);
							approachObj->predictPosR = get<4>(approachObj->rHandPredict);
						}

						return false;
					};



					/*
					* 5th,
					*
					* feed Ef pose
					*
					*/
					if(approachObj->FlAG_INDIVIDUAL)
					{
						if( (!approachObj->stopRtEf) )
						{
							approachObj->useRtEf = false;
							robotMarkersName = approachObj->robotLtMarkers;

							approachObj->goToHandoverPose(
								Xmax,
								0.11,
								0.7,
								approachObj->enableLHand,
								ltPosW,
								posTaskL,
								oriTaskL,
								approachObj->lHandPredict,
								fingerPos);

							approachObj->forceControllerIndividual(
								approachObj->enableLHand,
								X_0_rel,
								bodyPosS,
								(relaxPosL + X_0_rel.translation()),
								(initPosL + X_0_rel.translation()),
								initRotL,
								relaxRotL,
								leftForce,
								leftForceSurf,
								leftTh,
								efLAce,
								posTaskL,
								oriTaskL,
								"l_gripper",
								robotMarkersName,
								subjMarkersName,
								approachObj->obj_rel_robotLtHand);

							gripperControl("l_gripper");
						}
						else if( (!approachObj->stopLtEf) )
						{
							approachObj->useLtEf = false;
							robotMarkersName = approachObj->robotRtMarkers;

							approachObj->goToHandoverPose(
								Xmax,
								-0.7,
								0.10,
								approachObj->enableRHand,
								rtPosW,
								posTaskR,
								oriTaskR,
								approachObj->rHandPredict,
								fingerPos);

							approachObj->forceControllerIndividual(
								approachObj->enableRHand,
								X_0_rel,
								bodyPosS,
								(relaxPosR + X_0_rel.translation()),
								(initPosR + X_0_rel.translation()),
								initRotR,
								relaxRotR,
								rightForce,
								rightForceSurf,
								rightTh,
								efRAce,
								posTaskR,
								oriTaskR,
								"r_gripper",
								robotMarkersName,
								subjMarkersName,
								approachObj->obj_rel_robotRtHand);

							gripperControl("r_gripper");
						}
					}
					else //TOGETHER
					{
						if( (approachObj->useLeftEf) || (approachObj->useRightEf) )
						{
							if(approachObj->subjHasObject)
							{
								updateOffsetPosL = X_M_offsetL.translation();
								approachObj->goToHandoverPose(
									Xmax,
									-0.15,
									0.75,
									approachObj->enableHand,
									ltPosW,
									posTaskL,
									oriTaskL,
									approachObj->lHandPredict,
									updateOffsetPosL);


								updateOffsetPosR = X_M_offsetR.translation();
								approachObj->goToHandoverPose(
									Xmax,
									-0.75,
									0.15,
									approachObj->enableHand,
									rtPosW,
									posTaskR,
									oriTaskR,
									approachObj->rHandPredict,
									updateOffsetPosR);
							}
							else if( approachObj->robotHasObject && (!approachObj->pickNearestHand) )
							{
								if(approachObj->useLeftEf)
								{
									// robotLtHandOnObj();
									// updateOffsetPosL = X_M_offsetL.translation();

									updateOffsetPosL = approachObj->fingerPosR;

									approachObj->goToHandoverPose(
										Xmax,
										-0.15,
										0.75,
										approachObj->enableHand,
										ltPosW,
										posTaskL,
										oriTaskL,
										approachObj->lHandPredict,
										updateOffsetPosL);
								}
								else if(approachObj->useRightEf)
								{
									// robotRtHandOnObj();
									// updateOffsetPosR = X_M_offsetR.translation();

									updateOffsetPosR = approachObj->fingerPosL;

									approachObj->goToHandoverPose(
										Xmax,
										-0.75,
										0.15,
										approachObj->enableHand,
										rtPosW,
										posTaskR,
										oriTaskR,
										approachObj->rHandPredict,
										updateOffsetPosR);
								}
							}

							/*check both gripper forces together*/
							approachObj->forceControllerTogether(
								approachObj->enableHand,
								X_0_rel,
								(initPosR + X_0_rel.translation()), initRotR,
								(initPosL + X_0_rel.translation()), initRotL,
								(relaxPosR + X_0_rel.translation()), relaxRotR,
								(relaxPosL + X_0_rel.translation()), relaxRotL,
								thresh,
								leftForce, rightForce,
								leftForceSurf, rightForceSurf,
								efLAce, efRAce,
								posTaskL, oriTaskL,
								posTaskR, oriTaskR);

							gripperControl("bothGrippers");
						}
					}





					/*
					* 1st, TRIGGER HANDOVER ROUTINE
					*
					* Trigger only when both object & human come to 'START ZONE'
					*
					*/
					if( (!approachObj->startNow) &&

						(approachObj->objectPosC(0) > MIN_ALLOWED_DIST) && //0.1
						(approachObj->objectPosC(0) < START_ZONE_DIST) 	&& //1.4

						(approachObj->fingerPosL(0) > MIN_ALLOWED_DIST) && //0.1
						(approachObj->fingerPosL(0) < START_ZONE_DIST) 	&& //1.4

						(approachObj->fingerPosR(0) > MIN_ALLOWED_DIST) &&
						(approachObj->fingerPosR(0) < START_ZONE_DIST) )
					{

						approachObj->cycle_1st = true;
						approachObj->cycle_2nd = false;

						// if(approachObj->objectPosC(2) >= approachObj->objAboveWaist)
						// {
							approachObj->startNow = true;
							approachObj->enableWalk = false;

							ctl.solver().addTask(posTaskL);
							ctl.solver().addTask(oriTaskL);

							ctl.solver().addTask(posTaskR);
							ctl.solver().addTask(oriTaskR);

							LOG_SUCCESS("------------------------------> Handover Routine TRIGGERED")

						// }

					}





					/*
					* 2nd then non-stop
					*
					* as long as object is within robot's reachable space
					*
					*/
					if(approachObj->startNow)
					{

						/*when walk Fwd -- this condition must satisfy --- otherwise routine wont work*/
						if( (objBody_rel_robotBody >= ZERO) && //0.0
							(objBody_rel_robotBody <= START_ZONE_DIST) ) //1.4 //MAX_ALLOWED_DIST //1.2
						{
							obj_rel_robot();
						}



						/*
						* 2nd (a)
						*/
						if( (!approachObj->enableWalk) && (!approachObj->walkFwd) )
						{

							if(approachObj->cycle_1st)
							{
								if(approachObj->objectPosC(2) >= approachObj->objAboveWaist)
								{
									isHumanReady = true;
								}
							}
							else if(approachObj->cycle_2nd)
							{
								if( (approachObj->fingerPosL(2) >= approachObj->objAboveWaist) ||
									(approachObj->fingerPosR(2) >= approachObj->objAboveWaist) )
								{
									isHumanReady = true;
								}
							}

							if(isHumanReady)
							{
								/*check where human is standing w.r.t robot*/
								if(	(bodyPosS(0) > START_ZONE_DIST) && //1.4
									(bodyPosS(0) < SAFE_ZONE_DIST) ) //&&  //1.8

									// (approachObj->objectPosC(0) > MIN_ALLOWED_DIST) && //0.1
									// (approachObj->objectPosC(0) < START_ZONE_DIST)	) //1.4
								{
									approachObj->human_far = true;
									approachObj->human_near = false;
									// LOG_WARNING("farrrrrrrrrrrrr")
								}
								else
								{
									approachObj->human_near = true;
									approachObj->human_far = false;
									// LOG_SUCCESS("nearrrrrrrrrrrr")
									Xmax = 0.8;
								}


								auto where_Does_Human_Stand = [&]()
								{
									if(approachObj->human_near)
									{
										approachObj->enableWalk = false;
										approachObj->walkFwd = false;
									}
									else if(approachObj->human_far)
									{
										approachObj->enableWalk = true;
										approachObj->walkFwd = true;
									}
								};

								where_Does_Human_Stand();

								isHumanReady = false;
							}

						}
						/*
						* 2nd (b)
						*
						*trigger step-walk only when walk Fwd is enabled*/
						else if( (approachObj->enableWalk) && (approachObj->walkFwd) )
						{
							if( (ID > START_ZONE_DIST) && (ID < SAFE_ZONE_DIST) ) //1.8>ID>1.4
							{
								if(approachObj->objectPosC(2) >= approachObj->objAboveWaist)
								{
									approachObj->walkFwd = false;

									if(approachObj->Flag_WALK)
									{
										approachObj->stepSize = "20cm_10";
										// approachObj->stepSize = "20cm_20";
										logStepSize = 0.30;
										Xmax = 0.80 + 0.1 + logStepSize;

										approachObj->walkPlan = "HANDOVER_fwd_" + approachObj->stepSize + "cm";
										LOG_ERROR("------------------------------> FWD walking triggered with PLAN ---> "<< approachObj->walkPlan)

										ctl.loadFootstepPlan(approachObj->walkPlan);
										ctl.config().add("triggerWalk", true);
									}
									else
									{
										Xmax = 0.8;
									}
								}
							}
						}

					}// startNow




					/*
					* always, non-stop
					*
					* head visual tracking
					*
					*/
					if(Flag_HeadTracking)
					{

						if(approachObj->subjHasObject)
						{
							headTask->target(approachObj->objectPosC);
						}
						else
						{
							if(approachObj->FlAG_INDIVIDUAL)
							{
								headTask->target(fingerPos);
							}
							else //TOGETHER
							{
								headTask->target(approachObj->fingerPosR);
							}
						}

					}

				}// handoverRun

			}




			/*
			*
			* Reset handover flags and end-effectors flags
			*
			*/
			if(resetFlags_and_efPose)
			{

				cout<<"\033[1;33m------------------------------> ***handover reset triggered***\033[0m\n";
				/*COMMON FOR BOTH HANDOVER ROUTINE*/

				/*reset head*/
				ctl.solver().removeTask(headTask);
				headTask.reset(new mc_tasks::LookAtTask(
					"HEAD_LINK1", headVector, headTarget, ctl.robots(), ctl.robots().robotIndex(), 2., 500.));
				ctl.solver().addTask(headTask);
				headTask->target(Eigen::Vector3d(0.5, 0., 1.3));
				headTask->selectActiveJoints(ctl.solver(), activeJointsName);

				/*reset moap markers*/
				for(unsigned int d=0; d<approachObj->totalMarkers; d++)
				{
					approachObj->Markers[d] << Eigen::Vector3d(0, 0, 0);
				}

				relaxPosL << 0.20, 0.35, 1.0;
				relaxPosR << 0.20, -0.35, 1.0;

				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();

				gripperL->setTargetQ({closeGrippers});
				gripperR->setTargetQ({closeGrippers});

				approachObj->e = 1;

				approachObj->startNow = false;

				approachObj->gClose = false;
				approachObj->gOpen = false;
				approachObj->openGripper = false;
				approachObj->closeGripper = false;

				approachObj->graspObject = true;
				approachObj->goBackInitPose = true;
				approachObj->takeBackObject = false;
				approachObj->restartHandover = false;

				approachObj->walkFwdAgain = false;

				ID = 0.0;
				bodyPosR = Eigen::Vector3d::Zero();
				bodyPosS = Eigen::Vector3d::Zero();

				leftForce_Xabs = 0.0;
				leftForce_Yabs = 0.0;
				leftForce_Zabs = 0.0;
				rightForce_Xabs = 0.0;
				rightForce_Yabs = 0.0;
				rightForce_Zabs = 0.0;

				approachObj->t1 = 0.0;
				approachObj->t2 = 0.0;
				approachObj->t3 = 0.0;
				approachObj->t4 = 0.0;
				approachObj->t5 = 0.0;
				approachObj->t6 = 0.0;
				approachObj->t7 = 0.0;
				approachObj->t8 = 0.0;
				approachObj->t9 = 0.0;
				approachObj->t_falseClose = 0.0;

				approachObj->bool_t1 = true;
				approachObj->bool_t6 = true;

				approachObj->rh_fail = true;

				approachObj->count_reset++;

				approachObj->efLPosOfHandover = Eigen::Vector3d::Zero();
				approachObj->efRPosOfHandover = Eigen::Vector3d::Zero();
				approachObj->hLPosOfHandover  = Eigen::Vector3d::Zero();
				approachObj->hRPosOfHandover  = Eigen::Vector3d::Zero();


				approachObj->subjHasObject = true;
				approachObj->robotHasObject = false;


				approachObj->cycle_1st = false;
				approachObj->cycle_2nd = false;
				approachObj->human_near = false;
				approachObj->human_far = false;

				isHumanReady = false;

				ctl.solver().addTask(objEfTask);

				ctl.solver().addTask(posTaskL);
				ctl.solver().addTask(oriTaskL);

				ctl.solver().addTask(posTaskR);
				ctl.solver().addTask(oriTaskR);



				/*SPECIFIC TO INDIVIDUAL HANDOVER*/

				if(approachObj->FlAG_INDIVIDUAL)
				{
					if(!approachObj->stopRtEf)
					{
						posTaskL->position(initPosL + X_0_rel.translation());
						oriTaskL->orientation(initRotL);
					}
					else if(!approachObj->stopLtEf)
					{
						posTaskR->position(initPosR + X_0_rel.translation());
						oriTaskR->orientation(initRotR);
					}

					approachObj->selectRobotHand = true;

					approachObj->useLtEf = true;
					approachObj->stopLtEf = true;

					approachObj->useRtEf = true;
					approachObj->stopRtEf = true;

					approachObj->objMassZ = 0.2;
					approachObj->objMassNorm = 0.2;

					approachObj->enableLHand = true;
					approachObj->enableRHand = true;

					approachObj->tryToPull = false;

					approachObj->localSurf_Fzero = Eigen::Vector3d::Zero();
					approachObj->newTh = Eigen::Vector3d::Zero();
					approachObj->Finert = Eigen::Vector3d::Zero();
					approachObj->Fzero = Eigen::Vector3d::Zero();
					approachObj->Fclose = Eigen::Vector3d::Zero();
					approachObj->Fload = Eigen::Vector3d::Zero();
					approachObj->Fpull = Eigen::Vector3d::Zero();
				}
				else //TOGETHER
				{
					/*remove contacts*/
					ctl.removeContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
					ctl.removeContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});

					posTaskL->position(initPosL + X_0_rel.translation());
					oriTaskL->orientation(initRotL);

					posTaskR->position(initPosR + X_0_rel.translation());
					oriTaskR->orientation(initRotR);

					approachObj->objMassZ = 0.5;
					approachObj->objMassNorm = 0.5;

					approachObj->useLeftEf = false;
					approachObj->useRightEf = false;

					approachObj->addContacts = false;
					approachObj->removeContacts = false;

					approachObj->pickNearestHand = true;

					approachObj->handoverComplete = false;

					approachObj->enableHand = true;

					approachObj->localSurf_FzeroL = Eigen::Vector3d::Zero();
					approachObj->newThL = Eigen::Vector3d::Zero();
					approachObj->FinertL = Eigen::Vector3d::Zero();
					approachObj->FzeroL = Eigen::Vector3d::Zero();
					approachObj->FcloseL = Eigen::Vector3d::Zero();
					approachObj->FloadL = Eigen::Vector3d::Zero();
					approachObj->FpullL = Eigen::Vector3d::Zero();


					approachObj->localSurf_FzeroR = Eigen::Vector3d::Zero();
					approachObj->newThR = Eigen::Vector3d::Zero();
					approachObj->FinertR = Eigen::Vector3d::Zero();
					approachObj->FzeroR = Eigen::Vector3d::Zero();
					approachObj->FcloseR = Eigen::Vector3d::Zero();
					approachObj->FloadR = Eigen::Vector3d::Zero();
					approachObj->FpullR = Eigen::Vector3d::Zero();
				}


				/*SPECIFIC TO WALKING*/
				approachObj->enableWalk = false;

				approachObj->walkFwd = false;
				approachObj->walkBack = false;

				stepFwd = true;
				stepBack = true;

				approachObj->finishedWalk_ = false;
				ctl.config().add("finishedWalk", false);

				resetFlags_and_efPose = false;
				cout<<"\033[1;33m------------------------------> ***ready to start new handover routine***\033[0m\n";

			} // resetFlags_and_efPose




		}// runState






		bool states::Handover::checkTransitions()
		{
			// auto & ctl = controller();
			return false;
		}// checkTransition


		void states::Handover::teardown()
		{
			auto & ctl = controller();

			cout << "teardown Handover\n";

			if(Flag_HandoverGUI)
			{
				ctl.gui()->removeCategory({"Handover"});
			}

			if(Flag_HandoverTasks)
			{
				ctl.solver().removeTask(headTask);
				ctl.solver().removeTask(posTaskL);
				ctl.solver().removeTask(posTaskR);
				ctl.solver().removeTask(oriTaskL);
				ctl.solver().removeTask(oriTaskR);
				ctl.solver().removeTask(objEfTask);
			}
		}// teardown




	} // namespace states

} // namespace lipm_walking

EXPORT_SINGLE_STATE("Handover", lipm_walking::states::Handover)


