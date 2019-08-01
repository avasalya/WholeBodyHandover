#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states
	{
		void MyErrorMsgHandler(int iLevel, const char *szMsg)
		{
			const char *szLevel = NULL;

			if (iLevel == VL_Debug) { szLevel = "Debug"; }
			else if (iLevel == VL_Info) { szLevel = "Info"; }
			else if (iLevel == VL_Warning) { szLevel = "Warning"; }
			else if (iLevel == VL_Error) { szLevel = "Error"; }
			printf("  %s: %s\n", szLevel, szMsg);
		}


		void StartMocapStep::ros_spinner()
		{	ros::spin();	}


		void StartMocapStep::cortexCallback(const cortex_ros_bridge_msgs::Markers & msg)
		{
			// LOG_WARNING("cortexCallback")
			c = 0;
			for(unsigned int d=0; d<msg.markers.size(); d++)
			{
				if(msg.markers.at(d).marker_name== "dummy")
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


		void StartMocapStep::start(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*current time*/
			time_t now = time(0);
			char* dt = ctime(&now);
			cout << "The local date and time is: " << dt << endl;


			/*allocate memory*/
			approachObj = std::make_shared<mc_handover::ApproachObject>();
			approachObj->initials();


			/*close grippers*/
			auto  gripperL = ctl.grippers["l_gripper"].get();
			auto  gripperR = ctl.grippers["r_gripper"].get();
			gripperL->setTargetQ({closeGrippers});
			gripperR->setTargetQ({closeGrippers});


			constRotL<<
			-0.1624, -0.0616,   0.974,
			0.092,  0.9926,  0.0784,
			-0.9716,   0.104, -0.1554;
			constRotR<<
			0.013618,  0.0805374,   0.996659,
			-0.243029,   0.967128, -0.0748305,
			-0.969923,  -0.241198,  0.0327433;

			constPosL<<  0.20, 0.3, 0.9;
			constPosR<<  0.20, -0.3, 0.9;

			/*initial force/torque threshold*/
			thresh << 10, 10, 10, 4, 4, 4, 10, 10, 10, 4, 4, 4;
			leftTh = thresh.segment(3,3);
			rightTh = thresh.segment(9,3);


			/*HeadTask*/
			headVector<<1., 0., 0.;
			headTarget<<1., 0., 0.;
			headTask.reset(new mc_tasks::LookAtTask("HEAD_LINK1", headVector, headTarget, ctl.robots(), ctl.robots().robotIndex(), 2., 500.));
			ctl.solver().addTask(headTask);
			headTask->selectActiveJoints(ctl.solver(), activeJointsName);


			/*chest pos task*/
			chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestPosTask);
			chestPosTask->position({0.032, 0.0, 1.12});

			/*chest ori task*/
			chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestOriTask);
			chestOriTask->orientation(Eigen::Matrix3d::Identity());


			/*body pos task*/
			bodyPosTask.reset(new mc_tasks::PositionTask("BODY", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(bodyPosTask);

			/*body ori task*/
			bodyOriTask.reset(new mc_tasks::OrientationTask("BODY", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(bodyOriTask);
			bodyOriTask->orientation(Eigen::Matrix3d::Identity());


			/*Ef pos Tasks*/
			posTaskL = make_shared<mc_tasks::PositionTask>("LARM_LINK7", Eigen::Vector3d(0.0, 0.0, -0.09), ctl.robots(), 0, 4.0, 1e3);
			ctl.solver().addTask(posTaskL);
			initPosL = posTaskL->position();

			posTaskR = make_shared<mc_tasks::PositionTask>("RARM_LINK7", Eigen::Vector3d(0.0, 0.0, -0.09), ctl.robots(), 0, 4.0, 1e3);
			ctl.solver().addTask(posTaskR);
			initPosR = posTaskR->position();


			/*EfL ori Task*/
			oriTaskL = make_shared<mc_tasks::OrientationTask>("LARM_LINK6",ctl.robots(), 0, 4.0, 500);
			ctl.solver().addTask(oriTaskL);
			initRotL = oriTaskL->orientation();

			oriTaskR = make_shared<mc_tasks::OrientationTask>("RARM_LINK6",ctl.robots(), 0, 4.0, 500);
			ctl.solver().addTask(oriTaskR);
			initRotR = oriTaskR->orientation();


			/*handover endEffectorTask*/
			objEfTask = make_shared<mc_tasks::EndEffectorTask>("base_link", ctl.robots(), 2, 2.0, 1e3);
			ctl.solver().addTask(objEfTask);
			objEfTask->set_ef_pose(sva::PTransformd(Eigen::Vector3d(0.5, 0.0, 1.1)));


			/*restart mocapStep*/
			ctl.gui()->addElement({"Handover", "Restart"},
				mc_rtc::gui::Button("restartHandover", [this]()
					{restartEverything = true; }));

			/*Motion FOR CREATING MOCAP TEMPLATE*/
			ctl.gui()->addElement({"Handover", "Reset Pose"},

				mc_rtc::gui::Button("LEFT ARM orientation", [this, &ctl]()
				{
					oriTaskL->orientation(constRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM orientation", [this, &ctl]()
				{
					oriTaskR->orientation(constRotR);
				}),

				mc_rtc::gui::Button("LEFT ARM half-sit", [this, &ctl]()
				{
					posTaskL->position(initPosL);
					oriTaskL->orientation(initRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM half-sit", [this, &ctl]()
				{
					posTaskR->position(initPosR);
					oriTaskR->orientation(initRotR);
				})

				);

			/*publish wrench*/
			ctl.gui()->addElement({"Handover", "wrench"},

				mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {
					cout << "left hand Forces " <<
					ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
					cout << "right hand Forces " <<
					ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
				}),

				mc_rtc::gui::Button("Norm_LeftEf_Force",[this, &ctl](){
					Eigen::Vector3d v = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					Eigen::Vector3d v2 = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					cout<<"Norm Fl"<< v.norm() <<endl;
					cout<<"Norm Fr"<< v2.norm() <<endl; }),

				mc_rtc::gui::ArrayInput("set Threshold %",
					{"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
					[this]() { return thresh; },
					[this](const Eigen::VectorXd & t)
					{
						LOG_INFO("Changed threshold to:\nLeft: " << t.head(6).transpose() << "\nRight: " << t.tail(6).transpose() << "\n")
						thresh = t;
					}));

			/*change prediction_ settings*/
			ctl.gui()->addElement({"Handover", "tuner"},
				mc_rtc::gui::ArrayInput("t_predict/t_observe", {"t_predict", "t_observe", "it"},
					[this]() { return approachObj->tuner; },
					[this](const Eigen::Vector3d & to){approachObj->tuner = to;cout<< "t_predict = " << approachObj->tuner(0)*1/fps<< "sec, t_observe = "<<approachObj->tuner(1)*1/fps<< "sec"<<endl;}));


			/*display moving object*/
			ctl.gui()->addElement({"Handover","object pose"},
				mc_rtc::gui::Transform("object position",
					[this,&ctl] () { ctl.robots().robot(2).posW(objEfTask->get_ef_pose());
					return ctl.robots().robot(2).bodyPosW("base_link");
				}));


			// /*move object using cursor or simData*/
			// ctl.gui()->addElement({"Handover","cursor_moves_object"},
			// 	mc_rtc::gui::Transform("Position base",
			// 		[this,&ctl](){ return ctl.robots().robot(2).bodyPosW("base_link"); },
			// 		[this,&ctl] (const sva::PTransformd & pos) { ctl.robots().robot(2).posW(pos); }
			// 		));


			/*trajectory trail*/
			// ctl.gui()->addElement({"Handover", "Trajectories"},
				// mc_rtc::gui::Trajectory("traj_l_wrist", {{1,0,1}, 0.01, mc_rtc::gui::LineStyle::Dotted},
				// 	[this,&ctl](){ return ctl.robot().bodyPosW("LARM_LINK7").translation(); })
				// );

			/*com Task*/
			ctl.gui()->addElement({"Handover", "com"},

				mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
					[this]() { return move; },
					[this](const Eigen::Vector3d v) { move = v;
						cout << " com pos set to:\n" << initialCom + move << endl;})
				);

			comTask = make_shared<mc_tasks::CoMTask>(ctl.robots(), ctl.robots().robotIndex(), 10., 1e5);
			initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
			comTask->com(initialCom);
			ctl.solver().addTask(comTask);



			/*configure MOCAP*/
			if(Flag_CORTEX)
			{
				cout << "\033[1;32m ***MOCAP IS ENABLED*** \033[0m\n";
				Cortex_SetVerbosityLevel(VL_Info);
				Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);

				if(ctl.Flag_ROBOT)
					{ retval = Cortex_Initialize("10.1.1.180", "10.1.1.190"); }
				else{ retval = Cortex_Initialize("10.1.1.200", "10.1.1.190"); }

				if (retval != RC_Okay)
					{ printf("Error: Unable to initialize ethernet communication\n");
				retval = Cortex_Exit(); }

				// cortex frame rate //
				printf("\n****** Cortex_FrameRate ******\n");
				retval = Cortex_Request("GetContextFrameRate", &pResponse, &nBytes);
				if (retval != RC_Okay)
					printf("ERROR, GetContextFrameRate\n");
				float *contextFrameRate = (float*) pResponse;
				printf("ContextFrameRate = %3.1f Hz\n", *contextFrameRate);

				// get name of bodies being tracked and its set of markers //
				printf("\n****** Cortex_GetBodyDefs ******\n");
				pBodyDefs = Cortex_GetBodyDefs();
				if (pBodyDefs == NULL)
					{ printf("Failed to get body defs\n"); }
				else
				{
					totalBodies = pBodyDefs->nBodyDefs;
					cout << "total no of bodies tracked " << totalBodies << endl;
					for(int iBody=0; iBody<totalBodies; iBody++)
					{
						bodyMarkers.push_back(pBodyDefs->BodyDefs[iBody].nMarkers);
						pBody = &pBodyDefs->BodyDefs[iBody];
						cout << "number of markers defined in body " << iBody+1 << " (\"" << pBody->szName << "\") : " << bodyMarkers.at(iBody) << endl;

						for (int iMarker=0 ; iMarker<pBody->nMarkers; iMarker++)
							{ cout << iMarker << " " << pBody->szMarkerNames[iMarker] << endl; }
					}
				}
				printf("\n*** start live mode ***\n");
				Cortex_Request("LiveMode", &pResponse, &nBytes);
			}
			else
			{
				cout << "\033[1;32m ***ROS_MOCAP_BRIDGE IS ENABLED*** \033[0m\n";
				m_nh_ = mc_rtc::ROSBridge::get_node_handle();
				if(!m_nh_)
				{
				LOG_ERROR_AND_THROW(std::runtime_error, "This controller does not work withtout ROS")
				}
				m_ros_spinner_ = std::thread{[this](){ this->ros_spinner(); }};
				l_shape_sub_ = m_nh_->subscribe("novis_markers", 1, & mc_handover::states::StartMocapStep::cortexCallback, this);
			}



			/*specific logs*/
			ctl.logger().addLogEntry("logs-HANDOVER_posTaskL", [this]() -> Eigen::Vector3d { return posTaskL->position(); });
			ctl.logger().addLogEntry("logs-HANDOVER_posTaskR", [this]() -> Eigen::Vector3d { return posTaskR->position(); });

			ctl.logger().addLogEntry("logs-HANDOVER_obj mass",[this]() -> double { return approachObj->objMass; });

			ctl.logger().addLogEntry("logs-HANDOVER_bool enableLHand",[this]() -> double { return approachObj->enableLHand; });
			ctl.logger().addLogEntry("logs-HANDOVER_bool enableRHand",[this]() -> double { return approachObj->enableRHand; });

			ctl.logger().addLogEntry("logs-HANDOVER_bool useLtEf",[this]() -> double { return approachObj->useLtEf; });
			ctl.logger().addLogEntry("logs-HANDOVER_bool stopLtEf",[this]() -> double { return approachObj->stopLtEf; });

			ctl.logger().addLogEntry("logs-HANDOVER_bool useRtEf",[this]() -> double { return approachObj->useRtEf; });
			ctl.logger().addLogEntry("logs-HANDOVER_bool stopRtEf",[this]() -> double { return approachObj->stopRtEf; });

			ctl.logger().addLogEntry("logs-HANDOVER_objectPos",[this]()-> Eigen::Vector3d { return approachObj->objectPos; });
			ctl.logger().addLogEntry("logs-HANDOVER_subjFinL",[this]() -> Eigen::Vector3d { return approachObj->fingerPosL; });
			ctl.logger().addLogEntry("logs-HANDOVER_subjFinR",[this]() -> Eigen::Vector3d { return approachObj->fingerPosR; });

			ctl.logger().addLogEntry("logs-HANDOVER_Fzero",[this]() -> Eigen::Vector3d { return approachObj->Fzero; });
			ctl.logger().addLogEntry("logs-HANDOVER_Fclose",[this]() -> Eigen::Vector3d { return approachObj->Fclose; });
			ctl.logger().addLogEntry("logs-HANDOVER_efL Ace",[this]() -> Eigen::Vector3d { return efLAce; });
			ctl.logger().addLogEntry("logs-HANDOVER_efR Ace",[this]() -> Eigen::Vector3d { return efRAce; });
			ctl.logger().addLogEntry("logs-HANDOVER_Finert",[this]() -> Eigen::Vector3d { return approachObj->Finert; });
			ctl.logger().addLogEntry("logs-HANDOVER_Fload",[this]() -> Eigen::Vector3d { return approachObj->Fload; });
			ctl.logger().addLogEntry("logs-HANDOVER_Fpull",[this]() -> Eigen::Vector3d { return approachObj->Fpull; });

			ctl.logger().addLogEntry("logs-HANDOVER_new thresh",[this]() -> Eigen::Vector3d { return approachObj->newTh; });
			ctl.logger().addLogEntry("logs-HANDOVER_right Thresh",[this]() -> Eigen::Vector3d { return rightTh; });
			ctl.logger().addLogEntry("logs-HANDOVER_left Thresh",[this]() -> Eigen::Vector3d { return leftTh; });

			ctl.logger().addLogEntry("logs-HANDOVER_bool grippper open",[this]() -> double { return approachObj->gOpen; });
			ctl.logger().addLogEntry("logs-HANDOVER_bool grippper close",[this]() -> double { return approachObj->gClose; });


			ctl.logger().addLogEntry("logs-HANDOVER_leftHandForce-ABS_X",[this]() -> double { return leftForce_Xabs; });
			ctl.logger().addLogEntry("logs-HANDOVER_rightHandForce-ABS_X",[this]() -> double { return rightForce_Xabs; });

			ctl.logger().addLogEntry("logs-HANDOVER_leftHandForce-ABS_Y",[this]() -> double { return leftForce_Yabs; });
			ctl.logger().addLogEntry("logs-HANDOVER_rightHandForce-ABS_Y",[this]() -> double { return rightForce_Yabs; });

			ctl.logger().addLogEntry("logs-HANDOVER_leftHandForce-ABS_Z",[this]() -> double { return leftForce_Zabs; });
			ctl.logger().addLogEntry("logs-HANDOVER_rightHandForce-ABS_Z",[this]() -> double { return rightForce_Zabs; });


			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t1",[this]() -> double { return approachObj->t1; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t2",[this]() -> double { return approachObj->t2; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t3",[this]() -> double { return approachObj->t3; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t4",[this]() -> double { return approachObj->t4; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t5",[this]() -> double { return approachObj->t5; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t6",[this]() -> double { return approachObj->t6; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t7",[this]() -> double { return approachObj->t7; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t8",[this]() -> double { return approachObj->t8; });
			ctl.logger().addLogEntry("logs-HANDOVER_elapsed_t9",[this]() -> double { return approachObj->t9; });

			ctl.logger().addLogEntry("logs-HANDOVER_Trials_hr-success",[this]() -> double { return approachObj->count_hr_success; });
			ctl.logger().addLogEntry("logs-HANDOVER_Trials_hr-fail",[this]() -> double { return approachObj->count_hr_fail; });
			ctl.logger().addLogEntry("logs-HANDOVER_Trials_rh-success",[this]() -> double { return approachObj->count_rh_success; });
			ctl.logger().addLogEntry("logs-HANDOVER_Trials_rh-fail",[this]() -> double { return approachObj->count_rh_fail; });
			ctl.logger().addLogEntry("logs-HANDOVER_Trials_reset",[this]() -> double { return approachObj->count_reset; });


			ctl.logger().addLogEntry("logs-HANDOVER_bodyPosRobot",[this]() -> Eigen::Vector3d { return bodyPosR; });
			ctl.logger().addLogEntry("logs-HANDOVER_bodyPosSubj",[this]() -> Eigen::Vector3d { return bodyPosS; });
			ctl.logger().addLogEntry("logs-HANDOVER_bodies-distX",[this]() -> double { return bodiesDiffX; });



			LOG_SUCCESS("******** Both hands INDIVIDUAL scenario *********")

		}// start




		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*hand pose*/
			ltPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")].translation();
			ltRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].rotation();

			rtPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK7")].translation();
			rtRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")].rotation();

			/*Force sensor*/
			leftForce = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
			rightForce = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();


			leftForce_Xabs = abs(leftForce(0));
			leftForce_Yabs = abs(leftForce(1));
			leftForce_Zabs = abs(leftForce(2));


			rightForce_Xabs = abs(rightForce(0));
			rightForce_Yabs = abs(rightForce(1));
			rightForce_Zabs = abs(rightForce(2));


			leftForceLo = ctl.robot().surfaceWrench("InternLeftHand").force();
			rightForceLo = ctl.robot().surfaceWrench("InternRightHand").force();


			/*chest pos and chest/body joint ori*/
			chestPosTask->position({0.032, 0.0, 1.12});
			chestOriTask->orientation(Eigen::Matrix3d::Identity());
			bodyOriTask->orientation(Eigen::Matrix3d::Identity());


			/*human robot body distance*/
			if(approachObj->i>1)
			{
				bodyPosR = bodyPosTask->position();
				bodyPosS = approachObj->headPos;

				bodiesDiffX = bodyPosS(0)-bodyPosR(0);
			}


			efLPos.resize(3);
			efLVel.resize(2);

			efRPos.resize(3);
			efRVel.resize(2);

			/*get ef(s) acceleration*/
			if( approachObj->i > 3 )
			{
				efLPos[3-g] = ltPosW;
				efRPos[3-g] = rtPosW;
				g++;

				if(g>3)
				{
					g = 1;
					efLVel[0] = (efLPos[1]-efLPos[2])*fps;
					efLVel[1] = (efLPos[0]-efLPos[1])*fps;
					efLAce = (efLVel[1]-efLVel[0])*fps;


					efRVel[0] = (efRPos[1]-efRPos[2])*fps;
					efRVel[1] = (efRPos[0]-efRPos[1])*fps;
					efRAce = (efRVel[1]-efRVel[0])*fps;
				}
			}


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			auto open_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({openGrippers});

				approachObj->gOpen = false;
				approachObj->openGripper = true;
			};


			auto close_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({closeGrippers});

				approachObj->gClose = false;
			};


			auto gripperControl = [&](std::string gripperName)
			{
				if(approachObj->gOpen)
				{
					open_gripper(gripperName);
				}

				if(approachObj->gClose)
				{
					close_gripper(gripperName);
				}
			};


			/*Get non-stop MOCAP Frame*/
			if(Flag_CORTEX)
			{
				getCurFrame = Cortex_GetCurrentFrame();
				Cortex_CopyFrame(getCurFrame, &FrameofData);

				int ithFrame = FrameofData.iFrame;
				del+=FrameofData.fDelay;

				if(ithFrame == 0 || ithFrame == 1)
					{ startCapture = true; }
				else if(ithFrame <0)
				{
					Cortex_Request("Pause", &pResponse, &nBytes);
					Cortex_Exit();
					output("OK");
					return true;
				}
			}
			else
			{ startCapture = true; }


			/*start only when ithFrame == 1*/
			if(startCapture)
			{

				if(Flag_CORTEX)
				{
					/*get markers position FrameByFrame*/
					if(approachObj->Flag_withoutRobot)
						{ b_ = 2; c = 8; }
					else
						{ b_ = 0; c = 0; }

					for(int b=b_; b<totalBodies; b++)
					{
						/*make sure mocap template body marker's index are correct*/
						pBody = &pBodyDefs->BodyDefs[b];
						if(pBody->szName == approachObj->strMarkersBodyName[b])
						{
							// LOG_INFO("body name: "<<pBody->szName<<"\n"<<
							// 	" pBody->nMarkers: " << pBody->nMarkers<<"\n"<<
							// 	" & FrameofData.BodyData[b].nMarkers: " <<FrameofData.BodyData[b].nMarkers<<"\n" )

							for(int m=0; m<pBody->nMarkers; m++)
							{
								if(b==0 && m==4)
								{}
							else
							{
								approachObj->Markers[c] <<
									FrameofData.BodyData[b].Markers[m][0], // X
									FrameofData.BodyData[b].Markers[m][1], // Y
									FrameofData.BodyData[b].Markers[m][2]; // Z
									c+=1;
									// cout<<approachObj->Markers[c].transpose()<<"\n";
									// cout<<*getCurFrame->BodyData[b].Markers[m]<<endl;
								}
							}
						}
						else
							{ LOG_ERROR("approachObj->strMarkersBodyName[b] "<<approachObj->strMarkersBodyName[b]<<"\n"<<
								"pBody->szName "<<pBody->szName<<"\n"<<
								"pBody->nMarkers: " << pBody->nMarkers<<"\n"<<
								" & FrameofData.BodyData[b].nMarkers: " <<FrameofData.BodyData[b].nMarkers<<"\n" ) }
					}
				}

				if( approachObj->handoverRun() )
				{

					if(Flag_oneHand)
					{
						if( ((approachObj->i)%(approachObj->t_observe)==0) )
						{
							approachObj->useLtEf = false;

							/*subj right*/
							approachObj->lHandPredict = approachObj->predictionController(ltPosW, constRotL, approachObj->subjRtMarkers);

							approachObj->useLtEf = get<0>(approachObj->lHandPredict);
						}


						if( approachObj->useLtEf )
						{
							/*subj right*/
							taskOK = approachObj->goToHandoverPose(0.0, 0.7, approachObj->enableLHand, ltPosW, posTaskL, oriTaskL, approachObj->lHandPredict, approachObj->fingerPosR);

							taskOK = approachObj->forceController(approachObj->enableLHand, constPosL, initPosL, initRotL, leftForce, leftForceLo, leftTh, efLAce, posTaskL, oriTaskL, "l_gripper", approachObj->robotLtMarkers, approachObj->subjRtMarkers, approachObj->obj_rel_robotLtHand);

							gripperControl("l_gripper");
						}

						/*Head Pose*/
						headTask->target(approachObj->fingerPosR);
					}
					else
					{
						if( ((approachObj->i)%(approachObj->t_observe)==0) )
						{

							auto obj_rel_subj = [&]() -> std::vector<string>
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
								return subjMarkersName;
							};


							auto obj_rel_robot = [&]() -> bool
							{
								obj_rel_subj();

								// if(approachObj->obj_rel_robotLtHand < approachObj->obj_rel_robotRtHand)
								if(fingerPos(0) < 0.7)
								{
									if(approachObj->bool_t1)
									{
										approachObj->bool_t1 = false;
										approachObj->t1 = difftime( time(0), approachObj->start);
									}

									if( fingerPos(1) >= 0 )
									{
										if( (approachObj->stopRtEf) && (approachObj->useLtEf) )
										{
											LOG_INFO("robotLeftHand in use")

											approachObj->stopRtEf = false;
											posTaskR->stiffness(2.0);
											posTaskR->position(initPosR);
											oriTaskR->orientation(initRotR);

											approachObj->stopLtEf = true;
											posTaskL->stiffness(4.0);
										}

										approachObj->lHandPredict = approachObj->predictionController(ltPosW, constRotL, subjMarkersName);
									}
									else
									{
										if( (approachObj->stopLtEf) && (approachObj->useRtEf) )
										{
											LOG_WARNING("robotRightHand in use")

											approachObj->stopLtEf = false;
											posTaskL->stiffness(2.0);
											posTaskL->position(initPosL);
											oriTaskL->orientation(initRotL);

											approachObj->stopRtEf = true;
											posTaskR->stiffness(4.0);
										}

										approachObj->rHandPredict = approachObj->predictionController(rtPosW, constRotR, subjMarkersName);
									}
								}
								return false;
							};


							/* start only if object is within robot constraint space*/
							if( (approachObj->objectPos[0] > 1.1) && (approachObj->objectPos[0] < 2.0) &&
							    (approachObj->fingerPosL[0] > 1.1) && (approachObj->fingerPosL[0] < 2.0) &&
							    (approachObj->fingerPosR[0] > 1.1) && (approachObj->fingerPosR[0] < 2.0) )
							{ approachObj->pickaHand = true; }

							if( approachObj->pickaHand )
							{ obj_rel_robot(); }


						}// i%t_observe


						/*feed ef pose*/
						if( (!approachObj->stopRtEf) )
						{
							approachObj->useRtEf = false;
							robotMarkersName = approachObj->robotLtMarkers;

							taskOK = approachObj->goToHandoverPose(-0.1, 0.7, approachObj->enableLHand, ltPosW, posTaskL, oriTaskL, approachObj->lHandPredict, fingerPos);

							taskOK = approachObj->forceController(approachObj->enableLHand, constPosL, initPosL, initRotL, leftForce, leftForceLo, leftTh, efLAce, posTaskL, oriTaskL, "l_gripper", robotMarkersName, subjMarkersName, approachObj->obj_rel_robotLtHand);
							gripperControl("l_gripper");
						}
						else if( (!approachObj->stopLtEf) )
						{
							approachObj->useLtEf = false;
							robotMarkersName = approachObj->robotRtMarkers;

							taskOK = approachObj->goToHandoverPose(-0.7, 0.10, approachObj->enableRHand, rtPosW, posTaskR, oriTaskR, approachObj->rHandPredict, fingerPos);

							taskOK = approachObj->forceController(approachObj->enableRHand, constPosR, initPosR, initRotR, rightForce, rightForceLo, rightTh, efRAce, posTaskR, oriTaskR, "r_gripper", robotMarkersName, subjMarkersName, approachObj->obj_rel_robotRtHand);
							gripperControl("r_gripper");
						}

						/*Head Pose*/
						headTask->target(fingerPos);

						/*object Pose*/
						objEfTask->set_ef_pose(sva::PTransformd(approachObj->handRot, approachObj->objectPos));
					}

				}// handoverRun

			}//startCapture


			if(restartEverything)
			{
				dt+=1;

				/*reset moap markers*/
				for(unsigned int d=0; d<approachObj->totalMarkers; d++)
				{
					approachObj->Markers[d] << Eigen::Vector3d(0, 0, 0);
				}


				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();

				gripperL->setTargetQ({openGrippers});
				gripperR->setTargetQ({openGrippers});

				if(!approachObj->stopRtEf)
				{
					posTaskL->position(constPosL);
					oriTaskL->orientation(constRotL);
				}
				else if(!approachObj->stopLtEf)
				{
					posTaskR->position(constPosR);
					oriTaskR->orientation(constRotR);
				}


				approachObj->e = 1;

				approachObj->gClose = false;
				approachObj->gOpen = false;
				approachObj->openGripper = false;
				approachObj->closeGripper = false;

				approachObj->graspObject = true;
				approachObj->goBackInit = true;
				approachObj->takeBackObject = false;
				approachObj->restartHandover = false;

				approachObj->useLtEf = true;
				approachObj->stopLtEf = true;

				approachObj->useRtEf = true;
				approachObj->stopRtEf = true;

				approachObj->pickaHand = false;

				approachObj->objMass = 0.2;

				if( (dt%800) == 0 ) // 4 seconds )
				{
					/*reset head*/
					ctl.solver().removeTask(headTask);
					headTask.reset(new mc_tasks::LookAtTask("HEAD_LINK1", headVector, headTarget, ctl.robots(), ctl.robots().robotIndex(), 2., 500.));
					ctl.solver().addTask(headTask);
					headTask->selectActiveJoints(ctl.solver(), activeJointsName);

					posTaskL->position(initPosL);
					posTaskR->position(initPosR);
					oriTaskL->orientation(initRotL);
					oriTaskR->orientation(initRotR);

					approachObj->enableLHand = true;
					approachObj->enableRHand = true;

					gripperL->setTargetQ({closeGrippers});
					gripperR->setTargetQ({closeGrippers});

					restartEverything = false;

					bodyPosR = Eigen::Vector3d::Zero();
					bodyPosS = Eigen::Vector3d::Zero();

					double bodiesDiffX = 0.0;

					double leftForce_Xabs = 0.0;
					double leftForce_Yabs = 0.0;
					double leftForce_Zabs = 0.0;
					double rightForce_Xabs = 0.0;
					double rightForce_Yabs = 0.0;
					double rightForce_Zabs = 0.0;

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

					approachObj->local_Fzero = Eigen::Vector3d::Zero();
					approachObj->newTh = Eigen::Vector3d::Zero();
					approachObj->Finert = Eigen::Vector3d::Zero();
					approachObj->Fzero = Eigen::Vector3d::Zero();
					approachObj->Fclose = Eigen::Vector3d::Zero();
					approachObj->Fload = Eigen::Vector3d::Zero();
					approachObj->Fpull = Eigen::Vector3d::Zero();

					approachObj->count_reset++;

					cout<<"\033[1;33m***handover fresh start***\033[0m\n";
				}
			}
			return false;

		}// run


		void StartMocapStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.gui()->removeCategory({"Handover"});

			ctl.solver().removeTask(headTask);

			ctl.solver().removeTask(chestPosTask);
			ctl.solver().removeTask(chestOriTask);

			ctl.solver().removeTask(posTaskL);
			ctl.solver().removeTask(posTaskR);

			ctl.solver().removeTask(oriTaskL);
			ctl.solver().removeTask(oriTaskR);

			ctl.solver().removeTask(comTask);
		}

	} // namespace states
} // namespace mc_handover
