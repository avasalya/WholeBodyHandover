#include "handover_approachObject.h"

namespace mc_handover
{
	ApproachObject::ApproachObject() {}
	// { cout<<"\033[1;50mhandover object created\033[0m\n"; }



	ApproachObject::~ApproachObject() {}
	// { cout<<"\033[1;50mhandover object destroyed\033[0m\n"; }



	/*allocate memory*/
	void ApproachObject::initials()
	{
		/*markers Name strings*/
		strMarkersBodyName = {"4mars_robot_left_hand", "4mars_robot_right_hand", "3mars_obj", "7mars_subj_hands", "2mars_subj_head"};

		robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};//0-3 + dummy
		robotRtMarkers = {"wristRtEfA", "wristRtEfB", "gripperRtEfA", "gripperRtEfB"};//4-7

		objMarkers = {"center", "centerX", "centerY"}; //8-10 + dummyObj

		subjRtMarkers = {"lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD"};//11-14
		subjLtMarkers = {"lShapeLtA",              "lShapeLtC", "lShapeLtD"};//15-17
		subjMarkers = {"lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD", "lShapeLtA", "lShapeLtC", "lShapeLtD"}; //11-17

		subjHeadMarkers = {"head1", "head2"};//18-19 + dummyHead


		strMarkersName.insert(strMarkersName.begin(), robotLtMarkers.begin(), robotLtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), robotRtMarkers.begin(), robotRtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), objMarkers.begin(), objMarkers.end());
		strMarkersName.insert(strMarkersName.end(), subjMarkers.begin(), subjMarkers.end());
		strMarkersName.insert(strMarkersName.end(), subjHeadMarkers.begin(), subjHeadMarkers.end());

		totalMarkers = strMarkersName.size();

		for(unsigned int k=0; k<totalMarkers; k++)
			markers_name_index[strMarkersName[k]] = k;

		Markers.resize(totalMarkers);

		markersPos.resize(totalMarkers);
		for(int m=0; m<totalMarkers; m++)
			{ markersPos[m] = Eigen::MatrixXd::Zero(3,600000); }

		if(Flag_withoutRobot)
		{	LOG_ERROR("robot markers are not considered") }
		else
		{	LOG_WARNING("robot markers are considered")	}

		/*prediction controller parameter*/
		tuner << 100., 10., 10.;
		// tuner(2) = tuner(0)/tuner(1);

		t_predict = (int)tuner(0);
		t_observe = (int)tuner(1);
		it = (int)tuner(2);//t_predict/t_observe;

		/*timing of handover phases*/
		start = time(0);
	}



	bool ApproachObject::checkFrameOfData(std::vector<Eigen::Vector3d>)
	{
		bool checkNonZero{false};

		if(Flag_withoutRobot)
		{
			/*robot left markers*/
			Markers[0] << 0.208955, 0.350982, 0.552377;
			Markers[1] << 0.15638, 0.352496, 0.547814;
			Markers[2] << 0.217815, 0.334505, 0.432962;
			Markers[3] << 0.162401,  0.33451, 0.42898;

			/*robot right markers*/
			Markers[4] << 0.14048, -0.309184, 0.550067;
			Markers[5] << 0.198771, -0.308889,  0.555116;
			Markers[6] << 0.148997, -0.29622, 0.418868;
			Markers[7] << 0.202506, -0.293441, 0.425241;
		}


		for(unsigned int k=8; k<totalMarkers; k++)
		{
			// LOG_WARNING(k<<" "<<strMarkersName[k]<<" "<< Markers[ markers_name_index[ strMarkersName[k] ] ].transpose())
			if( Markers[k](0)>-10 && Markers[k](0)!=0 && Markers[k](0)<10 )
				{ checkNonZero = true; }
			else
			{ return false; }
		}
		return checkNonZero;
	}



	bool ApproachObject::handoverRun()
	{
		Eigen::Vector3d xl, yl, lshp_Xl, lshp_Yl, lshp_Zl;
		Eigen::Vector3d xr, yr, lshp_Xr, lshp_Yr, lshp_Zr;

		Eigen::Vector3d xo, yo, lshp_Xo, lshp_Yo, lshp_Zo;

		/*check for non zero frame only and store them*/
		if( checkFrameOfData(Markers) )
		{
			i+=1;
			for(int m=0; m<totalMarkers; m++)
				{ markersPos[m].col(i) << Markers[m]; }

			/*for GUI*/
			objectPosC = markersPos[8].col(i);//center
			objectPosCx = markersPos[9].col(i);//centerX
			objectPosCy = markersPos[10].col(i);//centerY

			fingerPosR = markersPos[11].col(i); //lShapeRtA
			fingerPosL = markersPos[15].col(i); //lShapeLtA

			gripperLtEfA = markersPos[2].col(i); //gripperLtEfA
			gripperLtEfB = markersPos[3].col(i); //gripperLtEfB

			gripperRtEfA = markersPos[6].col(i); //gripperRtEfA
			gripperRtEfB = markersPos[7].col(i); //gripperRtEfB

			gripperEfL = 0.5*( gripperLtEfA + gripperLtEfB );
			gripperEfR = 0.5*( gripperRtEfA + gripperRtEfB );

			/*right hand orientation*/
			yr = markersPos[14].col(i) - markersPos[13].col(i);//vCD=Yr
			xr = markersPos[13].col(i) - markersPos[11].col(i);//vAC=Xr

			lshp_Xr = xr/xr.norm();
			lshp_Yr = yr/yr.norm();
			lshp_Zr = lshp_Xr.cross(lshp_Yr);

			subjRHandRot.col(0) = lshp_Xr;
			subjRHandRot.col(1) = lshp_Yr;
			subjRHandRot.col(2) = lshp_Zr/lshp_Zr.norm();


			/*left hand orientation*/
			yl = markersPos[17].col(i) - markersPos[16].col(i);//vCD=Yl
			xl = markersPos[16].col(i) - markersPos[15].col(i);//vAC=Xl

			lshp_Xl = xl/xl.norm();
			lshp_Yl = yl/yl.norm();
			lshp_Zl = lshp_Xl.cross(lshp_Yl);

			subjLHandRot.col(0) = lshp_Xl;
			subjLHandRot.col(1) = lshp_Yl;
			subjLHandRot.col(2) = lshp_Zl/lshp_Zl.norm();


			/*object orientation*/
			yo = objectPosCy - objectPosC;//vCCy=yo
			xo = objectPosC - objectPosCx;//vCxC=xo

			lshp_Xo = xo/xo.norm();
			lshp_Yo = yo/yo.norm();
			lshp_Zo = lshp_Xo.cross(lshp_Yo);

			objRot.col(0) = lshp_Xo;
			objRot.col(1) = lshp_Yo;
			objRot.col(2) = lshp_Zo/lshp_Zo.norm();

			finR_rel_efL = (gripperEfL - fingerPosR).norm();
			finL_rel_efR = (gripperEfR - fingerPosL).norm();


			headPos1 = markersPos[18].col(i);//head left
			headPos2 = markersPos[19].col(i);//head right
			headPos = 0.5*(headPos1 + headPos2);


			/*virtual markers and their relative position*/
			virObjLeft = sva::PTransformd(Eigen::Vector3d(0,-0.3, 0)) * sva::PTransformd(objRot.transpose(), objectPosC);
			virObjRight = sva::PTransformd(Eigen::Vector3d(0,0.3, 0)) * sva::PTransformd(objRot.transpose(), objectPosC);

			virObj_rel_robotRtHand = ( gripperEfR - virObjLeft.translation() ).norm();//gripperRtEfA - virObjLeft
			virObj_rel_robotLtHand = ( gripperEfL - virObjRight.translation() ).norm();//gripperLtEfA - virObjRight

			virObj_rel_subjLtHand = ( fingerPosL - virObjLeft.translation() ).norm();//lshpLtA - virObjLeft
			virObj_rel_subjRtHand = ( fingerPosR - virObjRight.translation() ).norm();//lshpRtA - virObjRight


			/*move EF when subject approaches object 1st time*/
			obj_rel_robotRtHand = ( gripperEfR - virObjLeft.translation() ).norm();//gripperRtEfA - objLeft
			obj_rel_robotLtHand = ( gripperEfL - virObjRight.translation() ).norm();//gripperLtEfA - objRight

			obj_rel_subjLtHand = ( fingerPosL - virObjLeft.translation() ).norm();//lshpLtA - objLeft
			obj_rel_subjRtHand = ( fingerPosR - virObjRight.translation() ).norm();//lshpRtA - objRight


			// if(i%400 == 0)
			// {
			// 	LOG_ERROR("virObj_rel_robotLtHand "<< virObj_rel_robotLtHand)
			// 	LOG_WARNING("virObj_rel_robotRtHand "<< virObj_rel_robotRtHand)
			// }

			return true;
		}
		else
		{ return false; }
	}



	std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> ApproachObject::predictionController(
		const Eigen::Vector3d& curPosEf,
		const Eigen::Matrix3d & constRotLink6,
		std::vector<std::string> subjMarkersName
		)
	{
		bool ready{false};

		Eigen::Vector3d x, y, lshp_X, lshp_Y, lshp_Z;
		Eigen::Vector3d initRefPos, curPosLshp, initPosSubj, ithPosSubj, avgVelSubj, predictPos;

		Eigen::MatrixXd curVelSubj, P_M_Subj, wp;

		Eigen::Matrix3d subjHandRot, handoverRot;

		sva::PTransformd X_R_ef;
		sva::PTransformd X_R_M;

		sva::PTransformd X_M_Subj;
		sva::PTransformd X_ef_Subj;

		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_ef_Subj;

		/*prediction_ tuner*/
		t_predict = (int)tuner(0);
		t_observe = (int)tuner(1);
		it = (int)tuner(2); //t_predict/t_observe;

		/*Subject Hand L SHAPE*/
		auto sizeStr = subjMarkersName.size();
		curPosLshp = markersPos[markers_name_index[subjMarkersName[sizeStr-2]]].col(i);//C
		y = markersPos[markers_name_index[subjMarkersName[sizeStr-1]]].col(i) - curPosLshp;//vCD=Y
		x = curPosLshp - markersPos[markers_name_index[subjMarkersName[0]]].col(i);//vAC=X

		lshp_X = x/x.norm();
		lshp_Y = y/y.norm();
		lshp_Z = lshp_X.cross(lshp_Y);

		subjHandRot.col(0) = lshp_X;
		subjHandRot.col(1) = lshp_Y;
		subjHandRot.col(2) = lshp_Z/lshp_Z.norm();
		// LOG_ERROR(subjHandRot<<"\n\n")


		P_M_Subj = Eigen::MatrixXd::Zero(3, t_observe);
		for(int j=1;j<=t_observe; j++)
		{
			P_M_Subj.col(j-1) = markersPos[markers_name_index[subjMarkersName[0]]].col((i-t_observe)+j);
			// cout << "P_M_Subj.col(j-1) " << P_M_Subj.col(j-1).transpose() <<endl;

			if(j==1)
				{ initPosSubj = P_M_Subj.col(j-1); }

			if(j==t_observe)
				{ ithPosSubj = P_M_Subj.col(t_observe-1); }
		}

		/*get average velocity of previous *t_observe* sec Subj movement*/
		avgVelSubj = (ithPosSubj - initPosSubj)/0.1;
		// cout<< " vel " << avgVelSubj.transpose() <<endl;


		/*predict position in straight line after t_predict time*/
		predictPos = handoverTraj->constVelocityPredictPos(avgVelSubj, ithPosSubj, t_predict);

		if(subjHasObject)
		{ X_M_Subj = sva::PTransformd(objRot, predictPos); }
		else if(robotHasObject)
		{ X_M_Subj = sva::PTransformd(subjHandRot, predictPos); }


		X_R_ef = sva::PTransformd(constRotLink6, curPosEf);
		X_R_M = sva::PTransformd(idtMat, Eigen::Vector3d(0., 0., 0.));

		X_ef_Subj = X_M_Subj * X_R_M * X_R_ef.inv();

		handoverRot = X_ef_Subj.rotation().transpose();

		/*way points for robot ef to predict pos*/
		wp_ef_Subj=handoverTraj->constVelocity(curPosEf, predictPos, t_predict);
		// wp_ef_Subj=handoverTraj->constVelocity(curPosEf, X_ef_Subj.translation(), t_predict);
		wp = get<0>(wp_ef_Subj);
		initRefPos << wp(0,it), wp(1,it), wp(2,it);

		ready = true;

		return std::make_tuple(ready, wp, initRefPos, handoverRot);
	}


	void ApproachObject::goToHandoverPose(
		double min,
		double max,
		bool& enableHand,
		Eigen::Vector3d& curPosEf,
		std::shared_ptr<mc_tasks::PositionTask>& posTask,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
		std::tuple<bool,
		Eigen::MatrixXd,
		Eigen::Vector3d,
		Eigen::Matrix3d> handPredict,
		Eigen::Vector3d offsetPos)
	{
		Eigen::Vector3d wp, handoverPos;

		if(Flag_prediction)
		{
			it+= (int)tuner(2);
			if( it < get<1>(handPredict).cols() )
			{
				wp << get<1>(handPredict)(0,it), get<1>(handPredict)(1,it), get<1>(handPredict)(2,it);
				handoverPos = curPosEf + (wp - get<2>(handPredict));
			}
		}
		else
		{
			handoverPos = offsetPos;
		}

		/*robot constraint*/
		if(enableHand &&
			(handoverPos(0)>= 0.10) && (handoverPos(0)<= 0.8)
			&& (handoverPos(1)>= min)  && (handoverPos(1)<= max)
			&& (handoverPos(2)>= 0.80) /*&& (handoverPos(2)<= 1.4)*/
			)
		{
			sva::PTransformd new_pose(get<3>(handPredict), handoverPos);
			posTask->position(new_pose.translation());
			oriTask->orientation(new_pose.rotation());

			// LOG_INFO("it "<<it << "\npredictPos wp "<<handoverPos.transpose()<<"\n" << "offsetPos "<< offsetPos.transpose())
		}
	}


	bool ApproachObject::forceController(
		bool& enableHand,

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
		std::shared_ptr<mc_tasks::OrientationTask>& oriTaskR
	)
	{
		Eigen::Vector3d leftTh, rightTh;

		leftTh = thresh.segment(3,3);
		rightTh = thresh.segment(9,3);


		if( (finR_rel_efL < 0.35) || (finL_rel_efR < 0.35) )
		{
			auto checkForce = [&](const char *axis_name, int idx)
			{
				if( (finR_rel_efL < 0.15) || (finL_rel_efR < 0.15) ) // not effective n efficient
				{
					if(enableHand)
					{
						enableHand = false;
						t7 = difftime( time(0), start);
						LOG_WARNING("trying to pull object, motion stopped")
					}

					FinertL = (objMass/2) * efLAce;
					FinertR = (objMass/2) * efRAce;

					FpullL[0] = abs(leftForce[0]) - abs(FinertL[0]) - abs(FzeroL[0]);
					FpullL[1] = abs(leftForce[1]) - abs(FinertL[1]) - abs(FzeroL[1]);
					FpullL[2] = abs(leftForce[2]) - abs(FinertL[2]) - abs(FzeroL[2]);

					FpullR[0] = abs(rightForce[0]) - abs(FinertR[0]) - abs(FzeroR[0]);
					FpullR[1] = abs(rightForce[1]) - abs(FinertR[1]) - abs(FzeroR[1]);
					FpullR[2] = abs(rightForce[2]) - abs(FinertR[2]) - abs(FzeroR[2]);

					/*to avoid slip-- try to check if (Fpull > Th) for cont. over 1 sec or more*/

					if( (abs(FpullL[idx]) > newThL[idx]) || (abs(FpullR[idx]) > newThR[idx]) )
					{
						if(objHasContacts)
						{
							removeContacts = true;
							robotHasObject = false;

							gOpen = true;
							restartHandover = true;
							takeBackObject = false;

							if(goBackInit)
							{
								t8 = difftime( time(0), start);
								count_rh_success++;

								goBackInit = false;
								LOG_SUCCESS("object pulled and has mass(kg) = " << objMass)
							}
						}
						else
						{
							count_rh_fail++;
							LOG_ERROR("robot doesn't have contacts with the object")
						}
					}
				}
				return false;
			};

			/*check if object is being pulled*/
			if(takeBackObject)
			{
				return checkForce("x-axis", 0) || checkForce("y-axis", 1) || checkForce("z-axis", 2);
			}
			else
			{
				/*open empty gripper when subject come near to robot*/
				if( (!openGripper) )
				{
					gOpen = true;
					t2 = difftime( time(0), start);
					LOG_INFO("1st cycle, opening grippers")
				}

				/*stop motion*/
				else if( (openGripper)
						&& (!closeGripper)
						&& (!restartHandover)
						&& (enableHand)
						&& ( (virObj_rel_robotRtHand < 0.20) || (virObj_rel_robotLtHand < 0.20) )
						)
				{
					FzeroL = leftForce;
					FzeroR = rightForce;

					local_FzeroL = leftForceLo;
					local_FzeroR = rightForceLo;

					enableHand = false;
					t3 = difftime( time(0), start);
					LOG_WARNING("motion stopped with Fzero L & R Norms "<<FzeroL.norm()<<" & "<< FzeroR.norm())
				}

				/*closed WITH object*/
				else if( (!enableHand) &&
						(graspObject) && /*along localY direction*/
						( abs( (leftForceLo - local_FzeroL)(2) ) >4.0 ) &&
						( abs( (rightForceLo - local_FzeroR)(2) ) > 4.0 ) )
				{
					gClose = true;
					closeGripper = true;
					graspObject = false;

					FcloseL = leftForce;
					FcloseR = rightForce;
					t4 = difftime( time(0), start);
					LOG_WARNING("closing with Fclose L & R Norms "<<FcloseL.norm()<<" & "<<FcloseR.norm())
				}

				/*closed WITHOUT object*/
				else if(
						(!restartHandover) && (!graspObject)  &&
						(obj_rel_subjRtHand < obj_rel_robotLtHand) &&
						(obj_rel_subjLtHand < obj_rel_robotRtHand) &&
						(finR_rel_efL > 0.3) &&
						(finL_rel_efR > 0.3) )

				{
					if( (FcloseL.norm() < 2.0) || (FcloseR.norm() < 2.0) )
					{
						gClose = false;
						closeGripper = false;
						graspObject = true;

						gOpen = true;

						count_hr_fail++;

						t_falseClose = difftime( time(0), start);

						LOG_ERROR("false close, Fclose L & R Norms, try with object again"<<FcloseL.norm()<<" & "<<FcloseR.norm())
					}
					else
					{
						FcloseL = Eigen::Vector3d(1,1,1);
						FcloseR = Eigen::Vector3d(1,1,1);
					}
				}
			}
		}



		/*move EF to initial position*/
		if( (finR_rel_efL > 0.45) && (finL_rel_efR > 0.45) )
		{
			if(!goBackInit)
			{
				posTaskL->position(relaxPosL);
				oriTaskL->orientation(initRotL);

				posTaskR->position(relaxPosR);
				oriTaskR->orientation(initRotR);

				if(!gClose)
				{
					t9 = difftime( time(0), start);
					gClose = true;
				}
			}
		}



		/*restart handover*/
		if( (finR_rel_efL > 0.8) && (finL_rel_efR > 0.8) )
		{
			/*come only once after object is grasped*/
			if( (closeGripper) && (!restartHandover) && (!enableHand) )
			{
				/*add contacts*/
				if(e == 2)
				{
					addContacts = true;

					t5 = difftime( time(0), start);

					count_hr_success++;
				}


				if( (e%200==0) )//wait xx sec
				{
					FloadL <<
					accumulate( FloadLx.begin(), FloadLx.end(), 0.0)/double(FloadLx.size()),
					accumulate( FloadLy.begin(), FloadLy.end(), 0.0)/double(FloadLy.size()),
					accumulate( FloadLz.begin(), FloadLz.end(), 0.0)/double(FloadLz.size());

					FloadR <<
					accumulate( FloadRx.begin(), FloadRx.end(), 0.0)/double(FloadRx.size()),
					accumulate( FloadRy.begin(), FloadRy.end(), 0.0)/double(FloadRy.size()),
					accumulate( FloadRz.begin(), FloadRz.end(), 0.0)/double(FloadRz.size());


					/*try "worldWrench" with gravity*/
					// objMass = ( FloadL.norm() + FloadR.norm() )/9.81;
					objMass = ( (FloadL + FloadR).norm() )/9.81;

					/*new threshold*/
					newThL = FloadL + leftTh;
					newThR = FloadR + rightTh;

					if(objHasContacts)
					{
						/*move EF in-solver to relax pose*/
						posTaskL->position(relaxPosL);
						oriTaskL->orientation(relaxRotL);

						LOG_SUCCESS("Robot has object, Ef(s) returning to relax pose, FloadL & FloadR are "<< FloadL.transpose() <<" :: "<< FloadR.transpose())
					}

					if( subjHasObject &&
						(obj_rel_subjRtHand > obj_rel_robotLtHand) && /* less conservative with "||" */
						(obj_rel_subjLtHand > obj_rel_robotRtHand)
						)
					{
						subjHasObject = false;
						robotHasObject = true;
					}

					if(robotHasObject)
					{
						enableHand = true;
						takeBackObject = true;
						LOG_SUCCESS("begin 2nd cycle, motion enabled")
					}

					/*clear vector memories*/
					FloadLx.clear(); FloadLy.clear(); FloadLz.clear();
					FloadRx.clear(); FloadRy.clear(); FloadRz.clear();
				}
				else /*divide by 9.81 and you will get object mass*/
				{
					FloadLx.push_back( abs( abs(leftForce[0])-abs(FzeroL[0]) ) );
					FloadLy.push_back( abs( abs(leftForce[1])-abs(FzeroL[1]) ) );
					FloadLz.push_back( abs( abs(leftForce[2])-abs(FzeroL[2]) ) );

					FloadRx.push_back( abs( abs(rightForce[0])-abs(FzeroR[0]) ) );
					FloadRy.push_back( abs( abs(rightForce[1])-abs(FzeroR[1]) ) );
					FloadRz.push_back( abs( abs(rightForce[2])-abs(FzeroR[2]) ) );
				}
				e+=1;
			}


			if(restartHandover)
			{
				openGripper = false;
				closeGripper = false;

				graspObject = true;
				goBackInit = true;
				enableHand = true;
				e = 1;

				if( restartHandover && (posTaskL->eval().norm()) <0.05 && (posTaskR->eval().norm() <0.05) )
				{
					posTaskL->position(initPosL);
					posTaskR->position(initPosR);
					restartHandover = false;

					useLeftEf = false;
					useRightEf = false;

					startNow = false;

					subjHasObject = true;

					addContacts = false;
					removeContacts = false;

					pickNearestHand = true;

					t1 = 0.0;
					t2 = 0.0;
					t3 = 0.0;
					t4 = 0.0;
					t5 = 0.0;
					t6 = 0.0;
					t7 = 0.0;
					t8 = 0.0;
					t9 = 0.0;
					t_falseClose = 0.0;

					bool_t1 = true;
					bool_t6 = true;

					LOG_SUCCESS("object returned to subject, motion enabled, restarting handover\n")
				}
			}
		}

		return false;
	}

}//namespace mc_handover
