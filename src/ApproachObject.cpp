#include "ApproachObject.h"

namespace lipm_walking
{
	ApproachObject::ApproachObject(Controller & controller_):ctl(controller_) {}


	ApproachObject::~ApproachObject() {}


	void ApproachObject::initials()
	{

		/*markers Name strings*/
		strMarkersBodyName = {
			"4mars_robot_left_hand",
			"4mars_robot_right_hand",
			"3mars_obj",
			"7mars_subj_hands",
			"2mars_subj_head"};

		robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};//0-3 + dummy
		robotRtMarkers = {"wristRtEfA", "wristRtEfB", "gripperRtEfA", "gripperRtEfB"};//4-7

		objMarkers = {"center", "centerX", "centerY"}; //8-10 + dummyleft

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

		// LOG_INFO(" start here ")
		for(unsigned int k=8; k<totalMarkers; k++)
		{
			/*mostly human markers*/
			if( Markers[k](0) > 0.0 && Markers[k](0) < 2 )
			{
				checkNonZero = true;
				// LOG_WARNING("return true " << checkNonZero)
				// LOG_ERROR(" "<<k<<" "<<strMarkersName[k]<<" "<< Markers[ markers_name_index[ strMarkersName[k] ] ].transpose())
			}
			else
			{
				checkNonZero = false;
				return false;
			}
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

			/*marker ID*/
			gripperLtEfA = markersPos[2].col(i); //gripperLtEfA
			gripperLtEfB = markersPos[3].col(i); //gripperLtEfB

			gripperRtEfA = markersPos[6].col(i); //gripperRtEfA
			gripperRtEfB = markersPos[7].col(i); //gripperRtEfB

			gripperEfL = 0.5*( gripperLtEfA + gripperLtEfB );
			gripperEfR = 0.5*( gripperRtEfA + gripperRtEfB );

			objectPosC = markersPos[8].col(i);//center
			objectPosCx = markersPos[9].col(i);//centerX
			objectPosCy = markersPos[10].col(i);//centerY

			fingerPosR = markersPos[11].col(i); //lShapeRtA

			/*right hand orientation*/
			yr = markersPos[14].col(i) - markersPos[13].col(i);//vCD=Yr
			xr = markersPos[13].col(i) - markersPos[11].col(i);//vAC=Xr

			lshp_Xr = xr/xr.norm();
			lshp_Yr = yr/yr.norm();
			lshp_Zr = lshp_Xr.cross(lshp_Yr);

			subjRHandRot.col(0) = lshp_Xr;
			subjRHandRot.col(1) = lshp_Yr;
			subjRHandRot.col(2) = lshp_Zr/lshp_Zr.norm();


			fingerPosL = markersPos[15].col(i); //lShapeLtA

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


			/*body to body interpersonal distance*/
			headPos1 = markersPos[18].col(i);//head left
			headPos2 = markersPos[19].col(i);//head right
			headPos = 0.5*(headPos1 + headPos2);


			/*subj finger relative to  oposite ef*/
			finR_rel_efL = (gripperEfL - fingerPosR).norm();
			finL_rel_efR = (gripperEfR - fingerPosL).norm();


			/*virtual markers and their relative position*/
			virObjLeft = sva::PTransformd(Eigen::Vector3d(0,-0.3, 0)) * sva::PTransformd(objRot.transpose(), objectPosC);
			virObjRight = sva::PTransformd(Eigen::Vector3d(0,0.3, 0)) * sva::PTransformd(objRot.transpose(), objectPosC);

			virObj_rel_robotRtHand = ( gripperEfR - virObjLeft.translation() ).norm();//gripperRtEfA - virObjLeft
			virObj_rel_robotLtHand = ( gripperEfL - virObjRight.translation() ).norm();//gripperLtEfA - virObjRight

			virObj_rel_subjLtHand = ( fingerPosL - virObjLeft.translation() ).norm();//lshpLtA - virObjLeft
			virObj_rel_subjRtHand = ( fingerPosR - virObjRight.translation() ).norm();//lshpRtA - virObjRight


			if(FlAG_INDIVIDUAL)
			{
				/*move EF when subject approaches object 1st time*/
				obj_rel_subjRtHand = ( fingerPosR - objectPosC ).norm();//lshpRtA - obj
				obj_rel_subjLtHand = ( fingerPosL - objectPosC ).norm();//lshpLtA - obj

				obj_rel_robotLtHand = ( gripperEfL - objectPosC ).norm();//gripperEfA - obj
				obj_rel_robotRtHand = ( gripperEfR - objectPosC ).norm();//gripperEfA - obj
			}
			else
			{
				obj_rel_subjRtHand = ( fingerPosR - virObjRight.translation() ).norm();//lshpRtA - objRight
				obj_rel_subjLtHand = ( fingerPosL - virObjLeft.translation() ).norm();//lshpLtA - objLeft

				obj_rel_robotLtHand = ( gripperEfL - virObjRight.translation() ).norm();//gripperLtEfA - objRight
				obj_rel_robotRtHand = ( gripperEfR - virObjLeft.translation() ).norm();//gripperRtEfA - objLeft
			}

			return true;
		}
		else
			{ return false; }

	}


	std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> ApproachObject::predictionController(
		const Eigen::Vector3d& curPosEf,
		const Eigen::Matrix3d & constRotLink6,
		std::vector<std::string> lShpMarkersName)
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
		auto sizeStr = lShpMarkersName.size();
		curPosLshp = markersPos[markers_name_index[lShpMarkersName[sizeStr-2]]].col(i);//C
		y = markersPos[markers_name_index[lShpMarkersName[sizeStr-1]]].col(i) - curPosLshp;//vCD=Y
		x = curPosLshp - markersPos[markers_name_index[lShpMarkersName[0]]].col(i);//vAC=X

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
			P_M_Subj.col(j-1) = markersPos[markers_name_index[lShpMarkersName[0]]].col((i-t_observe)+j);
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


		if(FlAG_INDIVIDUAL)
		{
			X_M_Subj = sva::PTransformd(subjHandRot, predictPos);
		}
		else
		{
			if(subjHasObject)
			{
				X_M_Subj = sva::PTransformd(objRot, predictPos);
			}
			else if(robotHasObject)
			{
				X_M_Subj = sva::PTransformd(subjHandRot, predictPos);
			}
		}

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

		GlobalAvgVelSubjNorm = avgVelSubj.norm();

		handRot = subjHandRot.transpose();

		return std::make_tuple(ready, wp, initRefPos, handoverRot, predictPos);

	}


	void ApproachObject::goToHandoverPose(
		double Xmax,
		double Ymin,
		double Ymax,
		bool& enableHand,
		Eigen::Vector3d& curPosEf,
		std::shared_ptr<mc_tasks::PositionTask>& posTask,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> handPredict,
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
			// if(i%300 == 0)
			// { LOG_ERROR(Xmax << "   handoverPos   " << handoverPos(0)) }
		}

		/*robot constraint*/
		if(enableHand &&
			(handoverPos(0)>= 0.10) && (handoverPos(0)<= Xmax)
			&& (handoverPos(1)>= Ymin)  && (handoverPos(1)<= Ymax)
			&& (handoverPos(2)>= 0.80) /*&& (handoverPos(2)<= 1.4)*/
			)
		{

			sva::PTransformd new_pose(get<3>(handPredict), handoverPos);
			posTask->position(new_pose.translation());
			oriTask->orientation(new_pose.rotation());

			// LOG_INFO("it "<<it << "\npredictPos wp "<<handoverPos.transpose()<<"\n" << "offsetPos "<< offsetPos.transpose())

			if(FlAG_INDIVIDUAL)
			{
				if( (takeBackObject) && (bool_t6) )
				{
					bool_t6 = false;
					t6 = difftime( time(0), start);
				}
			}
		}

	}



	bool ApproachObject::forceControllerTogether(
		bool& enableHand,
		sva::PTransformd X_0_rel,
		Eigen::Vector3d initPosR, Eigen::Matrix3d initRotR,
		Eigen::Vector3d initPosL, Eigen::Matrix3d initRotL,
		Eigen::Vector3d relaxPosR, Eigen::Matrix3d relaxRotR,
		Eigen::Vector3d relaxPosL, Eigen::Matrix3d relaxRotL,
		Eigen::VectorXd thresh,
		Eigen::Vector3d leftForce, Eigen::Vector3d rightForce,
		Eigen::Vector3d leftForceSurf, Eigen::Vector3d rightForceSurf,
		Eigen::Vector3d efLAce, Eigen::Vector3d efRAce,
		std::shared_ptr<mc_tasks::PositionTask>& posTaskL,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTaskL,
		std::shared_ptr<mc_tasks::PositionTask>& posTaskR,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTaskR)
	{

		Eigen::Vector3d leftTh, rightTh;

		leftTh = thresh.segment(3,3);
		rightTh = thresh.segment(9,3);


		/*check pull forces/release object*/
		if( (finR_rel_efL < 0.35) || (finL_rel_efR < 0.35) )
		{

			/*
			*  7th
			*/
			auto checkForce = [&](const char *axis_name, int idx)
			{

				if( (finR_rel_efL < 0.15) || (finL_rel_efR < 0.15) ) // not effective n efficient
				/*try to check if (Fpull > Th) for cont. over 1 sec or more*/
				{
					if(enableHand)
					{
						enableHand = false;

						t7 = difftime( time(0), start);

						LOG_WARNING("------------------------------> trying to pull object, motion stopped")
					}

					FinertL = (objMassZ/2) * efLAce;
					FinertR = (objMassZ/2) * efRAce;

					FpullL[0] = abs(leftForce[0]) - abs(FinertL[0]) - abs(FzeroL[0]);
					FpullL[1] = abs(leftForce[1]) - abs(FinertL[1]) - abs(FzeroL[1]);
					FpullL[2] = abs(leftForce[2]) - abs(FinertL[2]) - abs(FzeroL[2]);

					FpullR[0] = abs(rightForce[0]) - abs(FinertR[0]) - abs(FzeroR[0]);
					FpullR[1] = abs(rightForce[1]) - abs(FinertR[1]) - abs(FzeroR[1]);
					FpullR[2] = abs(rightForce[2]) - abs(FinertR[2]) - abs(FzeroR[2]);


					if( (abs(FpullL[idx]) > newThL[idx]) || (abs(FpullR[idx]) > newThR[idx]) )
					{

						efLPosOfHandover = posTaskL->position();
						hLPosOfHandover = fingerPosL;

						efRPosOfHandover = posTaskR->position();
						hRPosOfHandover = fingerPosR;

						if(objHasContacts)
						{

							gOpen = true;
							restartHandover = true;
							takeBackObject = false;
							robotHasObject = false;

							removeContacts = true;

							if(goBackInitPose)
							{
								t8 = difftime( time(0), start);
								count_rh_success++;

								goBackInitPose = false;

								cout<< "------------------------------> object pulled with forces L("<<axis_name<<") = " << abs(FpullL[idx]) << "  and R("<<axis_name<<") = " << abs(FpullR[idx])<<endl;
								LOG_SUCCESS("------------------------------> object returned and estimated mass(Norm) was = " << objMassNorm)
							}

						}
						else if(rh_fail) /*check for false r-to-h trial during object pulling*/
						{
							count_rh_fail++;
							count_rh_success--;
							rh_fail = false;

							LOG_ERROR("------------------------------> robot doesn't have contacts with the object")
						}

					}
				}
				return false;

			};

			/*
			*  1st, 6th
			*/
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

					LOG_INFO("------------------------------> 1st cycle, opening grippers")
				}

				/*stop motion*/
				else if( (enableHand)
					&& (openGripper)
					&& (!closeGripper)
					&& (!restartHandover)
					&& ( (virObj_rel_robotRtHand < MIN_ALLOWED_DIST) || (virObj_rel_robotLtHand < MIN_ALLOWED_DIST) )
					)
				{
					FzeroL = leftForce;
					FzeroR = rightForce;

					localSurf_FzeroL = leftForceSurf;
					localSurf_FzeroR = rightForceSurf;

					enableHand = false;
					t3 = difftime( time(0), start);

					LOG_WARNING("------------------------------> motion stopped with Fzero L & R Norms "<<FzeroL.norm()<<" & "<< FzeroR.norm())
				}

				/*closed WITH object*/
				else if( (!enableHand) &&
						(graspObject) && /*along localY direction*/
					( abs( (leftForceSurf - localSurf_FzeroL)(2) ) > MIN_SURFACE_FORCE ) &&
					( abs( (rightForceSurf - localSurf_FzeroR)(2) ) > MIN_SURFACE_FORCE ) )
				{
					gClose = true;
					closeGripper = true;
					graspObject = false;

					FcloseL = leftForce;
					FcloseR = rightForce;

					t4 = difftime( time(0), start);

					efLPosOfHandover = posTaskL->position();
					hLPosOfHandover = fingerPosL;

					efRPosOfHandover = posTaskR->position();
					hRPosOfHandover = fingerPosR;

					LOG_WARNING("------------------------------> closing with Fclose L & R Norms "<<FcloseL.norm()<<" & "<<FcloseR.norm())
				}

				/*closed WITHOUT object*/
				else if( (!restartHandover) &&
					(!graspObject)  &&
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

						LOG_ERROR("------------------------------> false close, Fclose L & R Norms, try with object again "<<FcloseL.norm()<<" & "<<FcloseR.norm())
					}
					else
					{
						FcloseL = Eigen::Vector3d(1, 1, 1);
						FcloseR = Eigen::Vector3d(1, 1, 1);
					}
				}

			}

		}

		/*
		* 5th
		*/
		/*trigger again walk fwd when robot has the object*/
		if( walkFwdAgain && robotHasObject && takeBackObject && ( (finR_rel_efL < MAX_ALLOWED_DIST) || (finL_rel_efR < MAX_ALLOWED_DIST) ) )
		{

			if( (fingerPosL(2)>=objAboveWaist) || (fingerPosR(2)>=objAboveWaist) )
			{
				walkFwd = true;
				walkFwdAgain = false;
				LOG_WARNING("------------------------------> walk Fwd again, are you here in the beginning of 2nd cycle?")
			}

		}

		/*
		*  8th
		*/
		/*just before the end of 2nd cycle, move EF to initial position at end of handover routine*/
		if( restartHandover && (finR_rel_efL > 0.45) && (finL_rel_efR > 0.45) )
		{

			if( (!gClose) && (!goBackInitPose) )
			{
				posTaskL->position(initPosL);
				oriTaskL->orientation(initRotL);

				posTaskR->position(initPosR);
				oriTaskR->orientation(initRotR);

				gClose = true;
				goBackInitPose = true;

				t9 = difftime( time(0), start);

				LOG_WARNING("------------------------------> robot will move to half-sit pose")
			}
		}


		/*in between 1st and 2nd cycles*/
		if( (finR_rel_efL > 0.8) && (finL_rel_efR > 0.8) )
		{
			/*
			*  2nd
			*/
			/*come only once after object is grasped*/
			if( (closeGripper) && (!restartHandover) && subjHasObject )
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

					/*load force in the direction of gravity in world*/
					objMassZ = ( FloadL(2) + FloadR(2) )/GRAVITY;
					objMassNorm = ( FloadL.norm() + FloadR.norm() )/GRAVITY;

					/*new threshold*/
					newThL = FloadL + leftTh;
					newThR = FloadR + rightTh;

					if(objHasContacts)
					{
						/*move EF in-solver to relax pose*/
						posTaskL->position(Eigen::Vector3d(X_0_rel.translation()(0)+0.25, relaxPosL(1), relaxPosL(2)));
						oriTaskL->orientation(relaxRotL);

						LOG_SUCCESS("------------------------------> Robot has object of mass(Z) = "<< objMassZ)
						cout<< "------------------------------> FloadL & FloadR are "<< FloadL.transpose() <<" :: "<< FloadR.transpose()<<endl;
					}

					if( subjHasObject &&
						(obj_rel_subjRtHand > obj_rel_robotLtHand) && /* less aggressive with "||" */
						(obj_rel_subjLtHand > obj_rel_robotRtHand)
						)
					{
						subjHasObject = false;
						robotHasObject = true;

						LOG_SUCCESS("------------------------------> Ef(s) returning to relax pose")
					}

					/*clear vector memories*/
					FloadLx.clear(); FloadLy.clear(); FloadLz.clear();
					FloadRx.clear(); FloadRy.clear(); FloadRz.clear();
				}
				else /*averaging load force to get object mass*/
				{
					FloadLx.push_back( abs(leftForce[0]) );
					FloadLy.push_back( abs(leftForce[1]) );
					FloadLz.push_back( abs(leftForce[2]) );

					FloadRx.push_back( abs(rightForce[0]) );
					FloadRy.push_back( abs(rightForce[1]) );
					FloadRz.push_back( abs(rightForce[2]) );

					// FloadLx.push_back( abs( abs(leftForce[0])-abs(FzeroL[0]) ) );
					// FloadLy.push_back( abs( abs(leftForce[1])-abs(FzeroL[1]) ) );
					// FloadLz.push_back( abs( abs(leftForce[2])-abs(FzeroL[2]) ) );

					// FloadRx.push_back( abs( abs(rightForce[0])-abs(FzeroR[0]) ) );
					// FloadRy.push_back( abs( abs(rightForce[1])-abs(FzeroR[1]) ) );
					// FloadRz.push_back( abs( abs(rightForce[2])-abs(FzeroR[2]) ) );
				}
				e+=1;

			}

			/*
			*  3rd
			*/
			/*walk back when object arrives at relax pose*/
			if( (!takeBackObject) && (!walkBack) && abs( abs(X_0_rel.translation()(0)) - abs(objectPosC(0)) )<0.25 )
			{

				if(Flag_Walk)
				{
					ctl.postureTask->reset();
					ctl.solver().removeTask(posTaskL);
					ctl.solver().removeTask(oriTaskL);
				}

				if(robotHasObject)
				{
					takeBackObject = true;
					walkBack = true;

					LOG_SUCCESS("------------------------------> robot returning to relax pose")
				}

			}

			/*
			*  4th
			*/
			/*add ef task again*/
			if( walkBack && robotHasObject && (!enableHand) )
			{

				if(Flag_Walk && ctl.isLastDSP() )
				{

					/*true when last DSP is finished*/
					finishedWalk_ = ctl.config()("finishedWalk", false);

					if(finishedWalk_)
					{
						ctl.config().add("finishedWalk", false);

						ctl.solver().addTask(posTaskL);
						posTaskL->reset();

						ctl.solver().addTask(oriTaskL);
						oriTaskL->reset();

						enableHand = true;
						walkFwdAgain = true;
					}
					else
					{
						ctl.postureTask->reset();
					}
				}
				else
				{
					enableHand = true;
				}
				LOG_INFO("------------------------------> ready to begin 2nd cycle, motion enabled")
			}

			/*
			*  9th
			*/
			/*at the end of 2nd cycle*/
			if(restartHandover)
			{

				if(!pickNearestHand)
				{

					e = 1;
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
					rh_fail = true;

					openGripper = false;
					closeGripper = false;

					graspObject = true;
					enableHand = true;

					startNow = false;

					subjHasObject = true;

					useLeftEf = false;
					useRightEf = false;

					addContacts = false;
					removeContacts = false;

					pickNearestHand = true;

				}


				if( (!walkFwdAgain) && (posTaskL->eval().norm()) <0.05 && (posTaskR->eval().norm() <0.05) )
				{

					if(Flag_Walk)
					{
						walkBack = true;
						ctl.config().add("finishedWalk", false);

						ctl.postureTask->reset();

						ctl.solver().removeTask(posTaskL);
						ctl.solver().removeTask(oriTaskL);

						ctl.solver().removeTask(posTaskR);
						ctl.solver().removeTask(oriTaskR);

						handoverComplete = true;

						LOG_WARNING("------------------------------> now robot will walk back again\n")
					}
					else
					{
						walkBack = false;
						LOG_SUCCESS("------------------------------> Handover routine completed, begin next trial\n")
					}

					restartHandover = false;

				}

			}//restartHandover


			/*
			*  10th, final
			*/
			if(handoverComplete && Flag_Walk && ctl.isLastDSP() )
			{
				finishedWalk_ = ctl.config()("finishedWalk", false);

				if(finishedWalk_)
				{
					finishedWalk_ = false;
					ctl.config().add("finishedWalk", false);

					ctl.solver().addTask(posTaskL);
					ctl.solver().addTask(oriTaskL);
					ctl.solver().addTask(posTaskR);
					ctl.solver().addTask(oriTaskR);

					posTaskL->reset();
					oriTaskL->reset();
					posTaskR->reset();
					oriTaskR->reset();

					handoverComplete = false;

					LOG_SUCCESS("------------------------------> Handover routine completed, begin next trial\n")
				}
				else
				{
					ctl.postureTask->reset();
				}
			}
		}

		return false;

	}// forceControllerTogether





	bool ApproachObject::forceControllerIndividual(
		bool& enableHand,
		sva::PTransformd X_0_rel,
		Eigen::Vector3d relaxPos,
		Eigen::Vector3d initPos,
		Eigen::Matrix3d initRot,
		Eigen::Vector3d handForce,
		Eigen::Vector3d forceSurf,
		Eigen::Vector3d thesh,
		Eigen::Vector3d efAce,
		std::shared_ptr<mc_tasks::PositionTask>& posTask,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
		std::string gripperName,
		std::vector<std::string> robotMarkersName,
		std::vector<std::string> lShpMarkersName,
		double obj_rel_robotHand)
	{

		Eigen::Vector3d ef_wA_O, ef_wA_wB, ef_wA_gA, ef_wA_gB, ef_wA_f;

		double ef_wAB_theta_wAO;
		double ef_wAB_theta_wAgA;
		double ef_wAB_theta_wAgB;
		double ef_wAB_theta_wAf;

		double ef_area_wAB_O;
		double ef_area_wAB_gA;
		double ef_area_wAB_gB;

		double fin_rel_ef, obj_rel_ef, obj_rel_subj;

		Eigen::Vector3d nearestFingerPos, gripperEf;

		nearestFingerPos = markersPos[markers_name_index[lShpMarkersName[0]]].col(i);

		/*direction vectors, projections and area*/
		ef_wA_O  =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - objectPosC;
		ef_wA_f  =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - nearestFingerPos;
		ef_wA_wB =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[1]]].col(i);
		ef_wA_gA =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[2]]].col(i);
		ef_wA_gB =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[3]]].col(i);

		ef_wAB_theta_wAO = acos( ef_wA_wB.dot(ef_wA_O)/( ef_wA_wB.norm()*ef_wA_O.norm() ) );
		ef_wAB_theta_wAf = acos( ef_wA_wB.dot(ef_wA_f)/( ef_wA_wB.norm()*ef_wA_f.norm() ) );
		ef_wAB_theta_wAgA = acos( ef_wA_wB.dot(ef_wA_gA)/( ef_wA_wB.norm()*ef_wA_gA.norm() ) );
		ef_wAB_theta_wAgB = acos( ef_wA_wB.dot(ef_wA_gB)/( ef_wA_wB.norm()*ef_wA_gB.norm() ) );

		ef_area_wAB_O  = 0.5*ef_wA_wB.norm()*ef_wA_O.norm()*sin(ef_wAB_theta_wAO);
		ef_area_wAB_gA = 0.5*ef_wA_wB.norm()*ef_wA_gA.norm()*sin(ef_wAB_theta_wAgA);
		ef_area_wAB_gB = 0.5*ef_wA_wB.norm()*ef_wA_gB.norm()*sin(ef_wAB_theta_wAgB);


		gripperEf = 0.5*( markersPos[markers_name_index[robotMarkersName[2]]].col(i) + markersPos[markers_name_index[robotMarkersName[3]]].col(i) );

		fin_rel_ef   = (gripperEf  - nearestFingerPos).norm();
		obj_rel_subj = (objectPosC - nearestFingerPos).norm();
		obj_rel_ef   = (gripperEf  - objectPosC).norm();


		/*check for false r-to-h trial during object pulling*/
		if( rh_fail && (fin_rel_ef < 0.15) && (obj_rel_ef > 0.5) && (!goBackInitPose) )
		{
			count_rh_fail++;
			count_rh_success--;
			rh_fail = false;

			LOG_ERROR("------------------------------> did robot drop the object ?")
		}


		if( fin_rel_ef < 0.35 )
		{

			/*
			*  7th
			*/
			auto checkForce = [&](const char *axis_name, int idx)
			{

				if( fin_rel_ef < 0.15 ) // not effective n efficient
				/*try to check if (Fpull > Th) for cont. over 1 sec or more*/
				{
					if(enableHand)
					{
						enableHand = false;

						t7 = difftime( time(0), start);

						LOG_WARNING("------------------------------> trying to pull object, motion stopped")
					}

					Finert = objMassZ * efAce;

					Fpull[0] = abs(handForce[0]) - abs(Finert[0]) - abs(Fzero[0]);
					Fpull[1] = abs(handForce[1]) - abs(Finert[1]) - abs(Fzero[1]);
					Fpull[2] = abs(handForce[2]) - abs(Finert[2]) - abs(Fzero[2]);

					if( (abs(Fpull[idx]) > newTh[idx]) )
					{

						efPosOfHandover = posTask->position();
						hPosOfHandover = nearestFingerPos;

						gOpen = true;
						restartHandover = true;
						takeBackObject = false;
						robotHasObject = false;

						pickaHand = false;

						if(goBackInitPose)
						{
							t8 = difftime( time(0), start);
							count_rh_success++;

							goBackInitPose = false;

							cout <<"------------------------------> object pulled, threshold on " << axis_name << " with pull force " << Fpull[idx]<< " reached on "<< gripperName + " with newTh " << newTh.transpose() <<endl;
							LOG_SUCCESS("------------------------------> object returned and estimated mass(Norm) was = " << objMassNorm)
						}

					}
				}
				return false;

			};


			/*
			*  1st, 6th
			*/
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

					LOG_INFO("------------------------------> 1st cycle, opening " << gripperName)
				}

				/*stop motion*/
				else if( (enableHand)
					&& (openGripper)
					&& (!closeGripper)
					&& (!restartHandover)
					&& (fin_rel_ef < MIN_ALLOWED_DIST) ) //.12
				{
					Fzero = handForce;
					localSurf_Fzero = forceSurf;

					enableHand = false;
					t3 = difftime( time(0), start);

					LOG_WARNING("------------------------------> motion stopped with Fzero Norm "<< Fzero.norm())
				}

				/*closed WITH object*/
				else if( (!enableHand)
					&& (graspObject)
					&& ( abs( (forceSurf - localSurf_Fzero)(2) ) > MIN_SURFACE_FORCE )
					//  && ( (ef_area_wAB_gA > ef_area_wAB_O) || (ef_area_wAB_gB > ef_area_wAB_O) )
					)
				{
					gClose = true;
					closeGripper = true;
					graspObject = false;

					Fclose = handForce;

					t4 = difftime( time(0), start);

					efPosOfHandover = posTask->position();
					hPosOfHandover = nearestFingerPos;

					LOG_INFO("------------------------------> closing with Fclose Norm "<<Fclose.norm() << ",	is object inside gripper?")
				}

				/*closed WITHOUT object*/
				else if( (!restartHandover) &&
					(!graspObject) &&
					(obj_rel_robotHand > 0.1) &&
					(obj_rel_robotHand < 0.2) &&
					(ef_area_wAB_gA < ef_area_wAB_O) &&
					(ef_area_wAB_gB < ef_area_wAB_O) )
				{
					if( (Fclose.norm() < 2.0) )
					{
						gClose = false;
						closeGripper = false;
						graspObject = true;
						gOpen = true;

						count_hr_fail++;

						t_falseClose = difftime( time(0), start);

						LOG_ERROR("------------------------------> false close, try with object again")
					}
					else
						{ Fclose = Eigen::Vector3d(1, 1, 1); }
				}

			}

		}

		/*
		* 5th
		*/
		/*trigger again walk fwd when robot has the object*/
		if( walkFwdAgain && robotHasObject && takeBackObject && ( fin_rel_ef < MAX_ALLOWED_DIST ) )
		{

			if( (nearestFingerPos(2)>=objAboveWaist) )
			{
				walkFwd = true;
				walkFwdAgain = false;
				LOG_WARNING("------------------------------> walk Fwd again, are you here in the beginning of 2nd cycle?")
			}

		}

		/*
		*  8th
		*/
		/*just before the end of 2nd cycle, move EF to initial position at end of handover routine*/
		if( restartHandover && (fin_rel_ef > 0.45) )
		{

			if( (!gClose) && (!goBackInitPose) )
			{
				posTask->position(initPos);
				oriTask->orientation(initRot);

				gClose = true;
				goBackInitPose = true;

				t9 = difftime( time(0), start);

				LOG_WARNING("------------------------------> robot will move to half-sit pose")
			}
		}


		/*in between 1st and 2nd cycles*/
		if( fin_rel_ef > 0.5 )
		{
			/*
			*  2nd
			*/
			/*comes only if object is grasped*/
			if( (closeGripper) && (!restartHandover) && subjHasObject )
			{

				if(e == 2)
				{
					t5 = difftime( time(0), start);

					count_hr_success++;
				}

				if( (e%200==0) )//wait xx sec
				{
					Fload <<
					accumulate( Floadx.begin(), Floadx.end(), 0.0)/double(Floadx.size()),
					accumulate( Floady.begin(), Floady.end(), 0.0)/double(Floady.size()),
					accumulate( Floadz.begin(), Floadz.end(), 0.0)/double(Floadz.size());

					/*load force in the direction of gravity in world*/
					objMassZ = Fload(2)/GRAVITY;
					objMassNorm = Fload.norm()/GRAVITY;

					LOG_SUCCESS("------------------------------> Robot has object of mass(Z) = "<< objMassZ)
					cout<< "------------------------------> Fload was "<< Fload.transpose() << endl;

					/*new threshold*/
					newTh = Fload + thesh;

					if(subjHasObject)
					{
						/*move EF to initial position*/
						// posTask->position(initPos);
						posTask->position(Eigen::Vector3d(X_0_rel.translation()(0)+0.25, relaxPos(1), relaxPos(2)));
						oriTask->orientation(initRot);

						subjHasObject = false;
						robotHasObject = true;

						LOG_SUCCESS("------------------------------> Ef returning to relax pose")
					}

					/*clear vector memories*/
					Floadx.clear(); Floady.clear(); Floadz.clear();
				}
				else /*averaging load force to get object mass*/
				{
					Floadx.push_back( abs(handForce[0]) );
					Floady.push_back( abs(handForce[1]) );
					Floadz.push_back( abs(handForce[2]) );

					// Floadx.push_back( abs( abs(handForce[0])-abs(Fzero[0]) ) );
					// Floady.push_back( abs( abs(handForce[1])-abs(Fzero[1]) ) );
					// Floadz.push_back( abs( abs(handForce[2])-abs(Fzero[2]) ) );
				}
				e+=1;

			}

			/*
			*  3rd
			*/
			/*walk back when object arrives at relax pose*/
			if( (!takeBackObject) && (!walkBack) && abs( abs(X_0_rel.translation()(0)) - abs(objectPosC(0)) )<0.25 )
			{

				if(Flag_Walk)
				{
					ctl.postureTask->reset();
					ctl.solver().removeTask(posTask);
					ctl.solver().removeTask(oriTask);
				}

				if(robotHasObject)
				{
					takeBackObject = true;
					walkBack = true;

					LOG_SUCCESS("------------------------------> robot returning to relax pose")
				}

			}

			/*
			*  4th
			*/
			/*add ef task again*/
			if( walkBack && robotHasObject && (!enableHand) )
			{

				if(Flag_Walk && ctl.isLastDSP() )
				{

					/*true when last DSP is finished*/
					finishedWalk_ = ctl.config()("finishedWalk", false);

					if(finishedWalk_)
					{
						ctl.config().add("finishedWalk", false);

						ctl.solver().addTask(posTask);
						posTask->reset();

						ctl.solver().addTask(oriTask);
						oriTask->reset();

						enableHand = true;
						walkFwdAgain = true;
					}
					else
					{
						ctl.postureTask->reset();
					}
				}
				else
				{
					enableHand = true;
				}
				LOG_INFO("------------------------------> ready to begin 2nd cycle, motion enabled")
			}

			/*
			*  9th
			*/
			/*at the end of 2nd cycle*/
			if(restartHandover)
			{

				if(!subjHasObject)
				{

					e = 1;
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
					rh_fail = true;

					openGripper = false;
					closeGripper = false;

					graspObject = true;
					enableHand = true;

					startNow = false;

					subjHasObject = true;

					useLtEf=  true;
					stopLtEf = true;

					useRtEf = true;
					stopRtEf = true;

				}


				if( (!walkFwdAgain) && (posTask->eval().norm()) < 0.05 )
				{

					if(Flag_Walk)
					{
						walkBack = true;
						ctl.config().add("finishedWalk", false);

						ctl.postureTask->reset();

						ctl.solver().removeTask(posTask);
						ctl.solver().removeTask(oriTask);

						handoverComplete = true;

						LOG_WARNING("------------------------------> now robot will walk back again\n")
					}
					else
					{
						walkBack = false;
						LOG_SUCCESS("------------------------------> Handover routine completed, begin next trial\n")
					}

					restartHandover = false;

				}

			}//restartHandover


			/*
			*  10th, final
			*/
			if(handoverComplete && Flag_Walk && ctl.isLastDSP() )
			{
				finishedWalk_ = ctl.config()("finishedWalk", false);

				if(finishedWalk_)
				{
					finishedWalk_ = false;
					ctl.config().add("finishedWalk", false);

					ctl.solver().addTask(posTask);
					ctl.solver().addTask(oriTask);

					posTask->reset();
					oriTask->reset();

					handoverComplete = false;

					LOG_SUCCESS("------------------------------> Handover routine completed, begin next trial\n")
				}
				else
				{
					ctl.postureTask->reset();
				}
			}
		}

		return false;

	}// forceControllerIndividual

}//namespace lipm_walking
