#include "handover_approachObject.h"

namespace mc_handover
{
	ApproachObject::ApproachObject() {}
	// { cout<<"\033[1;50mHandover object created\033[0m\n"; }



	ApproachObject::~ApproachObject() {}
	// { cout<<"\033[1;50mHandover object destroyed\033[0m\n"; }



	/*allocate memory*/
	void ApproachObject::initials()
	{
		/*markers Name strings*/
		strMarkersBodyName = {"4mars_robot_left_hand", "4mars_robot_right_hand", "8mars_obj_subj_hands", "2mars_subj_head"};

		robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};//0-3 + dummy
		robotRtMarkers = {"wristRtEfA", "wristRtEfB", "gripperRtEfA", "gripperRtEfB"};//4-7
																			// object //8
		subjRtMarkers = {"lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD"};//9-12
		subjLtMarkers = {"lShapeLtA",              "lShapeLtC", "lShapeLtD"};//13-15
		subjMarkers = {"lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD", "lShapeLtA", "lShapeLtC", "lShapeLtD"}; //9-15

		subjHeadMarkers = {"head1", "head2"};//16-17 + dummyHead


		strMarkersName.insert(strMarkersName.begin(), robotLtMarkers.begin(), robotLtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), robotRtMarkers.begin(), robotRtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), "object");
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
		/*check for non zero frame only and store them*/
		if( checkFrameOfData(Markers) )
		{
			i+=1;
			for(int m=0; m<totalMarkers; m++)
				{ markersPos[m].col(i) << Markers[m]; }

			/*for GUI*/
			objectPos = markersPos[8].col(i);
			fingerPosR = markersPos[9].col(i);
			fingerPosL = markersPos[13].col(i);

			headPos1 = markersPos[16].col(i);//head left
			headPos2 = markersPos[17].col(i);//head right
			headPos = 0.5*(headPos1 + headPos2);

			/*move EF when subject approaches object 1st time*/
			obj_rel_subjRtHand = ( fingerPosR - objectPos ).norm();//lshpRtA - obj
			obj_rel_subjLtHand = ( fingerPosL - objectPos ).norm();//lshpLtA - obj

			obj_rel_robotLtHand = ( markersPos[2].col(i) - objectPos ).norm();//gripperLtEfA - obj
			obj_rel_robotRtHand = ( markersPos[6].col(i) - objectPos ).norm();//gripperRtEfA - obj

			return true;
		}
		else
			{ return false; }
	}



	std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> ApproachObject::predictionController(
		const Eigen::Vector3d& curPosEf,
		const Eigen::Matrix3d& constRotLink6,
		std::vector<std::string> subjMarkersName)
	{
		bool ready{false};

		Eigen::Vector3d x, y, z, lshp_X, lshp_Y, lshp_Z;
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

		X_M_Subj = sva::PTransformd(subjHandRot, predictPos);

		X_R_ef = sva::PTransformd(constRotLink6, curPosEf);
		X_R_M = sva::PTransformd(idtMat, Eigen::Vector3d(0., 0., 0.));

		X_ef_Subj = X_M_Subj * X_R_M * X_R_ef.inv();

		handoverRot = X_ef_Subj.rotation().transpose();

		/*** way points for robot ef to predict pos ****/
		wp_ef_Subj=handoverTraj->constVelocity(curPosEf, predictPos, t_predict);
		// wp_ef_Subj=handoverTraj->constVelocity(curPosEf, X_ef_Subj.translation(), t_predict);
		wp = get<0>(wp_ef_Subj);
		initRefPos << wp(0,it), wp(1,it), wp(2,it);

		ready = true;

		handRot = subjHandRot.transpose();

		return std::make_tuple(ready, wp, initRefPos, handoverRot);
	}



	bool ApproachObject::goToHandoverPose(
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
		Eigen::Vector3d fingerPos)
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
			handoverPos = fingerPos;
		}

		/*robot constraint*/
		if(enableHand &&
			(handoverPos(0)>= 0.10) && (handoverPos(0)<= 0.7) &&
			(handoverPos(1)>= min)  && (handoverPos(1)<= max) &&
			(handoverPos(2)>= 0.80) && (handoverPos(2)<= 1.4)
			)
		{
			sva::PTransformd new_pose(get<3>(handPredict), handoverPos);
			posTask->position(new_pose.translation());
			oriTask->orientation(new_pose.rotation());

			if( (takeBackObject) && (bool_t6) )
			{
				bool_t6 = false;
				t6 = difftime( time(0), start);
			}


			// LOG_INFO("it "<<it << "\npredictPos wp "<<handoverPos.transpose()<<"\n" << "fingerPos "<< fingerPos.transpose())
		}

		return true;
	}



	bool ApproachObject::forceController(
		bool& enableHand,
		Eigen::Vector3d constPos,
		Eigen::Vector3d initPos,
		Eigen::Matrix3d initRot,
		Eigen::Vector3d handForce,
		Eigen::Vector3d ForceLo,
		Eigen::Vector3d Th,
		Eigen::Vector3d efAce,
		std::shared_ptr<mc_tasks::PositionTask>& posTask,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
		std::string gripperName,
		std::vector<std::string> robotMarkersName,
		std::vector<std::string> subjMarkersName,
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
		double ef_area_wAB_f;

		double subj_rel_ef, obj_rel_ef;

		Eigen::Vector3d fingerPos, gripperEf;

		fingerPos = markersPos[markers_name_index[subjMarkersName[0]]].col(i);

		/*direction vectors, projections and area*/
		ef_wA_O  =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - objectPos;
		ef_wA_f  =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - fingerPos;
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
		ef_area_wAB_f  = 0.5*ef_wA_wB.norm()*ef_wA_f.norm()*sin(ef_wAB_theta_wAf);
		ef_area_wAB_gA = 0.5*ef_wA_wB.norm()*ef_wA_gA.norm()*sin(ef_wAB_theta_wAgA);
		ef_area_wAB_gB = 0.5*ef_wA_wB.norm()*ef_wA_gB.norm()*sin(ef_wAB_theta_wAgB);


		gripperEf = 0.5*( markersPos[markers_name_index[robotMarkersName[2]]].col(i) + markersPos[markers_name_index[robotMarkersName[3]]].col(i) );

		subj_rel_ef = (gripperEf - fingerPos).norm();
		obj_rel_ef = (gripperEf - objectPos).norm();


		/*check for false r-to-h trial*/
		if( rh_fail && (subj_rel_ef < 0.15) && (obj_rel_ef > 0.5) && (!goBackInit) )
		{
			count_rh_fail++;
			count_rh_success--;
			rh_fail = false;

			LOG_ERROR("did robot drop the object ?")
		}


		if( subj_rel_ef < 0.3 )
		{

			auto checkForce = [&](const char *axis_name, int idx)
			{
				// /*new threshold*/
				// newTh = Fload + Th;

				// objMass = Fload.norm()/9.81;

				if( subj_rel_ef < 0.15 )
				{
					if(enableHand)
					{
						enableHand = false;
						t7 = difftime( time(0), start);
						LOG_WARNING("2nd cycle, motion stopped, try to retreat object")
					}

					Finert = objMass * efAce;

					Fpull[0] = abs(handForce[0]) - abs(Finert[0]) - abs(Fzero[0]);
					Fpull[1] = abs(handForce[1]) - abs(Finert[1]) - abs(Fzero[1]);
					Fpull[2] = abs(handForce[2]) - abs(Finert[2]) - abs(Fzero[2]);

					if( (abs(Fpull[idx]) > newTh[idx]) )
					{
						gOpen = true;
						restartHandover = true;
						takeBackObject = false;
						pickaHand = false;

						if(goBackInit)
						{
							t8 = difftime( time(0), start);
							count_rh_success++;

							goBackInit = false;

							LOG_SUCCESS("object pulled, threshold on " << axis_name << " with pull force " << Fpull[idx]<< " reached on "<< gripperName + " with newTh " << newTh.transpose())
							cout << gripperName + "_Forces at Grasp "<< handForce.transpose() <<endl;
							cout << "Finert "<< Finert.transpose() <<endl;
							cout <<"object mass " << objMass <<endl;
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
					LOG_INFO("opening " + gripperName)
				}

				/*stop motion*/
				else if( (enableHand) && (openGripper) && (!closeGripper) && (!restartHandover) && (subj_rel_ef < 0.12) )
				{
					Fzero = handForce;
					local_Fzero = ForceLo;

					enableHand = false;
					t3 = difftime( time(0), start);

					LOG_WARNING("motion stopped with Fzero Norm "<< Fzero.norm())
				}

				/*closed WITH object*/
				else if( (!enableHand) && (graspObject) &&
					// ( (ef_area_wAB_gA > ef_area_wAB_O) || (ef_area_wAB_gB > ef_area_wAB_O) )
					( abs( (ForceLo - local_Fzero)(2) ) > 4.0 )
					)
				{
					gClose = true;
					closeGripper = true;
					graspObject = false;

					Fclose = handForce;
					t4 = difftime( time(0), start);
					LOG_INFO("closing with Fclose Norm "<<Fclose.norm() << ",	is object inside gripper?")
				}

				/*closed WITHOUT object*/
				else if( (!restartHandover) && (!graspObject) && (ef_area_wAB_gA < ef_area_wAB_O) && (ef_area_wAB_gB < ef_area_wAB_O) && (obj_rel_robotHand < 0.2) && (0.1 < obj_rel_robotHand) )
				{
					if( (Fclose.norm() < 2.0) )
					{
						gClose = false;
						closeGripper = false;
						graspObject = true;

						count_hr_fail++;

						gOpen = true;
						t_falseClose = difftime( time(0), start);
						LOG_ERROR("false close, try with object again")
					}
					else
					{ Fclose = Eigen::Vector3d(1,1,1); }
				}
			}
		}



		/*restart handover*/
		if( subj_rel_ef > 0.5 )
		{

			/*comes only if object is grasped*/
			if( (closeGripper) && (!restartHandover) && (!enableHand) )
			{
				if(e == 2)
				{
					t5 = difftime( time(0), start);
					count_hr_success++;
				}

				if( (e%200==0) )//wait xx sec
				{
					enableHand = true;
					takeBackObject = true;

					Fload <<
					accumulate( Floadx.begin(), Floadx.end(), 0.0)/double(Floadx.size()),
					accumulate( Floady.begin(), Floady.end(), 0.0)/double(Floady.size()),
					accumulate( Floadz.begin(), Floadz.end(), 0.0)/double(Floadz.size());

					LOG_SUCCESS("robot has object, motion enabled, Fload "<< Fload.transpose() << ", EF returning to init pose" )


					/*try "worldWrench" with gravity*/
					objMass = Fload.norm()/9.81;

					/*new threshold*/
					newTh = Fload + Th;

					/*clear vector memories*/
					Floadx.clear(); Floady.clear(); Floadz.clear();

					/*move EF to initial position*/
					posTask->position(initPos);
					oriTask->orientation(initRot);
				}
				else /*divide by 9.81 and you will get object mass*/
				{
					Floadx.push_back( abs( abs(handForce[0])-abs(Fzero[0]) ) );
					Floady.push_back( abs( abs(handForce[1])-abs(Fzero[1]) ) );
					Floadz.push_back( abs( abs(handForce[2])-abs(Fzero[2]) ) );
				}
				e+=1;
			}


			if(restartHandover)
			{
				/*move EF to initial position*/
				if(!goBackInit)
				{
					posTask->position(constPos);
					oriTask->orientation(initRot);

					if(!gClose)
					{
						t9 = difftime( time(0), start);
						gClose = true;
					}
				}

				openGripper = false;
				closeGripper = false;

				graspObject = true;
				goBackInit = true;
				enableHand = true;
				e = 1;

				if(restartHandover && posTask->eval().norm() <0.05)
				{
					posTask->position(initPos);
					restartHandover = false;

					useLtEf=  true;
					stopLtEf = true;

					useRtEf = true;
					stopRtEf = true;

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

					LOG_INFO("object returned to subject, motion enabled, restarting handover\n")
				}
			}
		}
		return false;
	}

}//namespace mc_handover
