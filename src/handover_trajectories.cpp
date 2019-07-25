
#include "handover_trajectories.h"


namespace mc_handover
{
	HandoverTrajectory::HandoverTrajectory()
	{
		std::cout << " handover trajectory constructor created " <<std::endl;
	}


    /* returns way points between xi and xf in tf time using nonZero boundary conditions */
	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> HandoverTrajectory::minJerkZeroBoundary(const Eigen::Vector3d & xi, const Eigen::Vector3d & xf, double tf)
	{ 
		Eigen::MatrixXd a, b1, b2, b3, pos, vel, ace;

		a.resize(tf,3);

		b1.resize(tf,1); b2.resize(tf,1); b3.resize(tf,1);

		pos.resize(tf,3); vel.resize(tf,3);	ace.resize(tf,3);

		for(int i=0; i<tf; i++)
		{
			a.row(i) = (xf-xi);
			b1(i) = (10*pow((i/tf),3) -  15*pow((i/tf),4) +    6*pow((i/tf),5));
			b2(i) = (30*pow((i/tf),2) -  60*pow((i/tf),3) +   30*pow((i/tf),4));
			b3(i) = (60*(i/tf)        - 180*pow((i/tf),2) +  120*pow((i/tf),3));

			pos.row(i) =  xi + (a.row(i).transpose()*b1(i));
			vel.row(i) =       (a.row(i).transpose()*b2(i));
			ace.row(i) =       (a.row(i).transpose()*b3(i));
		}
		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		// cout << ace << endl<< endl;
		return std::make_tuple(pos, vel, ace);
	}



    /* returns way points between xi and xf in tf time using nonZero boundary conditions */
	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>  HandoverTrajectory::minJerkNonZeroBoundary(const Eigen::Vector3d & xi, const Eigen::Vector3d & vi, const Eigen::Vector3d & ai, const Eigen::Vector3d & xc, double tf)
	{

		Eigen::MatrixXd a0, a1, a2, a3, a4, a5;
		Eigen::MatrixXd b1, b2, b3, pos, vel, ace;

		double tau, D;

		a0.resize(tf,3); a1.resize(tf,3); a2.resize(tf,3);
		a3.resize(tf,3); a4.resize(tf,3); a5.resize(tf,3);

		b1.resize(tf,1); b2.resize(tf,1); b3.resize(tf,1);

		pos.resize(tf,3); vel.resize(tf,3); ace.resize(tf,3);

		for(int i=0; i<tf; i++)
		{	
			D  = 1/tf;
			tau = i*D;

			a0.row(i) = xi;
			a1.row(i) = tf*vi;
			a2.row(i) = tf*ai.array()/2;
			a3.row(i) = (3*ai*pow((tf),2))/2 - (6*tf*vi) + (10*(xc-xi));
			a4.row(i) = (3*ai*pow((tf),2))/2 + (8*tf*vi) - (15*(xc-xi));
			a5.row(i) = -(ai*pow((tf),2))/2  - (3*tf*vi) + (6*(xc-xi));

			pos.row(i) = a0.row(i) + a1.row(i)*tau + a2.row(i)*pow((tau),2) + a3.row(i)*pow((tau),3)
			+ a4.row(i)*pow((tau),4) + a5.row(i)*pow((tau),5);

			vel.row(i) =			a1.row(i)/D + 2*a2.row(i)*tau/D + 3*a3.row(i)*pow((tau),2)/D
			+ 4*a4.row(i)*pow((tau),3)/D + 5*a5.row(i)*pow((tau),4)/D;


			ace.row(i) = 						 2*a2.row(i)/pow((D),2) + 6*a3.row(i)*tau/pow((D),2)
			+ 12*a4.row(i)*pow((tau),2)/pow((D),2) + 20*a5.row(i)*pow((tau),3)/pow((D),2);
		}
		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		// cout << ace << endl<< endl;
		return std::make_tuple(pos, vel, ace);
	}



	/* return xf in time tf based on current xc */
	std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> HandoverTrajectory::minJerkPredictPos(const Eigen::Vector3d & xi, const Eigen::Vector3d & xc, double t0, double tc, double tf)
	{
		Eigen::Vector3d a, pos, vel, ace;
		
		double b1, b2, b3;
		double t = tc-t0;
		double d = tf-t0;
		double T = (t/d);
		
		a  << (xc-xi);
		b1 = (10*pow((T),3) -  15*pow((T),4) +    6*pow((T),5));
		b2 = (30*pow((T),2) -  60*pow((T),3) +   30*pow((T),4));
		b3 = (60*(T)        - 180*pow((T),2) +  120*pow((T),3));

		pos <<	xi.array() + (a.array()/b1);
		vel <<				 (a.array()/b2);
		ace <<				 (a.array()/b3);
		
		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		// cout << ace << endl<< endl;
		return std::make_tuple(pos, vel, ace);
	}



	/* position based on const velocity */
	std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d>HandoverTrajectory::constVelocity(const Eigen::Vector3d & xi, const Eigen::Vector3d & xf, double tf)
	{	
		MatrixXd pos;

		pos.resize(3,tf);

		Eigen::Vector3d slope		= -(xi-xf)/(tf*0.005);
		Eigen::Vector3d constant	=  xf-slope*tf*0.005;

		for(int i=0; i<tf; i++)
		{
			pos.col(i) = slope*i*0.005 + constant;
		}

		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		return std::make_tuple(pos, slope, constant);
	}



	/* position at tf time based on const velocity */
	Eigen::Vector3d HandoverTrajectory::constVelocityPredictPos(const Eigen::Vector3d & xdot, const Eigen::Vector3d & C, double tf)
	{	
		Eigen::Vector3d pos;
		pos << xdot*tf*0.005 + C;
		// cout << pos << endl<< endl;
		return pos;
	}



	Eigen::MatrixXd HandoverTrajectory::diff(Eigen::MatrixXd data)
	{
		Eigen::MatrixXd matrix_diff, check;
		matrix_diff.resize(data.rows(),data.cols());
		check.resize(data.rows(),data.cols());

		for (unsigned int i=1; i<data.cols(); i++)
		{
			// ignore diff > XXXX   replace with previous value
			check(0,i) = (data(0,i)-data(0,i-1));
			check(1,i) = (data(1,i)-data(1,i-1));
			check(2,i) = (data(2,i)-data(2,i-1));

			if( (check(0,i)>0.05) || (check(0,i)<-0.05) ||
				(check(1,i)>0.05) || (check(1,i)<-0.05) ||
				(check(2,i)>0.05) || (check(2,i)<-0.05) ) //meters
			{
				matrix_diff(0,i) = check(0,i-1);
				matrix_diff(1,i) = check(1,i-1);
				matrix_diff(2,i) = check(2,i-1);
			}
			else
			{
				matrix_diff(0,i) = check(0,i);
				matrix_diff(1,i) = check(1,i);
				matrix_diff(2,i) = check(2,i);
			}
		}
		// cout << matrix_diff.transpose()*200<< endl<<endl;
		return matrix_diff;
	}




	Eigen::Vector3d HandoverTrajectory::takeAverage(Eigen::MatrixXd m)
	{
		std::vector<double> v[m.rows()];
		double avg[2];//avg[m.rows()];

		Eigen::Vector3d mean;

		for(int j=0; j<m.rows(); j++)
		{
			for(int i=0;i<m.cols();i++)
			{
				v[j].push_back(m(j,i));
			// cout << v[j].at(i) << endl;
				avg[j]+=v[j].at(i);
			}
		// cout << avg[j]/v[j].size() << endl;
			mean(j)= avg[j]/v[j].size();
		}
		// cout<<"mean velocity " << mean.transpose()<<endl;
		return mean;
	}

	// Eigen::Vector3d HandoverTrajectory::takeAverage(Eigen::MatrixXd m)
	// {
	// 	std::vector<double> vx, vy, vz;
	// 	double avgx=0, avgy=0, avgz=0;

	// 	Eigen::Vector3d mean;

	// 	for(int i=1;i<=m.cols();i++)
	// 	{
	// 		//ignore 1st value
	// 		if( (m(0,i)>-2) && (m(0,i)<2) )
	// 		{
	// 			vx.push_back(m(0,i));
	// 		}
	// 		else
	// 		{
	// 			vx.push_back(0.3);	
	// 		}

	// 		if( (m(1,i)>-2) && (m(1,i)<2) )
	// 		{
	// 			vy.push_back(m(1,i));
	// 		}
	// 		else
	// 		{
	// 			vy.push_back(0.3);	
	// 		}
			
	// 		if( (m(2,i)>-2) && (m(2,i)<2) )
	// 		{
	// 			vz.push_back(m(2,i));
	// 		}
	// 		else
	// 		{
	// 			vz.push_back(0.3);	
	// 		}		
	// 	}

	// 	avgx = accumulate( vx.begin(), vx.end(), 0.0)/vx.size(); 
	// 	avgy = accumulate( vy.begin(), vy.end(), 0.0)/vy.size(); 
	// 	avgz = accumulate( vz.begin(), vz.end(), 0.0)/vz.size(); 

	// 	mean << avgx, avgy, avgz;
	// 	return mean;
	// }








	// CircularTrajectory::CircularTrajectory(double radius, std::size_t nr_points, const Eigen::Vector3d& initial)
	// : r(radius), nr_points(nr_points), x0(initial)
	// {
	// 	reset();
	// }

	// std::pair<Eigen::Vector3d, Eigen::Vector3d> CircularTrajectory::pop()
	// {
	// 	std::pair<Eigen::Vector3d, Eigen::Vector3d> pair;
	// 	if(queue.empty())
	// 	{
	// 		Eigen::Vector3d zero = Eigen::Vector3d::Zero();
	// 		pair = {x0, zero};
	// 	}
	// 	else
	// 	{
	// 		pair = queue.front();
	// 		queue.pop();
	// 	}
	// 	return pair;
	// }

	// void CircularTrajectory::reset()
	// {
	// 	//Clear queue
	// 	std::queue<std::pair<Eigen::Vector3d, Eigen::Vector3d> > empty;
	// 	std::swap(empty, queue);
	// 	for(std::size_t i = 0; i < nr_points; ++i)
	// 	{
	// 		double theta = 2*M_PI*(double)i/(double)nr_points;
	// 		Eigen::Vector3d pos(0, cos(theta), sin(theta));
	// 		Eigen::Vector3d vel(0, -sin(theta), cos(theta));
	// 		queue.push({x0+r*pos, r*vel});
	// 	}
	// }













	// HandoverTrajectoryTask::HandoverTrajectoryTask(mc_solver::QPSolver & solver)
	// :solver(solver)
	// {
	// 	auto & robot = solver.robot();

	// 	positionTask = std::make_shared<tasks::qp::PositionTask>(solver.robots().mbs(), 0, "LARM_LINK6", robot.mbc().bodyPosW[robot.bodyIndexByName("LARM_LINK6")].translation());

	// 	Eigen::Vector3d dimW; dimW << 1, 1, 1;

	// 	trajTask = std::make_shared<tasks::qp::TrajectoryTask>(solver.robots().mbs(), 0, positionTask.get(), gainPos, gainVel, dimW, weight);
	// 	solver.addTask(trajTask.get());

	// 	/* Update position vector */
	// 	initPos = positionTask->position();
	// 	cout << "efL initial Pos " << initPos.transpose() << endl;


	// 	pos = Eigen::MatrixXd(3,tunParam2);
	// 	vel = Eigen::MatrixXd(3,tunParam2);
	// 	ace = Eigen::MatrixXd(3,tunParam2);
	// }


	// HandoverTrajectoryTask::~HandoverTrajectoryTask()
	// {
	// 	solver.removeTask(trajTask.get());
	// }


	// bool HandoverTrajectoryTask::update()
	// {
	// 	// cout << "pos.rows() " << pos.rows() <<endl;
	// 	// cout << "pos.cols() " << pos.cols() <<endl;

 // 		if(wp_index < pos.cols())
	// 	{
	// 		// cout << " Pos " << pos.col(wp_index).transpose() << endl;

	// 		//************ change initPos after every itr *****************8

	// 		positionTask->position(pos.col(wp_index) + initPos - pos.col(0));
	// 		// positionTask->position(pos.col(wp_index) + initPos);
	// 		// positionTask->position(pos.col(wp_index));

	// 		trajTask->refVel(vel.col(wp_index));
	// 		refVel = vel.col(wp_index);
	// 		trajTask->refAccel(ace.col(wp_index));
	// 		refAce = ace.col(wp_index);
	// 		wp_index++;
	// 		// cout << "wp_index "<< wp_index <<endl;

	// 		// cout << " Pos " << positionTask->position().transpose() << endl;

	// 		if(wp_index==tunParam2-1) //check if observation time is over then go to new observed pos
	// 		{
	// 			LOG_INFO("next iteration ")
	// 			wp_index = 0;
	// 			initPos = positionTask->position();
	// 			return true;

	// 		}
	// 		return false;
	// 	}
	// 	else
	// 	{	
	// 		LOG_WARNING(" nothing to do ")
	// 		// wpReady = true;

	// 		positionTask->position(initPos);

	// 		trajTask->refVel(Eigen::Vector3d::Zero());
	// 		refVel=Eigen::Vector3d::Zero();
			
	// 		trajTask->refAccel(Eigen::Vector3d::Zero());
	// 		refAce=Eigen::Vector3d::Zero();
	// 		return false;
	// 	}
	// }



}// namespace mc_handover


