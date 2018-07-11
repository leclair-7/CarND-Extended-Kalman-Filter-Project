#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  	// made it different than 4 elements like in the class material; need it to adapt to whatever size it needs to be
  	// also it seems as though different sensor packages with require different error calculations
  	
  	VectorXd rmse(4);

	if (estimations.size() != ground_truth.size() || estimations.size() == 0 )
	{
		cout<< "Invalid estimations of ground_truth size"<<endl;
		return rmse;
	}
		
	
	VectorXd temp(4);

	rmse<<0.0,0.0,0.0,0.0;

	for(unsigned int i=0; i < estimations.size(); ++i){
		temp = (estimations[i].array() - ground_truth[i].array() );
		rmse = rmse.array() + temp.array() * temp.array();
	}
	
	rmse = (1.0/estimations.size()) * rmse;
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    /**
      Calculates a Jacobian here.
    */

	MatrixXd Hj(3,4);

	if (x_state.size() != 4){
		cout<< "x_state is not length 4, you were wrong" << endl;
	}
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	
	if (px < .00001 || py < .00001){
	    //cout<<"Error, Divide by zero (almost) " <<endl;
	    //return Hj;
	    px = .001;
	    py = .001;
	}
    Hj <<
        px/pow(px*px+py*py,.5),py/pow(px*px+py*py,.5),0,0,
        -1*py/(px*px+py*py),px/(px*px+py*py),0,0,
        py*(vx*py - vy*px)/(pow((px*px+py*py),1.5)),
        px * (vy*px-vx*py )  / (pow((px*px+py*py),1.5)),
        px/pow(px*px+py*py,.5),py/pow(px*px+py*py,.5)
        ;
	
	return Hj;
}
