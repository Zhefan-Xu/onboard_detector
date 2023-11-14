/*
	FILE: kalman_filter.h
	--------------------------------------
	header of kalman_filter velocity estimator
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

namespace onboardDetector{
    class kalman_filter
    {
        private:
        // members
        bool is_initialized;
        MatrixXd states;
        MatrixXd A; // state matrix
        MatrixXd B; // input matrix
        MatrixXd H; // observation matrix
        MatrixXd P; // uncertianty
        MatrixXd Q; // process noise
        MatrixXd R; // obsevation noise

        public:
        // constructor
        kalman_filter();

        // set up the filter
        void setup(const MatrixXd& states,
                   const MatrixXd& A,
                   const MatrixXd& B,
                   const MatrixXd& H,
                   const MatrixXd& P,
                   const MatrixXd& Q,
                   const MatrixXd& R);

        // set A (sometimes sampling time will differ)
        void setA(const MatrixXd& A);

        // state estimate
        void estimate(const MatrixXd& z, const MatrixXd& u);

        // read output from the state
        double output(int state_index);
    };
}

#endif