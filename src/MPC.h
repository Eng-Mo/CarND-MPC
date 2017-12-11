#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct state_vector{
	vector<double> x_state;
	vector<double>y_state;
	vector<double> psi_stae;
	vector<double> v_state;
	vector<double> cte;
	vector<double> epsi;
	vector<double> delta_steering;
	vector<double> a;
};

class MPC {
 public:
	state_vector state_data;
	vector<double> mpc_x_vals;
	vector<double> mpc_y_vals;

	 double delta_prev {0};
	 double a_prev {0.1};

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
