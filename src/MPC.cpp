#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.05;

// set vehicle front length
const double Lf = 2.67;

// set reference velocity
double ref_v = 60*0.44704;

// establish when one variable starts and another ends
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
	// (section operator largely copied from tutorial website:)
	// implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // The cost is stored is the first element of `fg`.
	// Any additions to the cost should be added to `fg[0]`.
	fg[0] = 0;

	// Reference State Cost
	for(unsigned int i=0; i<N; i++) {
		fg[0] += 2.*CppAD::pow(vars[cte_start+i],2);
	    fg[0] += 2.*CppAD::pow(vars[epsi_start+i],2);
	    fg[0] += CppAD::pow(vars[v_start+i]-ref_v,2);
	}

	// Minimize the use of actuators.
	for(unsigned int t = 0; t < N - 1; t++) {
	   	fg[0] += 10.*CppAD::pow(vars[delta_start + t], 2);
	   	fg[0] += CppAD::pow(vars[a_start + t], 2);
	}

	// Minimize the value gap between sequential actuations.
	for(unsigned int t = 0; t < N - 2; t++) {
	  	fg[0] += 50000.*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	  	fg[0] += 500.*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	}

	// Setup Constraints
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for(unsigned int t = 1; t < N; t++) {
    	// The state at time t+1.
	    AD<double> x1 = vars[x_start + t];
	    AD<double> y1 = vars[y_start + t];
	    AD<double> psi1 = vars[psi_start + t];
	    AD<double> v1 = vars[v_start + t];
	    AD<double> cte1 = vars[cte_start + t];
	    AD<double> epsi1 = vars[epsi_start + t];
	    // The state at time t.
     	AD<double> x0 = vars[x_start + t - 1];
    	AD<double> y0 = vars[y_start + t - 1];
      	AD<double> psi0 = vars[psi_start + t - 1];
      	AD<double> v0 = vars[v_start + t - 1];
      	AD<double> epsi0 = vars[epsi_start + t - 1];

      	// Only consider the actuation at time t.
        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> a0 = vars[a_start + t - 1];

        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2);
        AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);

        // Here's `x` to get you started.
        // The idea here is to constraint this value to be 0.
        //
        // NOTE: The use of `AD<double>` and use of `CppAD`!
        // This is also CppAD can compute derivatives and pass
        // these to the solver.

        // Setup the rest of the model constraints
        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
        fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
        fg[1 + cte_start + t] = cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {this->steer_value = 0; this->throttle_value = 0;}
MPC::~MPC() {}

vector<vector<double>> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N*6 + (N-1)*2;
  // Set the number of constraints
  size_t n_constraints = N*6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // (from tutorial website):
  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (unsigned int i = 0; i < delta_start; i++) {
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1.0e19;
  }

  // (from tutorial website:)
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (unsigned int i = delta_start; i < a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
  }

  // (from tutorial website:)
  // Acceleration/decceleration upper and lower limits.
  for (unsigned int i = a_start; i < n_vars; i++) {
      vars_lowerbound[i] = -1.0;
      vars_upperbound[i] = 1.0;
  }

  // (from tutorial website:)
  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> x_mpc;
  for(unsigned int i=1; i<y_start; i++) {
	  x_mpc.push_back(solution.x[i]);
  }
  vector<double> y_mpc;
    for(unsigned int i=y_start+1; i<psi_start; i++) {
  	  y_mpc.push_back(solution.x[i]);
    }
  this->steer_value = solution.x[delta_start];
  this->throttle_value = solution.x[a_start];

  return {x_mpc, y_mpc};
}
