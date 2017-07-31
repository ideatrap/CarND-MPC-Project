
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 9; // total points
double dt = 0.135; // sets distance distance gap

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double v_target = 100;//Speed limit


//Fnd the starting index of each component, so that it can easily advance to the next item
size_t x_index = 0;
size_t y_index = x_index + N;
size_t psi_index = y_index + N;
size_t v_index = psi_index + N;
size_t cte_index = v_index + N;
size_t epsi_index = cte_index + N;
size_t delta_index = epsi_index + N; // steering
size_t a_index = delta_index + N - 1; //

class FG_eval {

public:
	// Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {

		// TODO: implement MPC
		// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.

		//reference: Udacity: https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ee21948d-7fad-4821-b61c-0d69bbfcc425


        // The cost is stored is the first element of `fg`.
		// add all errors to fg[] and multiply by the weight
        fg[0] = 0;

        // Reference State Cost
        // Define the cost weight related the reference state and
	    // any anything you think may be beneficial.


		// Weights for each factor
        double weight_cte = 2000.0; // weight for cross track error
        double weight_epsi = 6500.0; // weight for orientation
        double weight_v = 1.2; // weight for maintaining speed close to target
        double weight_delta = 1200.0; // weight for smooth throttle transition


        // Reference State Cost minimize  cross track, heading, and velocity errors
        //Minimize the difference between the states (cte, epsi, v) and desired
        for (int i = 0; i < N; i++) {
            fg[0] += weight_cte * CppAD::pow(vars[cte_index + i], 2); //Cross track error
            fg[0] += weight_epsi * CppAD::pow(vars[epsi_index + i], 2); //Orientation error
            fg[0] += weight_v * CppAD::pow(vars[v_index + i] - v_target, 2); //Speed error to the speed target
        }


		// if possible, avoid use actuations
        for (int i = 0; i < N - 1; i++) {
            fg[0] += CppAD::pow(vars[delta_index + i], 2);//steering
            fg[0] += CppAD::pow(vars[a_index + i], 2);//throttle
	    }

        //for smooth ride, error = difference between subsequent actuations
        for (int i = 0; i < N - 2; i++) {
            fg[0] += weight_delta * CppAD::pow(vars[delta_index + i + 1] - vars[delta_index + i], 2);
            fg[0] += CppAD::pow(vars[a_index + i + 1] - vars[a_index + i], 2);
        }

        // Setup model constraint with vars vector: [x,y,ψ,v,cte,eψ]   [δ,a]
        // Since fg[0] is zero, and all errors start from position 1
		// Other indices are bumped up by 1
        fg[1 + x_index] = vars[x_index];
        fg[1 + y_index] = vars[y_index];
        fg[1 + psi_index] = vars[psi_index];
        fg[1 + v_index] = vars[v_index];
        fg[1 + cte_index] = vars[cte_index];
        fg[1 + epsi_index] = vars[epsi_index];

        // The rest of the constraints
        for (int i = 0; i < N - 1; i++) {
            // The state at time i + 1
            AD<double> x1 = vars[x_index + i + 1];
            AD<double> y1 = vars[y_index + i + 1];
            AD<double> psi1 = vars[psi_index + i + 1];
            AD<double> v1 = vars[v_index + i + 1];
            AD<double> cte1 = vars[cte_index + i + 1];
            AD<double> epsi1 = vars[epsi_index + i + 1];

            // The state at time i
            AD<double> x0 = vars[x_index + i];
            AD<double> y0 = vars[y_index + i];
            AD<double> psi0 = vars[psi_index + i];
            AD<double> v0 = vars[v_index + i];
            AD<double> cte0 = vars[cte_index + i];
            AD<double> epsi0 = vars[epsi_index + i];

            //The actuation at time i
            AD<double> delta0 = vars[delta_index + i];//steering
            AD<double> a0 = vars[a_index + i];//throttle

            //3rd degree polynomial line prediction
            AD<double> f0  = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
            //desired psi at time i
            AD<double> psides0 = CppAD::atan(coeffs[1]+2*coeffs[2]*x0+3*coeffs[3]*x0*x0);

			// Setting up the rest of the model constraints
            fg[2 + x_index + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_index + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_index + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[2 + v_index + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_index + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_index + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double>
MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    // indx is the position of the N elements
    size_t indx;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector and actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    size_t n_vars = N * 6 + (N - 1) * 2;

	// TODO: Set the number of constraints
    size_t n_constraints = 6 * N;


    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

	// let the variable hold the current state value
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];


    // Set the initial variable values
    vars[x_index] = x;
    vars[y_index] = y;
    vars[psi_index] = psi;
    vars[v_index] = v;
    vars[cte_index] = cte;
    vars[epsi_index] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
	// TODO: Set lower and upper limits for variables.

	// Set all non-actuators upper and lower limits to the large numbers
    for (int i = 0; i < delta_index; i++) {
        vars_lowerbound[i] = -1.0e20;
        vars_upperbound[i] = 1.0e20;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (int i = delta_index; i < a_index; i++) {
		vars_lowerbound[i] = -0.45;
		vars_upperbound[i] = 0.45;
    }

    //Throttle limit
    for (int i = a_index; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

	//Set the upper lower bound to current value
    constraints_lowerbound[x_index] = x;
    constraints_lowerbound[y_index] = y;
    constraints_lowerbound[psi_index] = psi;
    constraints_lowerbound[v_index] = v;
    constraints_lowerbound[cte_index] = cte;
    constraints_lowerbound[epsi_index] = epsi;

    constraints_upperbound[x_index] = x;
    constraints_upperbound[y_index] = y;
    constraints_upperbound[psi_index] = psi;
    constraints_upperbound[v_index] = v;
    constraints_upperbound[cte_index] = cte;
    constraints_upperbound[epsi_index] = epsi;

    //Compute objective and constraints
    FG_eval fg_eval(coeffs);


    // NOTE: You don't have to worry about these options
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
    //auto cost = solution.obj_value;
    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.

    //Return the results
    vector<double> result = {solution.x[delta_index], solution.x[a_index]};
    for (indx = 0; indx < N; ++indx)
    {
        result.push_back(solution.x[x_index + indx]);
        result.push_back(solution.x[y_index + indx]);
    }
    return result;
}
