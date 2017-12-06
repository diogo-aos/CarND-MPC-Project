#include "MPC.h"
#include <stdio.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

#include <fstream>
#include <sstream>
#include <string>

double v_ref = 40;

double K_cte = 1.0;
double K_epsi = 1.0;
double K_v = 1.0;
double K_actuator_delta = 1.0;
double K_actuator_a = 1.0;
double K_like_before_delta = 6000;
double K_like_before_a = 1.0;
double K_fast_turn = 1.0;
double K_fast_away = 1.0;

int N = 25;
double dt = 0.01;

using CppAD::AD;



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


// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
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
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.
    for(int t=0; t<N; t++){
      fg[0] += K_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += K_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += K_v * CppAD::pow(vars[v_start + t] - v_ref, 2);

      fg[0] += K_fast_turn * CppAD::pow(vars[v_start + t] * vars[epsi_start + t], 2);
      fg[0] += K_fast_away * CppAD::pow(vars[v_start + t] * vars[cte_start + t], 2);

    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += K_actuator_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += K_actuator_a * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += K_like_before_delta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += K_like_before_a * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }


    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    AD<double> Lf_ = Lf;
    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // 1+ because the first element in fg is the cost
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - ( psi0 + (v0 / Lf_) * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

      // AD<double> f0 = coeffs[0] +
      //                  coeffs[1] * x0 +
      //                  coeffs[2] * x0 * x0 +
      //                  coeffs[3] * x0 * x0 * x0;
      AD<double> f0 = 0.0;
      for (int i = 0; i < coeffs.size(); i++) {
        f0 += coeffs[i] * CppAD::pow(x0, i);
      }
      AD<double> calc_cte0 = f0 - y0;
      fg[1 + cte_start + t] = cte1 - (calc_cte0 + v0 * CppAD::sin(epsi0) * dt);

      //   f'(x0)
      // AD<double> df0 = coeffs[1] +
      //                   2 * coeffs[2] * x0 +
      //                   3 * coeffs[2] * x0 * x0;
      AD<double> df0 = 0.0;
      for (int i = 1; i < coeffs.size(); i++) {
        f0 += i * coeffs[i] * CppAD::pow(x0, i-1);
      }
      AD<double> calc_epsi0 = psi0 - CppAD::atan(df0);
      fg[1 + epsi_start + t] = epsi1 - ( calc_epsi0 + (v0 / Lf_) * delta0 * dt);
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::project(vector<double> state, vector<double> actuators, double dt){
  double x = state[0],
         y = state[1],
         psi = state[2],
         v = state[3],
         cte = state[4],
         epsi = state[5];
  double delta = actuators[0],
         a = actuators[1];

  x = x + v * cos(psi) * dt;
  y = y + v * sin(psi) * dt;
  psi = psi + (v / Lf) * prev_delta * dt;
  cte = cte + v * sin(epsi) * dt;
  epsi = epsi + (v / Lf) * delta * dt;
  v = v + a * dt;

  return {x, y, psi, v, cte, epsi};
}

void MPC::init(char filename[]) {
  char line[1000];
  FILE * fp;
  int print_flags = 0;

  std::cout << "opening file..." << std::endl;
  fp = fopen(filename, "r");
  if (fp == NULL){
    fprintf(stderr, "file pointer is null");
    fclose(fp);
  }

  std::cout << "reading file..." << std::endl;
  while(fgets(line, sizeof(line), fp)){
    std::cout << "read config line: " << line;
    switch(line[0]){
      case 'a':
        sscanf(line, "%c %d", &line[0], &N); break;
      case 'b':
        sscanf(line, "%c %lf", &line[0], &dt); break;
      case 'c':
        sscanf(line, "%c %lf", &line[0], &v_ref); break;
      case 'd':
        sscanf(line, "%c %lf", &line[0], &K_cte); break;
      case 'e':
        sscanf(line, "%c %lf", &line[0], &K_epsi); break;
      case 'f':
        sscanf(line, "%c %lf", &line[0], &K_v); break;
      case 'g':
        sscanf(line, "%c %lf", &line[0], &K_actuator_delta); break;
      case 'h':
        sscanf(line, "%c %lf", &line[0], &K_actuator_a); break;
      case 'i':
        sscanf(line, "%c %lf", &line[0], &K_like_before_delta); break;
      case 'j':
        sscanf(line, "%c %lf", &line[0], &K_like_before_a); break;
      case 'k':
        sscanf(line, "%c %lf", &line[0], &K_fast_turn); break;
      case 'l':
        sscanf(line, "%c %lf", &line[0], &K_fast_away); break;
      case 'm':
        sscanf(line, "%c %d", &line[0], &model_latency); break;
      case 'n':
        sscanf(line, "%c %d", &line[0], &sys_latency); break;
      case 'z':
        sscanf(line, "%c %d", &line[0], &print_flags); break;
    }
  }
  fclose(fp);
  std::cout << "Number of steps: " << N << std::endl;
  std::cout << "dt: " << dt << std::endl;
  std::cout << "time horizon: " << dt * N << std::endl;
  std::cout << "reference velocity: " << v_ref << std::endl;
  std::cout << "weight cte: " << K_cte << std::endl;
  std::cout << "weight epsi: " << K_epsi << std::endl;
  std::cout << "weight ref vel: " << K_v << std::endl;
  std::cout << "weight actuator delta: " << K_actuator_delta << std::endl;
  std::cout << "weight actuator acc: " << K_actuator_a << std::endl;
  std::cout << "weight dif delta: " << K_like_before_delta << std::endl;
  std::cout << "weight dif acc: " << K_like_before_a << std::endl;
  std::cout << "weight fast turn: " << K_fast_turn << std::endl;
  std::cout << "weight fast away: " << K_fast_away << std::endl;
  std::cout << "model latency [ms]: " << model_latency << std::endl;
  std::cout << "sys latency [ms]: " << sys_latency << std::endl;
  std::cout << "print flags: " << print_flags << std::endl;


  print_in = print_flags % 10; print_flags /= 10;
  print_out = print_flags % 10; print_flags /= 10;
  print_errors = print_flags % 10; print_flags /= 10;

  std::cout << "  print telemetry: " << print_in << std::endl;
  std::cout << "  print output: " << print_out << std::endl;
  std::cout << "  print errors: " << print_errors << std::endl;
  // compute indexes for new N
  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  v_start = psi_start + N;
  cte_start = v_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  a_start = delta_start + N - 1;
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
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

  cout << "before fg_eval" << endl;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
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
  vector<double> ret_vec;
  ret_vec.push_back(solution.x[delta_start]);
  ret_vec.push_back(solution.x[a_start]);

  for(int i=0; i<N; i++){
    ret_vec.push_back(solution.x[x_start+i]);
    ret_vec.push_back(solution.x[y_start+i]);
  }

  return ret_vec;
}
