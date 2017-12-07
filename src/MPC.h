#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  int sys_latency;
  int model_latency;
  double prev_delta;
  double prev_a;
  int print_in;
  int print_out;
  int print_errors;

  void fill_default(void);
  void init(char filename[]);
  void print_params(void);
  vector<double> project(vector<double> state, vector<double> actuators, double dt);


  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
