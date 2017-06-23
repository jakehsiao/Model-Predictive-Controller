#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  double k_cte;
  double k_epsi;
  double k_v;
  double k_d;
  double k_a;
  double k_dd;
  double k_da;
  MPC();
  void Init(double k_cte, double k_epsi, double k_v, double k_d, double k_a, double k_dd, double k_da);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
