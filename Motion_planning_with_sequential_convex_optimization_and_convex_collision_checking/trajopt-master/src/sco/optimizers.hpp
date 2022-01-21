#pragma once
#include <string>
#include "modeling.hpp"
#include <boost/function.hpp>
/*
 * Algorithms for non-convex, constrained optimization
 */

namespace sco {

using std::string;
using std::vector;


enum OptStatus {
  OPT_CONVERGED,
  OPT_SCO_ITERATION_LIMIT, // hit iteration limit before convergence
  OPT_PENALTY_ITERATION_LIMIT,
  OPT_FAILED,
  INVALID
};
static const char* OptStatus_strings[]  = {
  "CONVERGED",
  "SCO_ITERATION_LIMIT",
  "PENALTY_ITERATION_LIMIT",
  "FAILED",
  "INVALID"
};
inline string statusToString(OptStatus status) {
  return OptStatus_strings[status];
}


struct OptResults {
  DblVec x; // solution estimate
  OptStatus status;
  double total_cost;
  vector<double> cost_vals;
  DblVec cnt_viols;
  int n_func_evals, n_qp_solves;
  void clear() {
    x.clear();
    status = INVALID;
    cost_vals.clear();
    cnt_viols.clear();
    n_func_evals = 0;
    n_qp_solves = 0;
  }
  OptResults() {clear();}
};
std::ostream& operator<<(std::ostream& o, const OptResults& r);

class Optimizer {
  /*
   * Solves an optimization problem
   */
public:
  virtual OptStatus optimize() = 0;
  virtual ~Optimizer() {}
  virtual void setProblem(OptProbPtr prob) {prob_ = prob;}
  void initialize(const vector<double>& x);
  vector<double>& x() {return results_.x;}
  OptResults& results() {return results_;}

  typedef boost::function<void(OptProb*, DblVec&)> Callback;
  void addCallback(const Callback& f); // called before each iteration
protected:
  vector<Callback> callbacks_;
  void callCallbacks(DblVec& x);
  OptProbPtr prob_;
  OptResults results_;
};

class BasicTrustRegionSQP : public Optimizer {
  /*
   * Alternates between convexifying objectives and constraints and then solving convex subproblem
   * Uses a merit function to decide whether or not to accept the step
   * merit function = objective + merit_err_coeff * | constraint_error |
   * Note: sometimes the convexified objectives and constraints lead to an infeasible subproblem
   * In that case, you should turn them into penalties and solve that problem
   * (todo: implement penalty-based sqp that gracefully handles infeasible constraints)
   */
public:
  double improve_ratio_threshold_, // minimum ratio true_improve/approx_improve to accept step
         min_trust_box_size_, // if trust region gets any smaller, exit and report convergence
         min_approx_improve_, // if model improves less than this, exit and report convergence
         min_approx_improve_frac_, // if model improves less than this, exit and report convergence
         max_iter_,
         trust_shrink_ratio_, // if improvement is less than improve_ratio_threshold, shrink trust region by this ratio
         trust_expand_ratio_, // see above
         cnt_tolerance_, // after convergence of penalty subproblem, if constraint violation is less than this, we're done
         max_merit_coeff_increases_, // number of times that we jack up penalty coefficient
         merit_coeff_increase_ratio_, // ratio that we increate coeff each time
         max_time_ // not yet implemented
         ;
  double merit_error_coeff_, // initial penalty coefficient
         trust_box_size_ // current size of trust region (component-wise)
         ;

  BasicTrustRegionSQP();
  BasicTrustRegionSQP(OptProbPtr prob);
  void setProblem(OptProbPtr prob);
  OptStatus optimize();
protected:
  void adjustTrustRegion(double ratio);
  void setTrustBoxConstraints(const vector<double>& x);
  void initParameters();
  ModelPtr model_;
};


}
