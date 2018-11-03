#ifndef PID_H
#define PID_H

#include<vector>

using namespace std;

class PID
{
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double pre_cte;              // previous cte
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  vector<double> p;            // a group of parameters
  vector<double> dp;           // a group of coefficient to optimize parameters 
 
  bool twiddle_or_not;         // whether to perform twiddling
  double err, best_err;        // error coefficients for twiddling
  int index;                   // parameter index in twiddling
  int count;                     // cte input counter
  int count_limit;                 // counter bar for each cycle
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool tw_tf, vector<double> dp);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Parameters Optimization
   */ 
  void twiddle();
};

#endif /* PID_H */