#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle_or_not_input, vector<double> dp)
{
    // Initialize all variables初始化所有变量
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    pre_cte = 0;
    p_error = 0;
    i_error = 0;
    d_error = 0;

    p.resize(3);  // initialize a group of parameters
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;

    this->dp = dp;  // initialize a group of coefficients to optimize parameters

    err = 0.0;        // error value in twiddling
    best_err = 10e9;  // best error in twiddling
    index = 0;        // the index of parameter
    count = 0;         
    count_limit = 2000;   
    
    twiddle_or_not = twiddle_or_not_input;  // whether to perform twiddling
    if (twiddle_or_not)
    {
        p[index] += dp[index];
    }
}

// Update p_error, d_error, i_error and pre_cte
void PID::UpdateError(double cte)
{
    p_error = cte;
    d_error = cte - pre_cte;
    i_error += cte;
    pre_cte = cte;
}

// get new steering value and do parameters optimization
double PID::TotalError()
{
    err += pre_cte * pre_cte;
    double total_error = -(p_error * Kp + d_error * Kd + i_error * Ki);

    if (twiddle_or_not) twiddle();
    return total_error;
}


// perform twiddling
void PID::twiddle()
{
    count++; 

    if (count == count_limit/2)
    {  
        if (err < best_err)
        {
            // update errors
            best_err = err;
            dp[index] *= 1.1;
            count = 0;
            
            index = (index+1) % 3; 
            p[index] += dp[index];
        } else
        {
            p[index] -= 2*dp[index];
        }
        err = 0.0;
        i_error = 0; //reset i_error to 0.0

        // update parameters
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
    } else if (count == count_limit)
    {
        if (err < best_err)
        {
            best_err = err;
            dp[index] *= 1.1;
        } else
        {
            p[index] += dp[index];
            dp[index] *= 0.9;                
        }
        count = 0;
        index = (index+1) % 3;
        p[index] += dp[index];
        err = 0.0;
        i_error = 0.0;

        // update parameters
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
    }

}