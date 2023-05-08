#include "PID.h"

PID::PID(double p, double i, double d, double K){
    kp = p;
    ki = i;
    kd = d;
    k = K;
    prevError = 0;
    integral = 0;
}

double PID::compute(double setpoint, double input, double dt){
    double error = setpoint - input;
    integral += error * dt;
    double derivative = (error - prevError) / dt;

    if(initialized == false){
        derivative_est_previous = derivative;
        initialized = true;
    }

    double derivative_est = k*derivative + (1-k)*derivative_est_previous; // low-pass filter (optional
    double output = kp * error + ki * integral + kd * derivative;
    prevError = error;
    derivative_est_previous = derivative_est;
    return output;
}