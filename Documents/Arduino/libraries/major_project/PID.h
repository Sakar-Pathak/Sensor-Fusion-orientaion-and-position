class PID{
public:
    PID(double p, double i, double d, double K);
    double compute(double setpoint, double input, double dt);

private:
    double kp, ki, kd, k, prevError, integral, derivative_est_previous;
    bool initialized = false;
};