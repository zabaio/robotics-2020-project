#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

class PIDcontrol
{
    private:
        /* PID parameters */
        double Ts;                      // Sampling time
        double Kc, Ti, Td, N;           // PID parameters (proportional gain, integral/derivative time)
        double uMin, uMax;              // Output saturation values
        double a1, b1, b2;              // PID equation internal coefficients
    
        /* PID states */
        double e_old, uI_old, uD_old;   // PID internal states

    public:
        PIDcontrol(double Kc, double Ts, double uMin, double uMax);
        PIDcontrol(double Kc, double Ti, double Ts, double uMin, double uMax);
        PIDcontrol(double Kc, double Ti, double Td, double N, double Ts, double uMin, double uMax);
        ~PIDcontrol();

        void execute(double y, double ysp, double& u);
        void resetState();
};

#endif /* PID_CONTROL_H_ */