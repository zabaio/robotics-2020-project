#include "PIDcontrol.h"

#include <stdexcept>


PIDcontrol::PIDcontrol(double Kc, double Ti, double Td, double N, double Ts, double uMin, double uMax)
{
    // Check parameter consistency
    if (Ts==0.0)
        throw std::invalid_argument("PID sampling time cannot be zero");
    if ((Kc==0.0) && (Ti==0.0) && (Td==0.0))
        throw std::invalid_argument("PID parameters cannot be all zero");
    if (uMin>uMax)
        throw std::invalid_argument("Lower control saturation cannot be greater then higher");

    // Initialise PID parameters
    this->Ts   = Ts;
    this->Kc   = Kc;
    this->Ti   = Ti;
    this->Td   = Td;
    this->N    = N;
    this->uMin = uMin;
    this->uMax = uMax;

    if (Ti==0.0)                        // No integral action
        this->a1 = 0.0;
    else
    {
        if (Kc==0.0)                    // Integral action without proportional action
            this->a1 = Ts/Ti;
        else                            // Integral action with proportional action
            this->a1 = Kc*Ts/Ti;
    }

    if (Td==0.0)                        // No derivative action
        this->b1 = this->b2 = 0.0;
    else
    {
        if (Kc==0.0)                    // Derivative action without proportional action
        {
            this->b1 = Td/(N*Ts+Td);
            this->b2 = N*this->b1;
        }
        else                            // Derivative action with proportional action
        {
            this->b1 = Td/(N*Ts+Td);
            this->b2 = Kc*N*this->b1;
        }
    }

    // Initialise PID state
    e_old = uI_old = uD_old = 0.0;
}

PIDcontrol::PIDcontrol(double Kc, double Ti, double Ts, double uMin, double uMax)
{
    // Check parameter consistency
    if (Ts==0.0)
        throw std::invalid_argument("PID sampling time cannot be zero");
    if ((Kc==0.0) && (Ti==0.0))
        throw std::invalid_argument("PID parameters cannot be all zero");
    if (uMin>uMax)
        throw std::invalid_argument("Lower control saturation cannot be greater then higher");

    // Initialise PID parameters
    this->Ts   = Ts;
    this->Kc   = Kc;
    this->Ti   = Ti;
    this->Td   = 0;
    this->N    = 0;
    this->uMin = uMin;
    this->uMax = uMax;

    if (Ti==0.0)                        // No integral action
        this->a1 = 0.0;
    else
    {
        if (Kc==0.0)                    // Integral action without proportional action
            this->a1 = Ts/Ti;
        else                            // Integral action with proportional action
            this->a1 = Kc*Ts/Ti;
    }

    // No derivative action
    this->b1 = this->b2 = 0.0;

    // Initialise PID state
    e_old = uI_old = uD_old = 0.0;
}

PIDcontrol::PIDcontrol(double Kc, double Ts, double uMin, double uMax)
{
    // Check parameter consistency
    if (Ts==0.0)
        throw std::invalid_argument("PID sampling time cannot be zero");
    if (Kc==0.0)
        throw std::invalid_argument("PID parameters cannot be all zero");
    if (uMin>uMax)
        throw std::invalid_argument("Lower control saturation cannot be greater then higher");

    // Initialise PID parameters
    this->Ts   = Ts;
    this->Kc   = Kc;
    this->Ti   = 0;
    this->Td   = 0;
    this->N    = 0;
    this->uMin = uMin;
    this->uMax = uMax;

    // No integral action
    this->a1 = 0.0;

    // No derivative action
    this->b1 = this->b2 = 0.0;

    // Initialise PID state
    e_old = uI_old = uD_old = 0.0;
}

PIDcontrol::~PIDcontrol()
{
    // Do nothing
}

void PIDcontrol::execute(double y, double ysp, double& u)
{
    // PID implemented equation
    // Kc * (1 + 1/Ti*1/s + s*Td/(1+s*Td/N)

    // Compute PID actions
    double e  = ysp-y;
    double uD = b1*uD_old + b2*(e-e_old);
    double uI = uI_old + a1*e;

    // Check windup condition
    double u_tmp = Kc*e+uI+uD;
    if (u_tmp>uMax)
    {
        u = uMax;
        uI_old = uI_old;
    }
    else if (u_tmp<uMin)
    {
        u = uMin;
        uI_old = uI_old;
    }
    else
    {
        u = u_tmp;
        uI_old = uI;
    }

    // Update PID states
    e_old  = e;
    uD_old = uD;
}

void PIDcontrol::resetState()
{
    // Reset PID state
    e_old = uI_old = uD_old = 0.0;
}
