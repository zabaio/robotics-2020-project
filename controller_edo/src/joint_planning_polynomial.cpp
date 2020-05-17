#include "joint_planning_polynomial.h"

#include <math.h>


void joint_planning_polynomial::init(double qi[], double qf[], double tf, int num_joint) {

    //salvataggio dati iniziali
    for(int i=0; i<num_joint; i++){
        this->qi[i]=qi[i];
        this->qf[i]=qf[i];
    }
    this->tf=tf;
    this->num_joint=num_joint;
    
    //calcola i coefficenti del polinomio
    for (int i = 0; i < num_joint; i++) {
            a0[i] = qi[i];
            a1[i] = 0.0;
            a2[i] = (3 * (qf[i] - qi[i])) / pow(tf, 2);
            a3[i] = (-2 * (qf[i] - qi[i])) / pow(tf, 3);
    }
}

void joint_planning_polynomial::plan(double time){
    for (int i = 0; i < num_joint; i++) {
        if (time<=tf)
        {
            q[i] = a0[i] + a1[i] * time + a2[i] * pow(time, 2) + a3[i] * pow(time, 3);
            v[i] = a1[i] + 2 * a2[i] * time + 3 * a3[i] * pow(time, 2);
            a[i] = 2 * a2[i] + 6 * a3[i] * time;
        }
        else
        {
            q[i] = a0[i] + a1[i] * tf + a2[i] * pow(tf, 2) + a3[i] * pow(tf, 3);
            v[i] = a1[i] + 2 * a2[i] * tf + 3 * a3[i] * pow(tf, 2);
            a[i] = 2 * a2[i] + 6 * a3[i] * tf;
        }
    }
}

void joint_planning_polynomial::getJointPosition(double q[]){
    for(int i=0; i<num_joint;i++){
        q[i]=this->q[i];
    }
}

void joint_planning_polynomial::getJointVelocity(double v[]){
    for(int i=0; i<num_joint;i++){
        v[i]=this->v[i];
    }
}

void joint_planning_polynomial::getJointAcceleration(double a[]){
    for(int i=0; i<num_joint;i++){
        a[i]=this->a[i];
    }
}
