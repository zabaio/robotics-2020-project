#include "joint_planning_trapezoidal.h"

#include <math.h>


void joint_planning_trapezoidal::init(double qi[], double qf[], double max_vel[], double max_acc[], int num_joint) {
    //salvataggio dati iniziali
    for(int i=0; i<num_joint; i++){
        this->qi[i] = qi[i];
        this->qf[i] = qf[i];
        this->max_acc[i] = max_acc[i];
        this->max_vel[i] = max_vel[i];
    }

    this->num_joint = num_joint;

    //calcola i parametri della traiettoria trapezoidale
    for(int i=0;i<num_joint;i++){
        //verifica se deve usare un profilo triangolare o trapezoidale e calcola i parametri di conseguenza
        if ( (pow(max_vel[i], 2.0) / max_acc[i]) >= (qf[i]-qi[i]) ){
            triangolare[i] = true;

            tc[i] = pow((qf[i] - qi[i]) / max_acc[i], 0.5);
            tf[i] = 2*tc[i];
        }
        else{
            triangolare[i] = false;

            tc[i] = max_vel[i] / max_acc[i];
            tf[i] = (qf[i] - qi[i] - max_acc[i] * pow(tc[i], 2.0)) / max_vel[i] + 2 * tc[i];
        }
    }
}

void joint_planning_trapezoidal::plan(double time){
    for (int i=0; i<num_joint; i++){
        if (triangolare[i]){
            if (time <= tc[i]){
                q[i] = qi[i] + 0.5 * max_acc[i] * pow(time, 2.0);
                v[i] = max_acc[i] * time;
                a[i] = max_acc[i];
            } else if (time <= tf[i]){
                q[i] = qf[i] - 0.5 * max_acc[i] * pow((tf[i] - time), 2.0);
                v[i] = max_acc[i] * (tf[i] - time);
                a[i] = -max_acc[i];
            } else{
                q[i] = qf[i];
                v[i] = 0.0;
                a[i] = 0.0;                
            }
        }
        else{
            if (time <= tc[i]){
                q[i] = qi[i] + 0.5 * max_acc[i] * pow(time, 2.0);
                v[i] = max_acc[i] * time;
                a[i] = max_acc[i];
            } else if (time <= (tf[i] - tc[i])){
                q[i] = (qi[i] + 0.5 * max_acc[i] * pow(tc[i], 2.0)) + max_vel[i] * (time - tc[i]);
                v[i] = max_vel[i];
                a[i] = 0.0;
            } else if (time <= tf[i]){
                q[i] = qf[i] - 0.5 * max_acc[i] * pow(tf[i] - time, 2.0);
                v[i] = max_acc[i] * (tf[i] - time);
                a[i] = -max_acc[i];
            } else{
                q[i] = qf[i];
                v[i] = 0.0;
                a[i] = 0.0;                
            }
        }
    }
}

void joint_planning_trapezoidal::getJointPosition(double q[]){
    for(int i=0; i<num_joint;i++){
        q[i]=this->q[i];
    }
}

void joint_planning_trapezoidal::getJointVelocity(double v[]){
    for(int i=0; i<num_joint;i++){
        v[i]=this->v[i];
    }
}

void joint_planning_trapezoidal::getJointAcceleration(double a[]){
    for(int i=0; i<num_joint;i++){
        a[i]=this->a[i];
    }
}
