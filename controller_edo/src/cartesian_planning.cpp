#include "cartesian_planning.h"

#include <math.h>


void cartesian_planning::init_line(double pi[], double pf[], double max_vel, double max_acc, int num_coordinate) {
    //salvataggio dati iniziali
    for(int i=0; i<num_coordinate; i++){
        this->pi[i] = pi[i];
        this->pf[i] = pf[i];
    }

    this->max_acc = max_acc;
    this->max_vel = max_vel;
    this->num_coordinate = num_coordinate;

    //calcola la lunghezza della traiettoria
    sf = 0;
    for(int i=0; i<num_coordinate; i++){
        sf += pow(pf[i]-pi[i], 2.0);
    }
    sf = pow(sf, 0.5);

    //verifica se deve usare un profilo triangolare o trapezoidale e calcola i parametri della traiettoria trapezoidale dell'ascissa curvilinea
    if ( (pow(max_vel, 2.0) / max_acc) >= sf ){
        triangolare = true;

        tc = pow(sf / max_acc, 0.5);
        tf = 2*tc;
    }
    else{
        triangolare = false;

        tc = max_vel / max_acc;
        tf = (sf - max_acc * pow(tc, 2.0)) / max_vel + 2 * tc;
    }
}

void cartesian_planning::plan(double time){
    //calcola il valore attuale dell'ascissa curvilinea
    if (triangolare){
        if (time <= tc){
            s  = 0.5 * max_acc * pow(time, 2.0);
            sv = max_acc * time;
            sa = max_acc;
        } else if (time <= tf){
            s  = sf - 0.5 * max_acc * pow((tf - time), 2.0);
            sv = max_acc * (tf - time);
            sa = -max_acc;
        } else{
            s  = sf;
            sv = 0.0;
            sa = 0.0;                
        }
    }
    else{
        if (time <= tc){
            s  = 0.5 * max_acc * pow(time, 2.0);
            sv = max_acc * time;
            sa = max_acc;
        } else if (time <= (tf - tc)){
            s  = (0.5 * max_acc * pow(tc, 2.0)) + max_vel * (time - tc);
            sv = max_vel;
            sa = 0.0;
        } else if (time <= tf){
            s  = sf - 0.5 * max_acc * pow(tf - time, 2.0);
            sv = max_acc * (tf - time);
            sa = -max_acc;
        } else{
            s  = sf;
            sv = 0.0;
            sa = 0.0;                
        }
    }

    //calcola il valore attuale della posizione, velocità, accelerazione cartesiana
    for(int i=0; i<num_coordinate; i++){
        p[i]  = pi[i] + s / sf * (pf[i] - pi[i]);
        pv[i] = sv / sf * (pf[i] - pi[i]);
        pa[i] = sa / sf * (pf[i] - pi[i]);
    }    
}

void cartesian_planning::getCartesianPosition(double p[]){
    for(int i=0; i<num_coordinate;i++){
        p[i]=this->p[i];
    }
}

void cartesian_planning::getCartesianVelocity(double pv[]){
    for(int i=0; i<num_coordinate;i++){
        pv[i]=this->pv[i];
    }
}

void cartesian_planning::getCartesianAcceleration(double pa[]){
    for(int i=0; i<num_coordinate;i++){
        pa[i]=this->pa[i];
    }
}
