#ifndef JOINT_PLANNING_TRAPEZOIDAL_H_
#define JOINT_PLANNING_TRAPEZOIDAL_H_

#define MAX_NUM_JOINT 6

// Classe per la pianificazione di una traiettoria ai giunti
class joint_planning_trapezoidal {

public:
		//inizializza la classe salvando le variabili in ingresso
		void init(double qi[], double qf[], double max_vel[], double max_acc[], int num_joint);

		//dato il timestamp calcola la traiettoria ai giunti
		void plan(double time);
		
		void getJointPosition(double q[]);
		void getJointVelocity(double v[]);
		void getJointAcceleration(double a[]);

private:
		double qi[MAX_NUM_JOINT]; //configurazione iniziale
		double qf[MAX_NUM_JOINT]; //configurazione finale

		double max_acc[MAX_NUM_JOINT]; //massima accelerazione angolare della traiettoria
		double max_vel[MAX_NUM_JOINT]; //massima velocità angolare della traiettoria

		double tc[MAX_NUM_JOINT]; //tempo di accelerazione/decelerazione
		double tf[MAX_NUM_JOINT]; //tempo finale

		bool triangolare[MAX_NUM_JOINT]; //vero nel caso di profilo di velocità triangolare

		double q[MAX_NUM_JOINT]; //configurazione attuale
		double v[MAX_NUM_JOINT]; //velocità attuale
		double a[MAX_NUM_JOINT]; //accelerazione attuale

		int num_joint; //numero di giunti da considerare

		//pianifica la traiettoria con il profilo trapezoidale
		void planTrapezoidal(double time);
		//pianifica la traiettoria con il polinomio
		void planPolynomial(double time);
};

#endif /* JOINT_PLANNING_TRAPEZOIDAL_H_ */
