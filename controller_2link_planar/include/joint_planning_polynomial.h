#ifndef JOINT_PLANNING_POLYNOMIAL_H_
#define JOINT_PLANNING_POLYNOMIAL_H_

#define MAX_NUM_JOINT 6

// Classe per la pianificazione di una traiettoria ai giunti
class joint_planning_polynomial {

public:
		//inizializza la classe salvando le variabili in ingresso
		void init(double qi[], double qf[], double tf, int num_joint);

		//dato il timestamp calcola la traiettoria ai giunti
		void plan(double time);
		
		void getJointPosition(double q[]);
		void getJointVelocity(double v[]);
		void getJointAcceleration(double a[]);

private:
		double a0[MAX_NUM_JOINT], //parametri del polinomio di terzo grado
			   a1[MAX_NUM_JOINT],
			   a2[MAX_NUM_JOINT],
			   a3[MAX_NUM_JOINT];

		double qi[MAX_NUM_JOINT]; //configurazione iniziale
		double qf[MAX_NUM_JOINT]; //configurazione finale

		double q[MAX_NUM_JOINT]; //configurazione attuale
		double v[MAX_NUM_JOINT]; //velocità attuale
		double a[MAX_NUM_JOINT]; //accelerazione attuale

		double tf; //tf per il calcolo con il polinomio
		int num_joint; //numero di giunti da considerare
};

#endif /* JOINT_PLANNING_POLYNOMIAL_H_ */
