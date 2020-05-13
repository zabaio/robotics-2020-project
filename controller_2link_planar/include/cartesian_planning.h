#ifndef CARTESIAN_PLANNING_H_
#define CARTESIAN_PLANNING_H_

#define MAX_NUM_COORDINATE 6

// Classe per la pianificazione di una traiettoria cartesiana
class cartesian_planning {

public:
		//inizializza la classe salvando le variabili in ingresso
		void init_line(double pi[], double pf[], double max_vel, double max_acc, int num_coordinate);

		//dato il timestamp calcola la traiettoria ai giunti
		void plan(double time);
		
		void getCartesianPosition(double p[]);
		void getCartesianVelocity(double pv[]);
		void getCartesianAcceleration(double pa[]);

private:
		double pi[MAX_NUM_COORDINATE]; //posizione iniziale
		double pf[MAX_NUM_COORDINATE]; //posizione finale

		double max_acc; //massima accelerazione dell'ascissa curvilinea
		double max_vel; //massima velocità dell'ascissa curvilinea

		double tc; //tempo di accelerazione/decelerazione
		double tf; //tempo finale
		double sf; //lunghezza traiettoria

		bool triangolare; //vero nel caso di profilo di velocità triangolare

		double s, sv, sa; //ascissa curvilinea attuale, velocità e accelerazione

		double p[MAX_NUM_COORDINATE]; //posizione attuale
		double pv[MAX_NUM_COORDINATE]; //velocità attuale
		double pa[MAX_NUM_COORDINATE]; //accelerazione attuale

		int num_coordinate; //numero di giunti da considerare
};

#endif /* CARTESIAN_PLANNING_H_ */
