/*
 *	@title	Discrete closed-loop controllers
 *	@author	ALEK GUEDEGBE (LAEH CEA GRENOBLE)
 *	@file	controllers.c <--> WPI=Workflow PSIM to IMPERIX
 */


#include "controllers.h"					                                    // Corresponding header file
#include <math.h>							                                    // Standard math library



/*
 * Routine to configure the PI controller and pre-compute the necessary constants.
 */

void ConfigPIController(PIController* me, float kp, float ti, float limup, float limlow, float Fsamp)
{
	// Set the controller parameters:
	me->kp = kp;
	me->ti = ti;
	me->limup = limup;
	me->limlow = limlow;
	me->Fsamp = Fsamp;

    me->A1 = kp / (ti*Fsamp);

	// Initialize the state quantities:

	me->e_prev = 0.0;
	me->ui_prev = 0.0;
}




/*
 * Routine to run the PI controller 'me' .
 */

float RunPIController(PIController* me, float error)
{
	float up;
	float ui;							                                    // Integral part of the output
	float u;


	up= me->kp * error;
	ui = ( me->A1 * error) + (me->ui_prev) ;

	u = up + ui ;

	//if ( (u>= me->limup) || (u <= me->limlow) )

	if ( (u>= me->limup) )
	{
		ui =  me->ui_prev;
	}
	else
	{
		ui = ( me->A1 * error) + (me->ui_prev) ;
	}



//Stores sample [K] in variables [K-1] in order to be used in the next sampling instant
	me->e_prev = error;
	me->ui_prev = ui;

//Limiter at the output of the compensator
	if (u>= me->limup) {
		u = me->limup;
	}

	if (u <= me->limlow) {
		u = me->limlow;
	}


	// Reset the integral when the outputs are inhibited (when the converter is blocked):
	//if(GetCoreState() != OPERATING)
	//	me->ui_prev = 0.0;					                                    // Avoid integrating when core has been disabled

	return u;
}



