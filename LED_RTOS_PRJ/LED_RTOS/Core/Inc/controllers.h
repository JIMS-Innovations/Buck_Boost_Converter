#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

#include <stdint.h>


/**
 * Library written by ALEK GUEDEGBE for Workflow PSIM to IMPERIX
 * PI controller library
 * Definition and operation according to the Workflow PSIM to IMPERIX
 */



typedef struct{
	float kp,ti;				// Gain and time constant of PI
	float limup;				// Upper saturation value of the output
	float limlow;				// Lower saturation value of the output
	float ui_prev;				// Previous value of the integral component
	float e_prev;				// Previous value of the error
    	float Fsamp;                // sampling frequency of the PI controller
	float A1 ;

} PIController;


/**
 * Routine to configure the PI controller 'me' and pre-compute the necessary constants.
 * @param *me		the PID pseudo-object to be configured
 * @param kp		proportional gain
 * @param ti		integral time constant
 * @param limup		upper saturation threshold of the output quantity
 * @param limlow	lower saturation threshold of the output quantity
 * @param Fsamp 	sampling (interrupt) frequency
 * @return void
 */

void ConfigPIController(PIController* me, float kp, float ti, float limup, float limlow, float Fsamp);



/**
 * Routines to run the pseudo-object 'me' depending of its actual nature (PI, PID, etc. controllers)
 * @param *me		the corresponding PID pseudo-object (parameters and state quantities)
 * @param error		the setpoint value minus the measured value
 * @return			the control variable for the measured quantity (output of the controller)
 */

float RunPIController(PIController* me, float error);


#endif /*CONTROLLERS_H_*/
