
//ÚÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¿

/*! @file InverseKinematics.h

* Copyright (c) ISG STEP, Gropiusplatz 10, 70563 Stuttgart

* @author isg_hor_chr

* @brief My first own model block.

* Multiply input value by parameter “Gain_Factor”

* 1 input

* 1 output

* @revision -

* @date 27-Jul-2020

* @section HISTORY:

* @subsection CHANGE: <date> <description>

*/

//ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÙ

#ifndef _INVERSE_KINEMATICS_H_

#define _INVERSE_KINEMATICS_H_

#include "scSubModel.h"
#include "scVector.h"
#include "scSpatial.h"

class InverseKinematics : public scSubModel {

public:

	//The method initModel() is called during the ramp up phase of ISG-virtuos

	virtual V_INT32 initModel(

		V_CHAR8 *modelName, // Use block this for messages

		PARLIST *parList, // Contains the block parameters

		V_INT32 inpNo, // Number of input port

		V_INT32 outNo, // Number of input port

		V_REAL64 hsim, // Simulation step size in seconds

		V_REAL64 tsim); // Simulation time in seconds

		//The method exeModel() is updated every simulation-step

	virtual V_INT32 execModel(

		V_CHAR8 *modelName,

		V_REAL64 *input, // Input port values

		V_INT32 inpNo,

		V_REAL64 *output, // Output port values

		V_INT32 outNo,

		V_REAL64 hsim,

		V_REAL64 tsim,

		V_REAL64 hcnc, // Output step size

		V_REAL64 tcnc); // Output time in seconds

private:

	// Declare your own methods and variables here
	
	V_REAL64 Radius_EE;
	V_REAL64 Radius_base;
	V_REAL64 Length_upper_arm;
	V_REAL64 Length_forearm;



	// ...

};

#endif //_INVERSE_KINEMATICS_H_