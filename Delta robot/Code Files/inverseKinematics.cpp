#include "InverseKinematics.h"

//The method initModel() is called during the ramp up phase of ISG-virtuos

V_INT32 InverseKinematics::initModel(

	V_CHAR8 *modelName, // Use block this for messages

	PARLIST *parList, // Contains the block parameters

	V_INT32 inpNo, // Number of input port

	V_INT32 outNo, // Number of input port

	V_REAL64 hsim, // Simulation step size in seconds

	V_REAL64 tsim) // Simulation time in seconds

{
	V_INT32 Status = DAMGD;

	Status = parList->ReadDbl("Radius_EE", 1, &this->Radius_EE);
	Status = parList->ReadDbl("Radius_base", 1, &this->Radius_base);
	Status = parList->ReadDbl("Length_upper_arm", 1, &this->Length_upper_arm);
	Status = parList->ReadDbl("Length_forearm", 1, &this->Length_forearm);


	if (Status != SUCCD)
	{
		scError(modelName, "Konnte den Parameter 'Multiplikator nicht lesen");
		return DAMGD;
	}

	if (inpNo != 3)

	{

		scError(modelName, "Number of Input-Ports must be #3!");

		return DAMGD; // Stop simulation

	}

	if (outNo != 3)

	{

		scError(modelName, "Number of Output-Ports must be #3!");

		return DAMGD; // Stop simulation

	}
	return SUCCD;

}

//The method exeModel() is updated every simulation-step

V_INT32 InverseKinematics::execModel(

	V_CHAR8 *modelName,

	V_REAL64 *input, // Input port values

	V_INT32 inpNo,

	V_REAL64 *output, // Output port values

	V_INT32 outNo,

	V_REAL64 hsim,

	V_REAL64 tsim,

	V_REAL64 hcnc, // Output step size

	V_REAL64 tcnc) // Output time in seconds

{
	V_REAL64 pos_x = GET(input, 1);
	V_REAL64 pos_y = GET(input, 2);
	V_REAL64 pos_z = GET(input, 3);

	V_REAL64 c = this->Radius_EE - this->Radius_base;
	V_REAL64 E1 = -(pos_x + 2 * c + COSMath::OSSqrt(3) * pos_y) * this->Length_upper_arm;
	V_REAL64 E2 = -(pos_x + 2 * c - COSMath::OSSqrt(3) * pos_y)* this->Length_upper_arm;
	V_REAL64 E3 = 2 * (pos_x - c)*this->Length_upper_arm;

	V_REAL64 F1 = 2 * pos_z * this->Length_forearm;
	V_REAL64 F2 = 2 * pos_z * this->Length_forearm;
	V_REAL64 F3 = 2 * pos_z * this->Length_forearm;

	V_REAL64 abs_pos = COSMath::OSPow(pos_x, 2) + COSMath::OSPow(pos_y, 2) + COSMath::OSPow(pos_z, 2);

	V_REAL64 G1 = abs_pos + c * pos_x + COSMath::OSPow(c, 2) + COSMath::OSPow(this->Length_upper_arm, 2) + COSMath::OSSqrt(3)* c* pos_y - this->Length_forearm;
	V_REAL64 G2 = abs_pos + c * pos_x + COSMath::OSPow(c, 2) + COSMath::OSPow(this->Length_upper_arm, 2) - COSMath::OSSqrt(3)* c* pos_y - this->Length_forearm;
	V_REAL64 G3 = abs_pos - 2* c * pos_x + COSMath::OSPow(c, 2) + COSMath::OSPow(this->Length_upper_arm, 2)  - this->Length_forearm;

	V_REAL64 aux_T_1 = COSMath::OSSqrt(COSMath::OSPow(E1, 2) + COSMath::OSPow(F1, 2) - COSMath::OSPow(G1, 2));
	V_REAL64 aux_T_2 = COSMath::OSSqrt(COSMath::OSPow(E2, 2) + COSMath::OSPow(F2, 2) - COSMath::OSPow(G2, 2));
	V_REAL64 aux_T_3 = COSMath::OSSqrt(COSMath::OSPow(E3, 2) + COSMath::OSPow(F3, 2) - COSMath::OSPow(G3, 2));

	V_REAL64 t_1_1 = (-F1 + aux_T_1) / (G1 - E1);
	V_REAL64 t_1_2 = (-F1 - aux_T_1) / (G1 - E1);

	V_REAL64 t_2_1 = (-F2 + aux_T_2) / (G2 - E2);
	V_REAL64 t_2_2 = (-F2 - aux_T_2) / (G2 - E2);

	V_REAL64 t_3_1 = (-F3 + aux_T_3) / (G3 - E3);
	V_REAL64 t_3_2 = (-F3 - aux_T_3) / (G3 - E3);

	V_REAL64 theta_1_1 = 2 * COSMath::OSAtan(t_1_1);
	V_REAL64 theta_1_2 = 2 * COSMath::OSAtan(t_1_2);

	V_REAL64 theta_2_1 = 2 * COSMath::OSAtan(t_2_1);
	V_REAL64 theta_2_2 = 2 * COSMath::OSAtan(t_2_2);

	V_REAL64 theta_3_1 = 2 * COSMath::OSAtan(t_3_1);
	V_REAL64 theta_3_2 = 2 * COSMath::OSAtan(t_3_2);

	V_REAL64 q1 = -theta_1_2;
	V_REAL64 q2 = -theta_2_2;
	V_REAL64 q3 = -theta_3_2;




	SET(output, 1, q1);
	SET(output, 2, q2);
	SET(output, 3, q3);


	return SUCCD;
}