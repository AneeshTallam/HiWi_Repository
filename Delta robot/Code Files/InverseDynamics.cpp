#include "InverseDynamics.h"
#include "scVector.h"
#include "scSpatial.h"

//The method initModel() is called during the ramp up phase of ISG-virtuos

V_INT32 InverseDynamics::initModel(

	V_CHAR8 *modelName, // Use block this for messages

	PARLIST *parList, // Contains the block parameters

	V_INT32 inpNo, // Number of input port

	V_INT32 outNo, // Number of input port

	V_REAL64 hsim, // Simulation step size in seconds

	V_REAL64 tsim) // Simulation time in seconds

{
	V_INT32 Status = DAMGD;
	Status = parList->ReadDbl("Masse_EE", 1, &this->Masse_EE);
	Status = parList->ReadDbl("Radius_EE", 1, &this->Radius_EE);
	Status = parList->ReadDbl("Radius_base", 1, &this->Radius_base);
	Status = parList->ReadDbl("Length_upper_arm", 1, &this->Length_upper_arm);
	Status = parList->ReadDbl("Length_forearm", 1, &this->Length_forearm);
	Status = parList->ReadDbl("mass_forearm", 1, &this->mass_forearm);
	Status = parList->ReadDbl("mass_upper_arm", 1, &this->mass_upper_arm);
	Status = parList->ReadDbl("inertia_motor", 1, &this->inertia_motor);
	Status = parList->ReadDbl("inertia_dominant_forearm", 1, &this->inertia_dominant_forearm);
	Status = parList->ReadDbl("inertia_upper_arm", 1, &this->inertia_upper_arm);


	if (Status != SUCCD)
	{
		scError(modelName, "Konnte den Parameter 'Multiplikator nicht lesen");
		return DAMGD;
	}

	if (inpNo != 12)

	{

		scError(modelName, "Number of Input-Ports must be #12!");

		return DAMGD; // Stop simulation

	}

	if (outNo != 33)

	{

		scError(modelName, "Number of Output-Ports must be #33!");

		return DAMGD; // Stop simulation

	}
	return SUCCD;

}

//The method exeModel() is updated every simulation-step

V_INT32 InverseDynamics::execModel(

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
	V_REAL64 vel_x = GET(input, 4);
	V_REAL64 vel_y = GET(input, 5);
	V_REAL64 vel_z = GET(input, 6);
	V_REAL64 acc_x = GET(input, 7);
	V_REAL64 acc_y = GET(input, 8);
	V_REAL64 acc_z = GET(input, 9);

	// IMPORTANT: angles must be negative because inverse Kinematics from Virtuos and Asadi`s dynamics
	V_REAL64 q1 = -GET(input, 10);
	V_REAL64 q2 = -GET(input, 11);
	V_REAL64 q3 = -GET(input, 12);

	scVector p_dot = {vel_x, vel_y, vel_z };
	scVector p_dot_dot = {acc_x, acc_y, acc_z };
	scVector p = { pos_x, pos_y, pos_z };
	scVector R_b = { 0, this->Radius_base,0 };
	scVector R_p = { 0, this->Radius_EE,0 };

	V_REAL64 Pi = 3.142592653589793;
	V_REAL64 phi_1 = -Pi / 6;
	V_REAL64 phi_2 = (-5 * Pi) / 6;
	V_REAL64 phi_3 = Pi / 2;

	scVector g = { 0, 0, -9.808 };

	scSpatial rotation_R_1 = { COSMath::OSCos(phi_1), COSMath::OSSin(phi_1), 0, -COSMath::OSSin(phi_1), COSMath::OSCos(phi_1), 0, 0, 0, 1 };
	scSpatial rotation_R_2 = { COSMath::OSCos(phi_2), COSMath::OSSin(phi_2), 0, -COSMath::OSSin(phi_2), COSMath::OSCos(phi_2), 0, 0, 0, 1 };
	scSpatial rotation_R_3 = { COSMath::OSCos(phi_3), COSMath::OSSin(phi_3), 0, -COSMath::OSSin(phi_3), COSMath::OSCos(phi_3), 0, 0, 0, 1 };

	scSpatial rotation_O_1 = { 1, 0, 0, 0, COSMath::OSCos(q1), COSMath::OSSin(q1), 0, -COSMath::OSSin(q1), COSMath::OSCos(q1) };
	scSpatial rotation_O_2 = { 1, 0, 0, 0, COSMath::OSCos(q2), COSMath::OSSin(q2), 0, -COSMath::OSSin(q2), COSMath::OSCos(q2) };
	scSpatial rotation_O_3 = { 1, 0, 0, 0, COSMath::OSCos(q3), COSMath::OSSin(q3), 0, -COSMath::OSSin(q3), COSMath::OSCos(q3) };

	scVector L_vec = { 0, this->Length_upper_arm, 0 };

	scVector U_1 = p - rotation_R_1 * ((R_b - R_p) + rotation_O_1 * L_vec);
	scVector U_2 = p - rotation_R_2 * ((R_b - R_p) + rotation_O_2 * L_vec);
	scVector U_3 = p - rotation_R_3 * ((R_b - R_p) + rotation_O_3 * L_vec);

	scSpatial S_1 = { 0, 0, 0, 0, 0, 1, 0, -1,0 };

	scVector T_1 = rotation_R_1 * S_1 * rotation_O_1* L_vec;
	scVector T_2 = rotation_R_2 * S_1 * rotation_O_2* L_vec;
	scVector T_3 = rotation_R_3 * S_1 * rotation_O_3* L_vec;

	scSpatial Jacobian_aux = { U_1*T_1, 0, 0, 0, U_2*T_2, 0, 0, 0, U_3*T_3 };
	V_INT32 isSingular;
	scSpatial inv_jacobian_aux = Jacobian_aux.Inverse(isSingular);
	scSpatial U_matrix = { U_1[0], U_2[0], U_3[0], U_1[1], U_2[1], U_3[1], U_1[2], U_2[2], U_3[2] };
	scSpatial Jacobian = inv_jacobian_aux * U_matrix;
	scVector q_dot = Jacobian * p_dot;

	scVector T_dot_1 = rotation_R_1 * S_1* S_1 * rotation_O_1 * L_vec * q_dot[0];
	scVector T_dot_2 = rotation_R_2 * S_1* S_1 * rotation_O_2 * L_vec * q_dot[1];
	scVector T_dot_3 = rotation_R_3 * S_1* S_1 * rotation_O_3 * L_vec * q_dot[2];

	scVector U_dot_1 = p_dot - T_1 * q_dot[0];
	scVector U_dot_2 = p_dot - T_2 * q_dot[1];
	scVector U_dot_3 = p_dot - T_3 * q_dot[2];

		
	scVector row_1_jacob = { Jacobian[0], Jacobian[3], Jacobian[6] };
	scVector row_2_jacob = { Jacobian[1], Jacobian[4], Jacobian[7] };
	scVector row_3_jacob = { Jacobian[2], Jacobian[5], Jacobian[8] };

		
		

	scVector vec_1 = U_dot_1 - row_1_jacob * (U_dot_1 * T_1 + U_1 * T_dot_1);
	scVector vec_2 = U_dot_2 - row_2_jacob * (U_dot_2 * T_2 + U_2 * T_dot_2);
	scVector vec_3 = U_dot_3 - row_3_jacob * (U_dot_3 * T_3 + U_3 * T_dot_3);
	scVector col_1 = { vec_1[0],vec_2[0], vec_3[0] };
	scVector col_2 = { vec_1[1],vec_2[1], vec_3[1] };
	scVector col_3 = { vec_1[2],vec_2[2], vec_3[2] };

	scVector col_1_invjacob = { inv_jacobian_aux[0], inv_jacobian_aux[1], inv_jacobian_aux[2] };
	scVector col_2_invjacob = { inv_jacobian_aux[3], inv_jacobian_aux[4], inv_jacobian_aux[5] };
	scVector col_3_invjacob = { inv_jacobian_aux[6], inv_jacobian_aux[7], inv_jacobian_aux[8] };

	scVector jacobian_dot_1 = col_1_invjacob.Cross(col_1);
	scVector jacobian_dot_2 = col_2_invjacob.Cross(col_2);
	scVector jacobian_dot_3 = col_3_invjacob.Cross(col_3);
	scSpatial Jacobian_dot = { jacobian_dot_1[0], jacobian_dot_1[1], jacobian_dot_1[2], jacobian_dot_2[0], jacobian_dot_2[1], jacobian_dot_2[2],
	jacobian_dot_3[0], jacobian_dot_3[1], jacobian_dot_3[2] };

	scSpatial matrix_aux_1 = { row_1_jacob[0], 0, 0, row_1_jacob[1], 0, 0, row_1_jacob[2], 0, 0 };
	scSpatial matrix_aux_2 = { row_2_jacob[0], 0, 0, row_2_jacob[1], 0, 0, row_2_jacob[2], 0, 0 };
	scSpatial matrix_aux_3 = { row_3_jacob[0], 0, 0, row_3_jacob[1], 0, 0, row_3_jacob[2], 0, 0 };

	scVector omega_L_1 = matrix_aux_1 * p_dot;
	scVector omega_L_2 = matrix_aux_2 * p_dot;
	scVector omega_L_3 = matrix_aux_3 * p_dot;

	V_REAL64 r_L = 0.373; // proximal link lenght ratio
	V_REAL64 r_K = 0.5; // distal link ratio COM


	//auxiliary Jacobians

	scSpatial Jacob_last_row_1 = { 0, 0, row_1_jacob[0], 0, 0 , row_1_jacob[1], 0, 0, row_1_jacob[2] };
	scSpatial Jacob_last_row_2 = { 0, 0, row_2_jacob[0], 0, 0 , row_2_jacob[1], 0, 0, row_2_jacob[2] };
	scSpatial Jacob_last_row_3 = { 0, 0, row_3_jacob[0], 0, 0 , row_3_jacob[1], 0, 0, row_3_jacob[2] };

	scSpatial J_L_1 = Jacob_last_row_1 * r_L * this->Length_upper_arm;
	scSpatial J_L_2 = Jacob_last_row_2 * r_L * this->Length_upper_arm;
	scSpatial J_L_3 = Jacob_last_row_3 * r_L * this->Length_upper_arm;

	scSpatial J_K_1 = Jacob_last_row_1 * (1 - r_K) * this->Length_upper_arm +  rotation_O_1.Transpose() * rotation_R_1.Transpose() * r_K;
	scSpatial J_K_2 = Jacob_last_row_2 * (1 - r_K) * this->Length_upper_arm + rotation_O_2.Transpose() * rotation_R_2.Transpose() * r_K;
	scSpatial J_K_3 = Jacob_last_row_3 * (1 - r_K) * this->Length_upper_arm + rotation_O_3.Transpose() * rotation_R_3.Transpose() * r_K;
		
	scSpatial J_C_1 = Jacob_last_row_1 * this->Length_upper_arm;
	scSpatial J_C_2 = Jacob_last_row_2 * this->Length_upper_arm;
	scSpatial J_C_3 = Jacob_last_row_3 * this->Length_upper_arm;

	//mass distributions
	scSpatial A_matrix = { 1, r_K, COSMath::OSPow(r_K,2) * COSMath::OSPow(this->Length_forearm,2), 1, r_K - 1, COSMath::OSPow((r_K - 1),2) *COSMath::OSPow(this->Length_forearm,2), 1, 0, 0 };
	scVector b_vec = { this->mass_forearm, 0, this->inertia_dominant_forearm};
	scSpatial inv_A = A_matrix.Inverse(isSingular);
		
	scVector x = inv_A * b_vec;
	
	V_REAL64 m_c = x[0]; // mass distributed to elbow
	V_REAL64 m_A = x[1]; //mass distributed to endeffector
	V_REAL64 m_K = x[2]; // mass distributed to COM distal link

	scSpatial identity = { 1, 0, 0,0,1,0, 0, 0,1 };

	// massMatrix
	scSpatial sum_one = J_K_1.Transpose() * rotation_O_1.Transpose() * rotation_R_1.Transpose() * m_K * r_K + J_K_2.Transpose()* rotation_O_2.Transpose()* rotation_R_2.Transpose()*m_K*r_K
		+ J_K_3.Transpose()* rotation_O_3.Transpose()* rotation_R_3.Transpose()*m_K * r_K;
	scSpatial sum_two_1 = (J_C_1.Transpose()* this->Length_upper_arm*m_c + J_L_1.Transpose() * r_L*this->Length_upper_arm* this->mass_upper_arm + J_K_1.Transpose() * this->Length_upper_arm*m_K*(1 - r_K))*Jacob_last_row_1;
	scSpatial sum_two_2 = (J_C_2.Transpose()* this->Length_upper_arm*m_c + J_L_2.Transpose() * r_L*this->Length_upper_arm* this->mass_upper_arm + J_K_2.Transpose() * this->Length_upper_arm*m_K*(1 - r_K))*Jacob_last_row_2;
	scSpatial sum_two_3 = (J_C_3.Transpose()* this->Length_upper_arm*m_c + J_L_3.Transpose() * r_L*this->Length_upper_arm* this->mass_upper_arm + J_K_3.Transpose() * this->Length_upper_arm*m_K*(1 - r_K))*Jacob_last_row_3;
	scSpatial sum_two = sum_two_1 + sum_two_2 + sum_two_3;
	scSpatial M_p_q = (Jacobian.Transpose()* Jacobian)*(this->inertia_motor + this->inertia_upper_arm) + identity * (this->Masse_EE + 3 * m_c) + sum_one + sum_two;


	//outer product
	scSpatial outer_product_1 = {row_1_jacob[0] * row_1_jacob[0], row_1_jacob[1] * row_1_jacob[0], row_1_jacob[2] * row_1_jacob[0],row_1_jacob[0] * row_1_jacob[1],row_1_jacob[1] * row_1_jacob[1],row_1_jacob[2] * row_1_jacob[1],
		row_1_jacob[0] * row_1_jacob[2],row_1_jacob[1] * row_1_jacob[2],row_1_jacob[2] * row_1_jacob[2] };
	scSpatial outer_product_2 = { row_2_jacob[0] * row_2_jacob[0], row_2_jacob[1] * row_2_jacob[0], row_2_jacob[2] * row_2_jacob[0],row_2_jacob[0] * row_2_jacob[1],row_2_jacob[1] * row_2_jacob[1],row_2_jacob[2] * row_2_jacob[1],
		row_2_jacob[0] * row_2_jacob[2],row_2_jacob[1] * row_2_jacob[2],row_2_jacob[2] * row_2_jacob[2] };
	scSpatial outer_product_3 = { row_3_jacob[0] * row_3_jacob[0], row_3_jacob[1] * row_3_jacob[0], row_3_jacob[2] * row_3_jacob[0],row_3_jacob[0] * row_3_jacob[1],row_3_jacob[1] * row_3_jacob[1],row_3_jacob[2] * row_3_jacob[1],
		row_3_jacob[0] * row_3_jacob[2],row_3_jacob[1] * row_3_jacob[2],row_3_jacob[2] * row_3_jacob[2] };


	scVector aux_term_1 = -p_dot * outer_product_1;
	scVector aux_term_2 = -p_dot * outer_product_2;
	scVector aux_term_3 = -p_dot * outer_product_3;
	scSpatial sum_c_aux_1 = { 0, aux_term_1[0], jacobian_dot_1[0], 0,aux_term_1[1], jacobian_dot_2[0], 0, aux_term_1[2], jacobian_dot_3[0]};
	scSpatial sum_c_aux_2 = { 0, aux_term_2[0], jacobian_dot_1[1], 0,aux_term_2[1], jacobian_dot_2[1], 0, aux_term_2[2], jacobian_dot_3[1]};
	scSpatial sum_c_aux_3 = { 0, aux_term_3[0], jacobian_dot_1[2], 0,aux_term_3[1], jacobian_dot_2[2], 0, aux_term_3[2], jacobian_dot_3[2]};
	scSpatial sum_c = (J_C_1.Transpose()* this->Length_upper_arm*m_c + J_L_1.Transpose() * this->Length_upper_arm* r_L*this->mass_upper_arm + J_K_1.Transpose() * m_K * (1 - r_K)* this->Length_upper_arm) * sum_c_aux_1
		+ (J_C_2.Transpose()* this->Length_upper_arm*m_c + J_L_2.Transpose() * this->Length_upper_arm* r_L*this->mass_upper_arm + J_K_2.Transpose() * m_K * (1 - r_K)* this->Length_upper_arm) * sum_c_aux_2
		+ (J_C_3.Transpose()* this->Length_upper_arm*m_c + J_L_3.Transpose() * this->Length_upper_arm* r_L*this->mass_upper_arm + J_K_3.Transpose() * m_K * (1 - r_K)* this->Length_upper_arm) * sum_c_aux_3;

	scSpatial C_p_q_pdot_qdot =  Jacobian.Transpose() * Jacobian_dot * (this->inertia_motor + this->inertia_upper_arm) + sum_c;
	scVector sum_g = (J_C_1.Transpose()* m_c+ J_L_1.Transpose()*this->mass_upper_arm + J_K_1.Transpose()*m_K)*rotation_O_1.Transpose() * rotation_R_1.Transpose() * g
		+ (J_C_2.Transpose()* m_c + J_L_2.Transpose()*this->mass_upper_arm + J_K_2.Transpose()*m_K)*rotation_O_2.Transpose() * rotation_R_2.Transpose() * g
		+(J_C_3.Transpose()* m_c + J_L_3.Transpose()*this->mass_upper_arm + J_K_3.Transpose()*m_K)*rotation_O_3.Transpose() * rotation_R_3.Transpose() * g;

	scVector G_p_q = - g * (this->Masse_EE + 3 * m_A) - sum_g;
	scSpatial inverse_M_p_q = M_p_q.Inverse(isSingular);
	scSpatial J_T = Jacobian.Transpose();
	scSpatial inv_J_t = J_T.Inverse(isSingular);
	scVector motion_equation = M_p_q * p_dot_dot + C_p_q_pdot_qdot * p_dot + G_p_q;
	scVector tau = inv_J_t * motion_equation;
	SET(output, 1, tau[0]);
	SET(output, 2, tau[1]);
	SET(output, 3, tau[2]);
	SET(output, 4, inverse_M_p_q[0]);
	SET(output, 5, inverse_M_p_q[1]);
	SET(output, 6, inverse_M_p_q[2]);
	SET(output, 7, inverse_M_p_q[3]);
	SET(output, 8, inverse_M_p_q[4]);
	SET(output, 9, inverse_M_p_q[5]);
	SET(output, 10, inverse_M_p_q[6]);
	SET(output, 11, inverse_M_p_q[7]);
	SET(output, 12, inverse_M_p_q[8]);
	SET(output, 13, C_p_q_pdot_qdot[0]);
	SET(output, 14, C_p_q_pdot_qdot[1]);
	SET(output, 15, C_p_q_pdot_qdot[2]);
	SET(output, 16, C_p_q_pdot_qdot[3]);
	SET(output, 17, C_p_q_pdot_qdot[4]);
	SET(output, 18, C_p_q_pdot_qdot[5]);
	SET(output, 19, C_p_q_pdot_qdot[6]);
	SET(output, 20, C_p_q_pdot_qdot[7]);
	SET(output, 21, C_p_q_pdot_qdot[8]);
	SET(output, 22, G_p_q[0]);
	SET(output, 23, G_p_q[1]);
	SET(output, 24, G_p_q[2]);
	SET(output, 25, J_T[0]);
	SET(output, 26, J_T[1]);
	SET(output, 27, J_T[2]);
	SET(output, 28, J_T[3]);
	SET(output, 29, J_T[4]);
	SET(output, 30, J_T[5]);
	SET(output, 31, J_T[6]);
	SET(output, 32, J_T[7]);
	SET(output, 33, J_T[8]);

	return SUCCD;



}
