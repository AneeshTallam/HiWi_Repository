import numpy as np

class InverseDynamics:
    def __init__(self):
        # Default values (can be overwritten in init_model)

        self.Mass_EE = None
        self.Radius_EE = None
        self.Radius_base = None
        self.Length_upper_arm = None
        self.Length_forearm = None
        self.mass_forearm = None
        self.mass_upper_arm = None
        self.inertia_motor = None
        self.inertia_dominant_forearm = None
        self.inertia_upper_arm = None

    def init_model(self, model_name, par_list, inp_no, out_no, hsim, tsim):
        
        #The method init_model() is called during the ramp up phase of ISG-virtuos
        """
        Initialize model parameters.
        
        Parameters:
        - model_name: str, name of the model
        - par_list: dict, block parameters
        - inp_no: int, number of input ports
        - out_no: int, number of output ports
        - hsim: float, simulation step size
        - tsim: float, total simulation time
        """
        self.model_name = model_name 
        self.inp_no = inp_no
        self.out_no = out_no
        self.hsim = hsim
        self.tsim = tsim
        # Reading parameters from the par_list dictionary and checking for any missing or wrong parameters

        try:
            self.Mass_EE = par_list["Mass_EE"]
            self.Radius_EE = par_list["Radius_EE"]
            self.Radius_base = par_list["Radius_base"]
            self.Length_upper_arm = par_list["Length_upper_arm"]
            self.Length_forearm = par_list["Length_forearm"]
            self.mass_forearm = par_list["mass_forearm"]
            self.mass_upper_arm = par_list["mass_upper_arm"]
            self.inertia_motor = par_list["inertia_motor"]
            self.inertia_dominant_forearm = par_list["inertia_dominant_forearm"]
            self.inertia_upper_arm = par_list["inertia_upper_arm"]
        except KeyError as e:
            print(f"{model_name}, Missing parameter: {e}")
            return False
        
        #verifying the number of output and input ports

        if inp_no != 12:
            print(f"{model_name}, Number of Input-Ports must be #12!")
            return False

        if out_no != 33:
            print(f"{model_name}, Number of Output-Ports must be #33!")
            return False

        return True
    
    def exec_model(self, model_name, input_values, inp_no, output_values, out_no, hsim, tsim, hcnc, tcnc):
        
        #The method exec_model() is updated every simulation-step
        """
        Parameters:
        - model_name: str, name of the model
        - input_values: float, Input port values
        - inp_no: int, number of input ports
        - output_values: float, Output port values
        - out_no: int, number of output ports
        - hsim: float, simulation step size
        - tsim: float, total simulation time
        - hcnc: float, output step size
        - tcnc: float, output time in seconds
        """
        
        #Extracting position, velocity and acceleration values

        pos_x = input_values[0]
        pos_y = input_values[1]
        pos_z = input_values[2]
        vel_x = input_values[3]
        vel_y = input_values[4]
        vel_z = input_values[5]
        acc_x = input_values[6]
        acc_y = input_values[7]
        acc_z = input_values[8]


        # IMPORTANT: actuator angles must be negative because inverse Kinematics from Virtuos and Asadi's dynamics
        q1 = -input[9]
        q2 = -input[10]
        q3 = -input[11]

        p_dot = np.array([vel_x, vel_y, vel_z])
        p_dot_dot = np.array([acc_x, acc_y, acc_z])
        p = np.array([pos_x, pos_y, pos_z])
        R_b = np.array([0, self.Radius_base, 0])
        R_a = np.array([0, self.Radius_EE, 0])

        #angles near the end effector
        Pi = 3.142592653589793
        phi_1 = -Pi / 6
        phi_2 = (-5 * Pi) / 6
        phi_3 = Pi / 2

        g = np.array([0, 0, -9.808]) # gravity vector

        # Rotation matrices around Z-axis using phi_1, phi_2, phi_3 
        rotation_R_1 = np.array([
            [np.cos(phi_1), np.sin(phi_1), 0],
            [-np.sin(phi_1), np.cos(phi_1), 0],
            [0, 0, 1]
        ])

        rotation_R_2 = np.array([
            [np.cos(phi_2), np.sin(phi_2), 0],
            [-np.sin(phi_2), np.cos(phi_2), 0],
            [0, 0, 1]
        ])

        rotation_R_3 = np.array([
            [np.cos(phi_3), np.sin(phi_3), 0],
            [-np.sin(phi_3), np.cos(phi_3), 0],
            [0, 0, 1]
        ])

        # Rotation matrices around X-axis using joint angles q1, q2, q3 
        rotation_O_1 = np.array([
            [1, 0, 0],
            [0, np.cos(q1), np.sin(q1)],
            [0, -np.sin(q1), np.cos(q1)]
        ])

        rotation_O_2 = np.array([
            [1, 0, 0],
            [0, np.cos(q2), np.sin(q2)],
            [0, -np.sin(q2), np.cos(q2)]
        ])

        rotation_O_3 = np.array([
            [1, 0, 0],
            [0, np.cos(q3), np.sin(q3)],
            [0, -np.sin(q3), np.cos(q3)]
        ])

        # Length vector of the upper arm
        L_vec = np.array([0, self.Length_upper_arm, 0])

        # Vector U_i (related to forearm)
        U_1 = p - rotation_R_1 @ ((R_b - R_a) + rotation_O_1 @ L_vec)
        U_2 = p - rotation_R_2 @ ((R_b - R_a) + rotation_O_2 @ L_vec)
        U_3 = p - rotation_R_3 @ ((R_b - R_a) + rotation_O_3 @ L_vec)


        # Skew-symmetric matrix related to the derivation of rotation around a general x axis
        S_1 = np.array([
            [0, 0, 0],
            [0, 0, 1],
            [0, -1, 0]
        ])

        # Torque/force vectors T_i
        T_1 = rotation_R_1 @ (S_1 @ (rotation_O_1 @ L_vec))
        T_2 = rotation_R_2 @ (S_1 @ (rotation_O_2 @ L_vec))
        T_3 = rotation_R_3 @ (S_1 @ (rotation_O_3 @ L_vec))
        
        # Compute dot products for the Jacobian auxiliary matrix
        J_aux = np.array([
            [np.dot(U_1, T_1), 0, 0],
            [0, np.dot(U_2, T_2), 0],
            [0, 0, np.dot(U_3, T_3)]
        ])

        # Checking the jacobian matrix for singularity
        try:
            inv_J_aux = np.linalg.inv(J_aux)
        except np.linalg.LinAlgError:
            print("Error: Jacobian is singular. Aborting simulation step.")
            raise RuntimeError("Jacobian is singular — inverse dynamics cannot proceed.")

        # Create U_matrix from column stacking of U vectors
        U_matrix = np.column_stack((U_1, U_2, U_3))

        # Compute the full Jacobian
        Jacobian = inv_J_aux @ U_matrix

        # Joint velocities
        q_dot = Jacobian @ p_dot

        # T_dot_i computation
        T_dot_1 = rotation_R_1 @ (S_1 @ (S_1 @ (rotation_O_1 @ L_vec))) * q_dot[0]
        T_dot_2 = rotation_R_2 @ (S_1 @ (S_1 @ (rotation_O_2 @ L_vec))) * q_dot[1]
        T_dot_3 = rotation_R_3 @ (S_1 @ (S_1 @ (rotation_O_3 @ L_vec))) * q_dot[2]

        # U_dot_i computation
        U_dot_1 = p_dot - T_1 * q_dot[0]
        U_dot_2 = p_dot - T_2 * q_dot[1]
        U_dot_3 = p_dot - T_3 * q_dot[2]

        # Extract rows of the Jacobian
        row_1_jacob = Jacobian[0, :]
        row_2_jacob = Jacobian[1, :]
        row_3_jacob = Jacobian[2, :]

        # Steps for calculating the derivative of jacobian
        # Step 1: vec_i = U_dot_i - row_i_jacob * (U_dot_i ⋅ T_i + U_i ⋅ T_dot_i)
        vec_1 = U_dot_1 - row_1_jacob * (np.dot(U_dot_1, T_1) + np.dot(U_1, T_dot_1))
        vec_2 = U_dot_2 - row_2_jacob * (np.dot(U_dot_2, T_2) + np.dot(U_2, T_dot_2))
        vec_3 = U_dot_3 - row_3_jacob * (np.dot(U_dot_3, T_3) + np.dot(U_3, T_dot_3))

        # Step 2: form col vectors from each coordinate
        col_1 = np.array([vec_1[0], vec_2[0], vec_3[0]])
        col_2 = np.array([vec_1[1], vec_2[1], vec_3[1]])
        col_3 = np.array([vec_1[2], vec_2[2], vec_3[2]])

        # Step 3: extract columns from inv_jacobian_aux matrix
        col_1_invjacob = inv_J_aux[:, 0]
        col_2_invjacob = inv_J_aux[:, 1]
        col_3_invjacob = inv_J_aux[:, 2]

        # Step 4: compute cross products for each pair
        jacobian_dot_1 = np.cross(col_1_invjacob, col_1)
        jacobian_dot_2 = np.cross(col_2_invjacob, col_2)
        jacobian_dot_3 = np.cross(col_3_invjacob, col_3)

        # Step 5: assemble Jacobian_dot as a 3x3 matrix
        Jacobian_dot = np.array([
            jacobian_dot_1,
            jacobian_dot_2,
            jacobian_dot_3
        ])

        # Steps to calculate the auxillary jacobian matrices
        matrix_aux_1 = np.array([
            [row_1_jacob[0], 0, 0],
            [row_1_jacob[1], 0, 0],
            [row_1_jacob[2], 0, 0]
        ])
        matrix_aux_2 = np.array([
            [row_2_jacob[0], 0, 0],
            [row_2_jacob[1], 0, 0],
            [row_2_jacob[2], 0, 0]
        ])
        matrix_aux_3 = np.array([
            [row_3_jacob[0], 0, 0],
            [row_3_jacob[1], 0, 0],
            [row_3_jacob[2], 0, 0]
        ])
        
        omega_L_1 = matrix_aux_1 @ p_dot
        omega_L_2 = matrix_aux_2 @ p_dot
        omega_L_3 = matrix_aux_3 @ p_dot

        r_L = 0.373  # proximal link Length ratio (upper arm)
        r_K = 0.5    # distal link ratio COM (forearm)

        Jacob_last_row_1 = np.array([
            [0, 0, row_1_jacob[0]],
            [0, 0, row_1_jacob[1]],
            [0, 0, row_1_jacob[2]]
        ])
        Jacob_last_row_2 = np.array([
            [0, 0, row_2_jacob[0]],
            [0, 0, row_2_jacob[1]],
            [0, 0, row_2_jacob[2]]
        ])
        Jacob_last_row_3 = np.array([
            [0, 0, row_3_jacob[0]],
            [0, 0, row_3_jacob[1]],
            [0, 0, row_3_jacob[2]]
        ])
        #Auxiallary Jacobians

        J_L_1 = Jacob_last_row_1 * r_L * self.Length_upper_arm
        J_L_2 = Jacob_last_row_2 * r_L * self.Length_upper_arm
        J_L_3 = Jacob_last_row_3 * r_L * self.Length_upper_arm

        
        J_K_1 = Jacob_last_row_1 * (1 - r_K) * self.Length_upper_arm + rotation_O_1.T @ rotation_R_1.T * r_K
        J_K_2 = Jacob_last_row_2 * (1 - r_K) * self.Length_upper_arm + rotation_O_2.T @ rotation_R_2.T * r_K
        J_K_3 = Jacob_last_row_3 * (1 - r_K) * self.Length_upper_arm + rotation_O_3.T @ rotation_R_3.T * r_K


        J_C_1 = Jacob_last_row_1 * self.Length_upper_arm
        J_C_2 = Jacob_last_row_2 * self.Length_upper_arm
        J_C_3 = Jacob_last_row_3 * self.Length_upper_arm

        #Solving for mass distributions in points A, C and forearm COM
        L = self.Length_forearm  # For code readability

        A_matrix = np.array([
            [1, r_K, r_K**2 * L**2],
            [1, r_K - 1, (r_K - 1)**2 * L**2],
            [1, 0, 0]
        ])

        b_vec = np.array([
        self.mass_forearm,
        0,
        self.inertia_dominant_forearm
        ])

        try:
            inv_A = np.linalg.inv(A_matrix)
        except np.linalg.LinAlgError:
            raise ValueError("Singular matrix: cannot solve for mass distribution.")

        x = inv_A @ b_vec
        m_c = x[0]  # mass at elbow
        m_A = x[1]  # mass at end-effector
        m_K = x[2]  # mass at COM of distal link (forearm)

        identity = np.identity(3)

        # Terms required for Mass matrix calculations 
        # sum_one: mass of distal links at COM position
        sum_one = (
            np.transpose(J_K_1) @ rotation_O_1.T @ rotation_R_1.T * m_K * r_K +
            np.transpose(J_K_2) @ rotation_O_2.T @ rotation_R_2.T * m_K * r_K +
            np.transpose(J_K_3) @ rotation_O_3.T @ rotation_R_3.T * m_K * r_K
        )
        # sum_two: combined mass contribution from elbow, upper arm, and distal links
        sum_two_1 = (
            np.transpose(J_C_1) * self.Length_upper_arm * m_c
            + np.transpose(J_L_1) * r_L * self.Length_upper_arm * self.mass_upper_arm
            + np.transpose(J_K_1) * self.Length_upper_arm * m_K * (1 - r_K)
        ) @ Jacob_last_row_1

        sum_two_2 = (
            np.transpose(J_C_2) * self.Length_upper_arm * m_c
            + np.transpose(J_L_2) * r_L * self.Length_upper_arm * self.mass_upper_arm
            + np.transpose(J_K_2) * self.Length_upper_arm * m_K * (1 - r_K)
        ) @ Jacob_last_row_2

        sum_two_3 = (
            np.transpose(J_C_3) * self.Length_upper_arm * m_c
            + np.transpose(J_L_3) * r_L * self.Length_upper_arm * self.mass_upper_arm
            + np.transpose(J_K_3) * self.Length_upper_arm * m_K * (1 - r_K)
        ) @ Jacob_last_row_3

        sum_two = sum_two_1 + sum_two_2 + sum_two_3

        # Mass matrix M(X,q) (Equation 40 of Asadi and Heydari)
        M_p_q = (
        np.transpose(Jacobian) @ Jacobian * (self.inertia_motor + self.inertia_upper_arm)
        + identity * (self.Mass_EE + 3 * m_c)
        + sum_one
        + sum_two
        )

        # Outer product for each row 
        outer_product_1 = np.array([
            row_1_jacob[0] * row_1_jacob[0], row_1_jacob[1] * row_1_jacob[0], row_1_jacob[2] * row_1_jacob[0],
            row_1_jacob[0] * row_1_jacob[1], row_1_jacob[1] * row_1_jacob[1], row_1_jacob[2] * row_1_jacob[1],
            row_1_jacob[0] * row_1_jacob[2], row_1_jacob[1] * row_1_jacob[2], row_1_jacob[2] * row_1_jacob[2]
        ])

        outer_product_2 = np.array([
            row_2_jacob[0] * row_2_jacob[0], row_2_jacob[1] * row_2_jacob[0], row_2_jacob[2] * row_2_jacob[0],
            row_2_jacob[0] * row_2_jacob[1], row_2_jacob[1] * row_2_jacob[1], row_2_jacob[2] * row_2_jacob[1],
            row_2_jacob[0] * row_2_jacob[2], row_2_jacob[1] * row_2_jacob[2], row_2_jacob[2] * row_2_jacob[2]
        ])

        outer_product_3 = np.array([
            row_3_jacob[0] * row_3_jacob[0], row_3_jacob[1] * row_3_jacob[0], row_3_jacob[2] * row_3_jacob[0],
            row_3_jacob[0] * row_3_jacob[1], row_3_jacob[1] * row_3_jacob[1], row_3_jacob[2] * row_3_jacob[1],
            row_3_jacob[0] * row_3_jacob[2], row_3_jacob[1] * row_3_jacob[2], row_3_jacob[2] * row_3_jacob[2]
        ])

        # Compute auxiliary terms
        aux_term_1 = -p_dot * outer_product_1
        aux_term_2 = -p_dot * outer_product_2
        aux_term_3 = -p_dot * outer_product_3

        # Compose sum_c_aux vectors
        sum_c_aux_1 = np.array([0, aux_term_1[0], jacobian_dot_1[0], 0, aux_term_1[1], jacobian_dot_2[0], 0, aux_term_1[2], jacobian_dot_3[0]])
        sum_c_aux_2 = np.array([0, aux_term_2[0], jacobian_dot_1[1], 0, aux_term_2[1], jacobian_dot_2[1], 0, aux_term_2[2], jacobian_dot_3[1]])
        sum_c_aux_3 = np.array([0, aux_term_3[0], jacobian_dot_1[2], 0, aux_term_3[1], jacobian_dot_2[2], 0, aux_term_3[2], jacobian_dot_3[2]])

        # summation term present in C_p_q_pdot_qdot
        sum_c = (
            (np.transpose(J_C_1) * self.Length_upper_arm * m_c +
            np.transpose(J_L_1) * self.Length_upper_arm * r_L * self.mass_upper_arm +
            np.transpose(J_K_1) * m_K * (1 - r_K) * self.Length_upper_arm) @ sum_c_aux_1 +

            (np.transpose(J_C_2) * self.Length_upper_arm * m_c +
            np.transpose(J_L_2) * self.Length_upper_arm * r_L * self.mass_upper_arm +
            np.transpose(J_K_2) * m_K * (1 - r_K) * self.Length_upper_arm) @ sum_c_aux_2 +

            (np.transpose(J_C_3) * self.Length_upper_arm * m_c +
            np.transpose(J_L_3) * self.Length_upper_arm * r_L * self.mass_upper_arm +
            np.transpose(J_K_3) * m_K * (1 - r_K) * self.Length_upper_arm) @ sum_c_aux_3
        )

        # Final matrix consisting of coriolis and certifugal forces (Equation 41 of Asadi and Heydari)
        C_p_q_pdot_qdot = (
            np.transpose(Jacobian) @ Jacobian_dot * (self.inertia_motor + self.inertia_upper_arm)
            + sum_c
        )

        # Summation term present in gravity vector
        sum_g = (
            (np.transpose(J_C_1) * m_c + np.transpose(J_L_1) * self.mass_upper_arm + np.transpose(J_K_1) * m_K)
            @ np.transpose(rotation_O_1) @ np.transpose(rotation_R_1) @ g
            +
            (np.transpose(J_C_2) * m_c + np.transpose(J_L_2) * self.mass_upper_arm + np.transpose(J_K_2) * m_K)
            @ np.transpose(rotation_O_2) @ np.transpose(rotation_R_2) @ g
            +
            (np.transpose(J_C_3) * m_c + np.transpose(J_L_3) * self.mass_upper_arm + np.transpose(J_K_3) * m_K)
            @ np.transpose(rotation_O_3) @ np.transpose(rotation_R_3) @ g
        )
        #Gravity vector (Equation 42 of Asadi and Heydari)
        G_p_q = -g * (self.Mass_EE + 3 * m_A) - sum_g

        try:
            inverse_M_p_q = np.linalg.inv(M_p_q)
        except:
            raise ValueError('The mass matrix is not invertible')  
        
        # Transpose and inverse of Jacobian transpose
        J_T = np.transpose(Jacobian)
        inv_J_t = np.linalg.inv(J_T)

        # Motion equation (Equation 39 of Asadi and Heydari)
        motion_equation = M_p_q @ p_dot_dot + C_p_q_pdot_qdot @ p_dot + G_p_q

        # Required torque computation
        tau = inv_J_t @ motion_equation

        # Assigning all the obtained values to output
        output_values[0] = tau[0]
        output_values[1] = tau[1]
        output_values[2] = tau[2]

        output_values[3] = inverse_M_p_q[0]
        output_values[4] = inverse_M_p_q[1]
        output_values[5] = inverse_M_p_q[2]
        output_values[6] = inverse_M_p_q[3]
        output_values[7] = inverse_M_p_q[4]
        output_values[8] = inverse_M_p_q[5]
        output_values[9] = inverse_M_p_q[6]
        output_values[10] = inverse_M_p_q[7]
        output_values[11] = inverse_M_p_q[8]

        output_values[12] = C_p_q_pdot_qdot[0]
        output_values[13] = C_p_q_pdot_qdot[1]
        output_values[14] = C_p_q_pdot_qdot[2]
        output_values[15] = C_p_q_pdot_qdot[3]
        output_values[16] = C_p_q_pdot_qdot[4]
        output_values[17] = C_p_q_pdot_qdot[5]
        output_values[18] = C_p_q_pdot_qdot[6]
        output_values[19] = C_p_q_pdot_qdot[7]
        output_values[20] = C_p_q_pdot_qdot[8]

        output_values[21] = G_p_q[0]
        output_values[22] = G_p_q[1]
        output_values[23] = G_p_q[2]

        output_values[24] = J_T[0]
        output_values[25] = J_T[1]
        output_values[26] = J_T[2]
        output_values[27] = J_T[3]
        output_values[28] = J_T[4]
        output_values[29] = J_T[5]
        output_values[30] = J_T[6]
        output_values[31] = J_T[7]
        output_values[32] = J_T[8]


                        










                












