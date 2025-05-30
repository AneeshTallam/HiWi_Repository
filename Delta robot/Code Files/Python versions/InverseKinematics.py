import numpy as np

class InverseKinematics:
    def __init__(self):
        self.Radius_EE = None
        self.Radius_base = None
        self.Length_upper_arm = None
        self.Length_forearm = None

    def init_model(self, model_name, par_list, inp_no, out_no, hsim, tsim):
        """
        Initializes the model during the ISG-virtuos ramp-up phase.
        
        Parameters:
        - model_name: Name of the model block.
        - par_list: Parameter list .
        - inp_no: Number of input ports.
        - out_no: Number of output ports.
        - hsim: Simulation step size in seconds.
        - tsim: Total simulation time in seconds.
        """
        # Reading parameters from the par_list dictionary and checking for any missing or wrong parameters

        try:
            self.Radius_base = par_list["Radius_base"]
            self.Length_upper_arm = par_list["Length_upper_arm"]
            self.Length_forearm = par_list["Length_forearm"]
            self.Radius_EE = par_list["Radius_EE"]
        except KeyError as e:
             raise ValueError(f"{model_name}, Missing parameter: {e}")

        #verifying the number of output and input ports
        if inp_no != 3:
            raise ValueError(f"{model_name}, Number of Input-Ports must be #3!")

        if out_no != 3:
            raise ValueError(f"{model_name}, Number of Output-Ports must be #3!")
    

    def exec_model(self, model_name, input_values, inp_no, output_values, out_no, hsim, tsim, hcnc, tcnc):
        # Ensure input is a numpy array for convenience
        input_values = np.asarray(input_values)

        # Extract positions from input
        pos_x = input_values[0]
        pos_y = input_values[1]
        pos_z = input_values[2]

        c = self.Radius_EE - self.Radius_base

        sqrt3 = np.sqrt(3)

        E1 = -(pos_x + 2 * c + sqrt3 * pos_y) * self.Length_upper_arm
        E2 = -(pos_x + 2 * c - sqrt3 * pos_y) * self.Length_upper_arm
        E3 = 2 * (pos_x - c) * self.Length_upper_arm

        F1 = 2 * pos_z * self.Length_forearm
        F2 = 2 * pos_z * self.Length_forearm
        F3 = 2 * pos_z * self.Length_forearm

        abs_pos = pos_x**2 + pos_y**2 + pos_z**2

        G1 = abs_pos + c * pos_x + c**2 + self.Length_upper_arm**2 + sqrt3 * c * pos_y - self.Length_forearm
        G2 = abs_pos + c * pos_x + c**2 + self.Length_upper_arm**2 - sqrt3 * c * pos_y - self.Length_forearm
        G3 = abs_pos - 2 * c * pos_x + c**2 + self.Length_upper_arm**2 - self.Length_forearm

        aux_T_1 = np.sqrt(E1**2 + F1**2 - G1**2)
        aux_T_2 = np.sqrt(E2**2 + F2**2 - G2**2)
        aux_T_3 = np.sqrt(E3**2 + F3**2 - G3**2)

        t_1_1 = (-F1 + aux_T_1) / (G1 - E1)
        t_1_2 = (-F1 - aux_T_1) / (G1 - E1)
        
        t_2_1 = (-F2 + aux_T_2) / (G2 - E2)
        t_2_2 = (-F2 - aux_T_2) / (G2 - E2)

        t_3_1 = (-F3 + aux_T_3) / (G3 - E3)
        t_3_2 = (-F3 - aux_T_3) / (G3 - E3)

        theta_1_1 = 2 * np.arctan(t_1_1)
        theta_1_2 = 2 * np.arctan(t_1_2)

        theta_2_1 = 2 * np.arctan(t_2_1)
        theta_2_2 = 2 * np.arctan(t_2_2)

        theta_3_1 = 2 * np.arctan(t_3_1)
        theta_3_2 = 2 * np.arctan(t_3_2)

        #Final actuator angles for the given set of X, Y, Z positions
        q1 = -theta_1_2
        q2 = -theta_2_2
        q3 = -theta_3_2

        # Assigning all the obtained values to output
        output_values[0] = q1
        output_values[1] = q2
        output_values[2] = q3

        print(output_values)


  

