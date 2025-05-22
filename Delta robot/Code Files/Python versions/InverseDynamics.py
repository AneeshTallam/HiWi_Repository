import numpy as np

class InverseDynamics:
    def __init__(self):
        # Default values (can be overwritten in init_model)

        self.Masse_EE = None
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
            self.Masse_EE = par_list["Masse_EE"]
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
    
    def exec_model(self, input_values, hsim, tsim, hcnc, tcnc):
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


