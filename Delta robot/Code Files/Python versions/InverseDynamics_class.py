
class InverseDynamics:
    def __init__(self):
        # Physical parameters (can be initialized later in init_model)
        self.Mass_EE = None
        self.Radius_EE = None
        self.Radius_base = None
        self.Length_upper_arm = None
        self.Length_forearm = None
        self.mass_forearm = None
        self.mass_upper_arm = None
        self.inertia_dominant_forearm = None
        self.inertia_motor = None
        self.inertia_upper_arm = None

    def init_model(self, param_dict, inp_no, out_no):
        # Read parameters from the dictionary
        try:
            self.Masse_EE = param_dict["Masse_EE"]
            self.Radius_EE = param_dict["Radius_EE"]
            self.Radius_base = param_dict["Radius_base"]
            self.Length_upper_arm = param_dict["Length_upper_arm"]
            self.Length_forearm = param_dict["Length_forearm"]
            self.mass_forearm = param_dict["mass_forearm"]
            self.mass_upper_arm = param_dict["mass_upper_arm"]
            self.inertia_motor = param_dict["inertia_motor"]
            self.inertia_dominant_forearm = param_dict["inertia_dominant_forearm"]
            self.inertia_upper_arm = param_dict["inertia_upper_arm"]
        except KeyError as e:
            print(f"Missing parameter: {e}")
            return False

        if inp_no != 12:
            print("Number of Input-Ports must be #12!")
            return False

        if out_no != 33:
            print("Number of Output-Ports must be #33!")
            return False

        return True


    def exec_model(self, input_values, hsim, tsim, hcnc, tcnc):
        """
        Executes one simulation step.

        :param input_values: List[float] - input port values
        :param hsim: float - simulation step size
        :param tsim: float - simulation time
        :param hcnc: float - output step size
        :param tcnc: float - output time
        :return: List[float] - output port values
        """

        # Example: compute dummy outputs based on inputs (placeholder logic)
        output_values = [0.0 for _ in range(len(input_values))]

        return output_values
