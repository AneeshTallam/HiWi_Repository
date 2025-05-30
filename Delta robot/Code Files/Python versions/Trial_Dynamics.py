import numpy as np
from InverseDynamics import InverseDynamics
import traceback

# Instantiate your class
idyn = InverseDynamics()

# Example parameter list (mock values)
par_list = {
    "Mass_EE": 1.2,
    "Radius_EE": 0.1,
    "Radius_base": 0.2,
    "Length_upper_arm": 0.4,
    "Length_forearm": 0.3,
    "mass_forearm": 1.0,
    "mass_upper_arm": 1.2,
    "inertia_motor": 0.01,
    "inertia_dominant_forearm": 0.005,
    "inertia_upper_arm": 0.02
}

# Initialize the model
model_name = "TestModel"
inp_no = 12
out_no = 33
hsim = 0.001
tsim = 1.0

success = idyn.init_model(model_name, par_list, inp_no, out_no, hsim, tsim)
print(f"Model initialized: {success}")

# Create test input: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, q1, q2, q3]
input_values = np.array([
    0.1, 0.2, 0.3,   # position
    4.0, 4.0, 4.0,   # velocity
    0.0, 0.0, -9.81, # acceleration
    0.1, 0.1, 0.1    # actuator angles (note: negated in exec_model)
])

output_values = np.zeros(out_no)  
hcnc = 0.001
tcnc = 0.0

# Execute model (to test that it runs)
try:
    idyn.exec_model(model_name, input_values, inp_no, output_values, out_no, hsim, tsim, hcnc, tcnc)
    print("exec_model ran successfully.")
except Exception as e:
    print("exec_model failed:", e)
    traceback.print_exc()
