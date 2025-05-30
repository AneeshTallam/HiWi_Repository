from InverseKinematics import InverseKinematics
import numpy as np
import traceback
trial_class = InverseKinematics()

par_list = {
    "Radius_EE": 0.1,
    "Radius_base": 0.2,
    "Length_upper_arm": 0.4,
    "Length_forearm": 0.3
}

model_name = "TestModel"
inp_no = 3
out_no = 3
hsim = 0.001
tsim = 1.0

try:
    trial_class.init_model(model_name, par_list, inp_no, out_no, hsim, tsim)
    print('Init_model method ran successfully.')
except Exception as e:
    print("init_model failed:", e)
    traceback.print_exc()



input_pos = [0.1, 0.2, 0.3]
output_values = np.zeros(out_no)  
hcnc = 0.001
tcnc = 0.0

try:
    trial_class.exec_model(model_name, input_pos, inp_no, output_values, out_no, hsim, tsim, hcnc, tcnc)
    print("exec_model ran successfully.")
except Exception as e:
    print("exec_model failed:", e)
    traceback.print_exc()
