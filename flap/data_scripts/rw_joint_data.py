import numpy as np
import pandas as pd



'''
IMPORTANT NOTE, THIS DATA IS BASED OFF THE RIGHT WING IN THE HOME POSITION (T=0) 
DEFINED AT 0 DEGREES WHERE THERE IS 0 PITCH (WING ALIGNS WITH BODY) AND 
FLAP JOINT EXTENDS ALONG THE X-AXIS (ALONG BODY LINE)

TODO: WHEN BODY STARTS MOVING, NEED TO ACCOUNT FOR ITS ROTATION MATRIX
CURRENTLY ASSUMING 0 POSITION (THATS WHY 90 - ) IN CALCULATION, USING
IDENTITY MATRIX FOR BODY ROTATION MATRIX
'''

# Load CSV file
df = pd.read_csv("../data/aerodynamics_data.csv")

# Extract necessary columns
time = df['time'].values
rw_rot = df[['rw_rot_0_0', 'rw_rot_1_0', 'rw_rot_2_0', 'rw_rot_0_1', 'rw_rot_1_1', 'rw_rot_2_1', 'rw_rot_0_2', 'rw_rot_1_2', 'rw_rot_2_2']].values.reshape(-1,3, 3)
rw_joint_link_rot = df[['rw_joint_link_rot_0_0', 'rw_joint_link_rot_1_0', 'rw_joint_link_rot_2_0', 'rw_joint_link_rot_0_1', 'rw_joint_link_rot_1_1', 'rw_joint_link_rot_2_1', 'rw_joint_link_rot_0_2', 'rw_joint_link_rot_1_2', 'rw_joint_link_rot_2_2']].values.reshape(-1,3, 3)
index = np.where(time == .50019)[0][0]
# index = 1
# print("index: ", index)
# print(rw_rot[index])
# print("fresh rw_joint_link_rot matrix transposed:")
# print(rw_joint_link_rot[index])

# Transpose rw_joint_link_rot matrices
# print("rw_rot_matrix")
rw_rot_transposed = [matrix.T for matrix in rw_rot]
# print(rw_rot_transposed[index])
# Perform matrix multiplication
#tranpose means opposite based on how data is parsed. TODO: Rename variables to make it more clear
result_matrices = [np.matmul(rw_joint_link_m, rw_rot_m) for rw_joint_link_m, rw_rot_m in zip(rw_joint_link_rot, rw_rot_transposed)]
# print("result matrix")
# print(result_matrices[index])
print(np.arcsin(result_matrices[index][2][1]) * 180 / np.pi)

rw_joint_link_rot_nt = [matrix.T for matrix in rw_joint_link_rot]
# print("rw_joint_link_rot_nt: ")
# print(rw_joint_link_rot_nt[index])
# print("angle: ")
print((90 - np.arccos(rw_joint_link_rot_nt[index][1][0]) * 180 / np.pi))

passive_pitch_angles = [np.arcsin(matrix[2][1]) * 180 / np.pi for matrix in result_matrices]
flap_input_angles = [(90 - np.arccos(matrix[1][0]) * 180 / np.pi) for matrix in rw_joint_link_rot_nt]

results_df = pd.DataFrame({'time': time, 'passive_pitch_angles': passive_pitch_angles, 'flap_input_angles': flap_input_angles})

results_df.to_csv("../data/rw_angles.csv", index=False)

