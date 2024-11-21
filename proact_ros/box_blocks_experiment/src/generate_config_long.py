import os
import yaml

# Locations
x_near = -0.058
x_far = 0.058
y_lateral = -0.2083
y_medial = -0.092

variable_names = ["red_block1_x", "red_block1_y", "red_block2_x", "red_block2_y", "blue_block1_x", "blue_block1_y", "blue_block2_x", "blue_block2_y"]

# main types of configs: A (diagonal), B (portrait), C (landscape)
# a, b, c (A, B, C with red/blue swapped)
# 1, 2, 3, 4 for each of A, B, C, a, b, c

configs = [[[], [], [], []], 
           [[], [], [], []], 
           [[], [], [], []], 
           [[], [], [], []], 
           [[], [], [], []], 
           [[], [], [], []]]
configs[0][0:4] = [[x_far, y_lateral, x_near, y_medial, x_far, y_medial, x_near, y_lateral]] *4 # A
configs[1][0:4] = [[x_far, y_lateral, x_near, y_lateral, x_far, y_medial, x_near, y_medial]] *4 # B
configs[2][0:4] = [[x_far, y_medial, x_far, y_lateral, x_near, y_medial, x_near, y_lateral]] *4 # C

for k in range(4):    
    configs[3][k] = configs[0][k][4:8] + configs[0][k][0:4] # a
    configs[4][k] = configs[1][k][4:8] + configs[1][k][0:4]  # b
    configs[5][k] = configs[2][k][4:8] + configs[2][k][0:4] # c

#print 
for i in range(6):
    print(configs[i])

for k in range(6):
    # swap red 1 with red 2
    configs[k][1][0], configs[k][1][1], configs[k][1][2], configs[k][1][3] = configs[k][1][2], configs[k][1][3], configs[k][1][0], configs[k][1][1]
    # swap blue 1 with blue 2
    configs[k][2][4], configs[k][2][5], configs[k][2][6], configs[k][2][7] = configs[k][2][6], configs[k][2][7], configs[k][2][4], configs[k][2][5] 
    # swap red 1 with red 2 and blue 1 with blue 2
    configs[k][3][0], configs[k][3][1], configs[k][3][2], configs[k][3][3] = configs[k][2][2], configs[k][2][3], configs[k][2][0], configs[k][2][1]

folder_path = os.path.expanduser('~/ws_arm/src/arm_sim/box_blocks_experiment/config/')

A1 = configs[0][0]
A2 = configs[0][1]
A3 = configs[0][2]
A4 = configs[0][3]
B1 = configs[1][0]
B2 = configs[1][1]
B3 = configs[1][2]
B4 = configs[1][3]
C1 = configs[2][0]
C2 = configs[2][1]
C3 = configs[2][2]
C4 = configs[2][3]
a1 = configs[3][0]
a2 = configs[3][1]
a3 = configs[3][2]
a4 = configs[3][3]
b1 = configs[4][0]
b2 = configs[4][1]
b3 = configs[4][2]
b4 = configs[4][3]
c1 = configs[5][0]
c2 = configs[5][1]
c3 = configs[5][2]
c4 = configs[5][3]

# plan = [[A1, C2, B3, A4, a1, c2, b3, a4],
#          [B1, A2, C3, B4, b1, a2, c3, b4], 
#          [C1, B2, A3, C4, c1, b2, a3, c4]]

plan = [A1, B1, C1, a1, b1, c1, A4, B4, C4, a4, b4, c4, A2, B2, C2, a2, b2, c2, A3, B3, C3, a3, b3, c3]

for j in range(len(plan)): # trial
    locations = plan[j]      
    data = {}
    for name, value in zip(variable_names, locations):
        data[name] = value
    file_path = os.path.join(folder_path, f'trial{j+1}.yaml')

    with open(file_path, 'w') as file:
        yaml.dump(data, file)
