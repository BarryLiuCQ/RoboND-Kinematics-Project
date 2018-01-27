### KUKA K210 Forward Kinematics ###

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix

# Create symbols for DH param
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')                                 # joint angles theta
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')                                 # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')                                 # link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # joint twist angles

# DH Table
s = {alpha0:      0, a0:      0, d1:  0.75, q1:        q1,
     alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
     alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
     alpha3: -pi/2., a3: -0.054, d4:  1.50, q4:        q4,
     alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
     alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
     alpha6:      0, a6:      0, d7: 0.303, q7:         0}

#### Homogeneous Transforms
## (Base) Link_0 to Link_1
T0_1 = Matrix([ [             cos(q1),            -sin(q1),            0,              a0],
                [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                [                   0,                   0,            0,               1]])
## Link_1 to Link_2
T1_2 = Matrix([ [             cos(q2),            -sin(q2),            0,              a1],
                [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                [                   0,                   0,            0,               1]])
## Link_2 to Link_3
T2_3 = Matrix([ [             cos(q3),            -sin(q3),            0,              a2],
                [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                [                   0,                   0,            0,               1]])
## Link_3 to Link_4
T3_4 = Matrix([ [             cos(q4),            -sin(q4),            0,              a3],
                [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                [                   0,                   0,            0,               1]])
## Link_4 to Link_5
T4_5 = Matrix([ [             cos(q5),            -sin(q5),            0,              a4],
                [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                [                   0,                   0,            0,               1]])
## Link_5 to Link_6
T5_6 = Matrix([ [             cos(q6),            -sin(q6),            0,              a5],
                [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                [                   0,                   0,            0,               1]])
## Link_6 to Link_7 (end effector)
T6_7 = Matrix([ [             cos(q7),            -sin(q7),            0,              a6],
                [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                [                   0,                   0,            0,               1]])


## Substiute DH_Table
T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_7 = T6_7.subs(s)

# Composition of Homogeneous Transforms
# Transform from Base link to end effector (Gripper)
T0_2 = simplify(T0_1 * T1_2) ## (Base) Link_0 to Link_2
T0_3 = simplify(T0_2 * T2_3) ## (Base) Link_0 to Link_3
T0_4 = simplify(T0_3 * T3_4) ## (Base) Link_0 to Link_4
T0_5 = simplify(T0_4 * T4_5) ## (Base) Link_0 to Link_5
T0_6 = simplify(T0_5 * T5_6) ## (Base) Link_0 to Link_6
T0_7 = simplify(T0_6 * T6_7) ## (Base) Link_0 to Link_E (End Effector)

# Correction Needed to Account for Orientation Difference Between
# Difinition of Gripper Link_G in URDF versus DH Convention


R_y = Matrix([[ cos(-np.pi/2),           0, sin(-np.pi/2), 0],
              [             0,           1,             0, 0],
              [-sin(-np.pi/2),           0, cos(-np.pi/2), 0],
              [             0,           0,             0, 1]])

R_z = Matrix([[    cos(np.pi), -sin(np.pi),             0, 0],
              [    sin(np.pi),  cos(np.pi),             0, 0],
              [             0,           0,             1, 0],
              [             0,           0,             0, 1]])


R_corr = simplify(R_z * R_y)


### Numerically evaluate transforms (compare this to output of tf_echo)
print("\nT0_1 = \n")
pprint(T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("\nT0_2 = \n")
pprint(T0_2.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("\nT0_3 = \n")
pprint(T0_3.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("\nT0_4 = \n")
pprint(T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("\nT0_5 = \n")
pprint(T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("\nT0_6 = \n")
pprint(T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))


# Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_E
# With orientation correction applied
T_total = simplify(T0_7 * R_corr)

print("\nT_total Matrix : \n")
pprint(T_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))

