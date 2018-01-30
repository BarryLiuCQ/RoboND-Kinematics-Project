#!/usr/bin/env python3

from sympy import symbols, cos, sin, pi, sqrt
from sympy.matrices import Matrix

### Create symbols for joint variables
q1, q2 = symbols('q1:3')

### Create symbols for joint variables
q1, q2 = symbols('q1:3')

# Create a symbolic matrix representing an intrinsic sequence of rotations 
# about the Y and then Z axes. Let the rotation about the Y axis be described
# by q1 and the rotation about Z by q2. 

# Replace R_y and R_z with the appropriate (symbolic) elementary rotation matrices 
# and then compute YZ_intrinsic. 

R_y = Matrix([[ cos(q1),        0, sin(q1)],
              [ 0,              1,       0],
              [-sin(q1),        0, cos(q1)]])
R_z = Matrix([[ cos(q2), -sin(q2),       0],
              [ sin(q2),  cos(q2),       0],
              [       0,        0,       1]])

YZ_intrinsic_sym = R_y * R_z
YZ_intrinsic_num = YZ_intrinsic_sym.evalf(subs={q1: pi/4, q2: pi/3})

print(YZ_intrinsic_sym)
print(YZ_intrinsic_num)
