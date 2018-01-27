[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic Arm Pick & Place Project

## Writeup by Muthanna A. Attyah
## Feb 2018

[//]: # (Image References)

[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
---

### Kinematic Analysis

#### Extracting joint positions and orientations from URDF file.

from the URDF file `kr210.urdf.xacro` we can extract the position xyz and orientation rpy of each joint from **origin** tag in each joint XML section:

for example, from the following fixed base joint XML section:

```xml
 <!-- joints -->
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
```
In the **origin** tag:
```xml
<origin xyz="0 0 0" rpy="0 0 0"/>
```
We can see that **xyz="0 0 0"** and **rpy="0 0 0"**. Following table is showing complete extracted list for all joints base to gripper:

joint | parant | child | x | y | z | r | p | y |
--- | --- | --- | --- | --- | --- | --- | --- | --- |
fixed_base | base_footprint | base_link | 0 | 0 | 0 | 0 | 0 | 0 |
joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0 |
joint_2 | link_1 | link_2 | 0 .35| 0 | 0.42 | 0 | 0 | 0 |
joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0 |
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 |
joint_5 | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0 |
joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0 |
gripper | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0 |
**Total (m)** |  |  | 2.153 | 0 | 1.946 | 0 | 0 | 0 |

Below figure is showing the KR210 forward kinematics diagram using above joint data:

<p align="center"> <img src="./misc_images/KR210_FK.jpg"> </p>

Note that:
---
**Origin** O(i) = intersection between Xi and Zi axies

**Link Length:** a(i-1) = Zi-1 - Zi along the X(i-1) axis

**Link Offset:** d(i) = X(i-1) - X(i) along Z(i) axis

**Link Twist:** alpha(i-1) = angle from Z(i-1) to Z(i) mesured about Xi-1 using right hand rule

**Joint Angle:** theta(i) = angle from X(i-1) to X(i) mesured about Zi using right hand rule. all joint angles will be zero at initial Robot state in KR210 except joint 2 which has a -90 degree constant offset between X(1) and X(2).

**Gripper frame:** is the end point that we care about. it is displaced from Frame 6 by a translation along Z(6).


### calculating alpha(i), a(i), and d(i)

We can use the following to generate values for alpha, a, and d:







i | alpha(i-1) | a(i-1) | d(i) | theta(i) |
--- | --- | --- | --- | --- |
1 | 0 | 0 | 0.75 | 0 |
2 | -90 | 0.35 | 0 | -90 |
3 | 0 |  | 1.25 | 0 |
4 | -90 | -0.05 | 1.5 | 0 |
5 | 90 | 0 | 0 | 0 |
6 | -90 | 0 | 0 | 0 |
7 | 0 | 0 | 0.303 | 0 |

### Kuka KR210 robot DH parameters.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


Your writeup should contain individual transform matrices about each joint using the DH table and a homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link. These matrices can be created using any software of your choice or hand written. Also include an explanation on how you created these matrices.


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Based on the geometric Inverse Kinematics method described here, breakdown the IK problem into Position and Orientation problems. Derive the equations for individual joint angles. Your writeup must contain details about the steps you took to arrive at those equations. Add figures where necessary. If any given joint has multiple solutions, select the best solution and provide explanation about your choice (Hint: Observe the active robot workspace in this project and the fact that some joints have physical limits).


And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

IK_server.py must contain properly commented code. The robot must track the planned trajectory and successfully complete pick and place operation. Your writeup must include explanation for the code and a discussion on the results.

Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

And just for fun, another example image:
![alt text][image3]


### Standout submission

To have a standout submission, calculate and plot the error in end-effector pose generated by your joint angle commands. You can do this by calculating end-effector poses via Forward Kinematics using your code output (which is a set of joint angles) and comparing it with the end-effector poses you received as input.
