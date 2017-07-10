## Project: Kinematics Pick & Place

[//]: # (Image References)

[corr]: ./results/corr_1000.png
[dh-params]: ./results/dh-params_1000.png
[hom-transform]: ./results/hom-transform_1000.png
[inv-main]: ./results/inv-main_1000.png
[wrist-pos]: ./results/wrist-pos_1000.png
[wrist-pos-geom]: ./results/wrist-pos-geom_1000.png
[q3-deriv]: ./results/q3-deriv_1000.png
[q2-deriv]: ./results/q2-deriv_1000.png
[wrist-rot]: ./results/wrist-rot_1000.png
[wrist-rot-cases]: ./results/wrist-rot-cases_1000.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! (yep thats left from template:)

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![DH Parameters][dh-params]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Homogeneous transformations from base-link {0} to gripper_link {G} are below:

![Homogeneous transformations][hom-transform]

In order to check forward kinematics transformations we need to apply correction to the gripper_link that converts from our reference frame selected during DH parameters selection to the standard URDF (RViz) orientation.

![Correction for gripper_link][corr]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Yep, there a lot of images here for the derivation.

First, let's look how our arm looks like in some random position.

![Inverse kinematics arm position][inv-main]

Wrist position could be found by moving along Z axis of the final rotation (R) backwards on the d7.

![Wrist position][wrist-pos]

O - gripper orientation and position in matrix form.

Having {Wc} we can solve inverse kinematics problem for first three angles `q1`, `q2`, `q3` geometrically on the plane. Below are the select plane (A - B - Wc - C) transformation and notations that simplify the problem to a chain of two links.

![Wrist position geometry transformations][wrist-pos-geom]

Angle `q3` could be found be applying cosine law (2.1) to the triangle (A-C-Wc).

![q3 derivation][q3-deriv]

`q2` angle formula depends on the `q3` angle (`beta` to be exact) that gives us the next formula:

![q2 derivation][q2-deriv]

To find wrist rotation (`q4`, `q5` and `q6`) we decompose the rotation from base link to wrist center in two parts and calculates the remaining transform symbolically (using DH parameters) and numerically (using already found `q1`, `q2` and `q3`).

![Wrist rotation][wrist-rot]

There two special cases here: 1) when `q5` = 0 and 2) when `q5` = pi. In both these cases `sin(q5) = 0` and we have a singularity and we have multiple possible solutions. Below is the derivation of these solutions:

![Wrist rotation special cases][wrist-rot-cases]

#### More than one possible solution exists

`q1` could have also value `q1` + pi so when arm can swing back along `joint_2` but this is not for our Kuka arm because `joint_2` movements is limited to just (-45 deg, 85 deg).

`q3` could have two possible values +/- options in formula (2.3) that corresponds to elbow-up and elbow-down configuration but again this is not possible for our Kuka arm because `joint_2` limits are not giving us the option to make elbow-down form.

`q2` and `q5` looks like can be in second configuration as well but I didn't have time to analyze them thoroughly...

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

I've implemented Forward and Inverse Kinematics logic in `fk-ik.pynb` file and later moved almost without changes code to the `IK_server.py`.

One of the problem is that singularity cases are not handling the previous state and sometime arm is just rotating almost 360 degrees without any need for this. In order to fix it we could add the logic that checks previous theta angles and selects the closest pair for angles `q4` and `q6` for cases when `sin(q5) = 0` i.e. singular situation where infinite number of solution exists.

Sometimes simulator stops grasping objects correctly, it's just not closing enough the gripper links and objects remain on the shelf. Not sure is it the simulation + speed of my machine issue or I should change something in IK code (but seems former is more probable).

But the biggest issue is my MacBook Pro 13 inch and VM that runs on about 3-6 FPS which is making big trouble in testing everything together in ROS. I am just about to buy a new machine for this course. (that's my problem and I will solve it)

Need more integration testing on a better machine ...
