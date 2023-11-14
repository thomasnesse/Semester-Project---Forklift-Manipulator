clear, close all
clc


%% 3.2.2 Forward and inverse kinematics - robot arm
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;

T01 = [cos(theta1), -sin(theta1)*cos(-pi/2), sin(theta1)*sin(-pi/2), 0;
       sin(theta1), cos(theta1)*cos(-pi/2), -cos(theta1)*sin(-pi/2), -pi/2*sin(theta1);
       0, sin(-pi/2), cos(-pi/2), 0.2;
       0, 0, 0, 1];

T12 = [cos(theta2), -sin(theta2), 0, 1.59*cos(theta2);
       sin(theta2), cos(theta2), 0, 0;
       0, 0, 1, 0; 
       0, 0, 0, 1];

T23 = [cos(theta3), -sin(theta3), 0, 1.59*cos(theta3);
       sin(theta3), cos(theta3), 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];

T34 = [cos(theta4), -sin(theta4), 0, 1.2*cos(theta4);
       sin(theta4), cos(theta4), 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];

T_manually = (T01*T12*T23*T34) 

%% Demonstrate equivalence of the forward kinematic solution 
import ETS3.*
L(1) = Link('revolute', 'd', 0.2, 'a', 0, 'alpha', -pi/2, 'offset', 0);
L(2) = Link('revolute', 'd', 0, 'a', 1.59, 'alpha', 0, 'offset', 0);
L(3) = Link('revolute', 'd', 0, 'a', 1.59, 'alpha', 0, 'offset', 0);
L(4) = Link('revolute', 'd', 0, 'a', 1.2, 'alpha', 0, 'offset', 0);
robot = SerialLink(L,'name', 'Forklift Manipulator');
q = [0 0 0 0];

T_Toolbox = robot.fkine(q) 

%% 3.2.2 Develop the inverse kinematics, and demonstrate how it could be used
mask = [1 1 1 1 0 0];

q_inverse_kinematics = robot.ikine(T_Toolbox, 'mask', mask)


%% 3.3 Trajectory planning
q0 = [deg2rad(0) deg2rad(-86.4) deg2rad(-3.6) deg2rad(154.8)]; %Hvileposisjon
q1 = [deg2rad(0) deg2rad(-68.4) deg2rad(68.4) deg2rad(0)]; %Plukke opp palle
q2 = [deg2rad(0) deg2rad(-90.0) deg2rad(0) deg2rad(90)]; %Hvileposisjon med palle
q3 = [deg2rad(-180) deg2rad(-90.0) deg2rad(0) deg2rad(90)]; %Snu 180 grader
q4 = [deg2rad(-180) deg2rad(-68.4) deg2rad(68.4) deg2rad(0)]; %Levere palle
q5 = [deg2rad(-180) deg2rad(-86.4) deg2rad(-3.6) deg2rad(154.8)]; %Hvileposisjon

%Creating trajectories
steps = 50;
qi = jtraj(q0, q1, steps);
qii = jtraj(q1, q2, steps);
qiii = jtraj(q2, q3, steps);
qiv = jtraj(q3, q4, steps);
qv = jtraj(q4, q5, steps);

% Trajectory matrix
Qmatrix = [qi; qii; qiii; qiv; qv];

% Teach function to manually set angles
%robot.teach 

% Plotting combined trajectory
robot.plot(Qmatrix, 'trail', 'r');


%% 3.4 Differential kinematics
% Using jacobe to calculate force on each joint

force = [0 4905 0 0 0 0]; %4905N force applied in the Y-direction in end effector frame when pallet is 500kg
Jacobiforce = robot.jacobe(q1)' * force'; %The q1 position is where the force is the highest
JacobiForceNewton = Jacobiforce'


