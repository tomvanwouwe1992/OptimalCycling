%% EoM quadruple Pendulum


clc; clear;
disp('Calculating the equations of motion symbolically...')
syms q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5 x dx ddx y dy ddy T1 T2 T3 T4 T5  'real'; 
syms q1_plus q2_plus q3_plus q4_plus q5_plus dq1_plus dq2_plus dq3_plus dq4_plus dq5_plus
syms q1_min q2_min q3_min q4_min q5_min dq1_min dq2_min dq3_min dq4_min dq5_min
syms Fx Fy

% th_A == hook of angle to world axisses
% th_K == 
% th_H == 
% dth_A == first time derivative of th_A
% dth_K == first time derivative of th_K
% ddth_A == second time derivative of th_A
% ddth_K == second time derivative of th_K

syms m1 m2 m3 m4 m5 g l1 l2 l3 l4 l5 I1 I2 I3 I4 I5 lc1 lc2 lc3 lc4 lc5 'real';
% m1, m2, m3 == mass of the pendulum links
% g == acceleration due to gravity
% l1, l2, l3== length of the pendulum's first and second links
% I1, I2, I3 == moment of inertias of the first and second links
% lc1, lc2, lc3 == distance between link's parent joint and CoM


% Note on the ANGLE DEFINITIONS:
% The angles are the positive angles with respect to the direction of the
% previous segment. For the first angle (th_A) - this is the positive angle
% with respect to the y-axis.

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            vector stuff                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

% Positions of joints and link COMs
P_0 = [0;0;0];

G_1 = P_0 + lc1*[cos(q1 + pi/2); sin(q1 + pi/2);0];
P_1 = P_0 + l1*[cos(q1 + pi/2); sin(q1 + pi/2);0];

G_2 = P_1 + lc2*[cos(q2 + pi/2); sin(q2 + pi/2);0];
P_2 = P_1 + l2*[cos(q2 + pi/2); sin(q2 + pi/2);0];

G_3 = P_2 + lc3*[cos(q3 + pi/2); sin(q3 + pi/2);0];
P_3 = P_2 + l3*[cos(q3 + pi/2); sin(q3 + pi/2);0];

G_4 = P_3 + lc4*[cos(q4 + pi/2); sin(q4 + pi/2);0];
P_4 = P_3 + l4*[cos(q4 + pi/2); sin(q4 + pi/2);0];


% Collect in column vectors
P = [P_0; P_1; P_2; P_3; P_4];
G = [G_1; G_2; G_3; G_4];
q = [q1; q2; q3; q4]; 
dq = [dq1; dq2; dq3; dq4]; 
ddq = [ddq1; ddq2; ddq3; ddq4];

% Get velocities
dP_1 = jacobian(P_1,q)*dq; dP_2 = jacobian(P_2,q)*dq; dP_3 = jacobian(P_3,q)*dq; dP_4 = jacobian(P_4,q)*dq;
dG_1 = jacobian(G_1,q)*dq; dG_2 = jacobian(G_2,q)*dq; dG_3 = jacobian(G_3,q)*dq; dG_4 = jacobian(G_4,q)*dq;  

% Get accelerations
ddP_1 = jacobian(dP_1,[q;dq])*[dq; ddq]; ddP_2 = jacobian(dP_2,[q;dq])*[dq; ddq]; ddP_3 = jacobian(dP_3,[q;dq])*[dq; ddq]; ddP_4 = jacobian(dP_4,[q;dq])*[dq; ddq]; 
ddG_1 = jacobian(dG_1,[q;dq])*[dq; ddq]; ddG_2 = jacobian(dG_2,[q;dq])*[dq; ddq]; ddG_3 = jacobian(dG_3,[q;dq])*[dq; ddq]; ddG_4 = jacobian(dG_4,[q;dq])*[dq; ddq]; 

% External force at saddle
Fext = [Fx;Fy;0];
% EQUATIONS OF MOTION
% Write equations for angular momentum balance about each joint o get five
% independent equations in order to construct EoM

% 1.
eq1 = [0; 0; T1 + T5] + k.*(cross((G_1 - P_0),(-m1*g*j)) + cross((G_2 - P_0),(-m2*g*j)) + cross((G_3 - P_0),(-m3*g*j)) + cross((G_4 - P_0),(-m4*g*j))) + ...
    k.*(cross(P_4-P_0,Fext)) - ...
    k.*(cross((G_1 - P_0),(m1*ddG_1)) + cross((G_2 - P_0),(m2*ddG_2)) + cross((G_3 - P_0),(m3*ddG_3)) + cross((G_4 - P_0),(m4*ddG_4)) + ...
    ddq1*I1*k + ddq2*I2*k + ddq3*I3*k + ddq4*I4*k);
eq1 = eq1(3);


% 2.
eq2 = [0; 0; T2 + T5] + k.*(cross((G_2 - P_1),(-m2*g*j)) + cross((G_3 - P_1),(-m3*g*j)) + cross((G_4 - P_1),(-m4*g*j))) + ...
    k.*(cross(P_4-P_1,Fext)) - ...
    k.*(cross((G_2 - P_1),(m2*ddG_2)) + cross((G_3 - P_1),(m3*ddG_3)) + cross((G_4 - P_1),(m4*ddG_4)) + ...
    ddq2*I2*k + ddq3*I3*k + ddq4*I4*k);
eq2 = eq2(3);

% 3.
eq3 = [0; 0; T3 + T5] + k.*(cross((G_3 - P_2),(-m3*g*j)) + cross((G_4 - P_2),(-m4*g*j))) + ...
    k.*(cross(P_4-P_2,Fext)) - ...
    k.*(cross((G_3 - P_2),(m3*ddG_3)) + cross((G_4 - P_2),(m4*ddG_4)) + ...
    ddq3*I3*k + ddq4*I4*k);
eq3 = eq3(3);

% 4.
eq4 = [0; 0; T4 + T5] + k.*(cross((G_4 - P_3),(-m4*g*j))) + ...
    k.*(cross(P_4-P_3,Fext)) - ...
    k.*(cross((G_4 - P_3),(m4*ddG_4)) + ...
    ddq4*I4*k);
eq4 = eq4(3);

eq_SysDyn = simplify([eq1; eq2; eq3; eq4]);
eq_SysDyn_fcn = matlabFunction(eq_SysDyn,'File','eq_SysDyn.m');

JointPos = P([1 2 4 5 7 8 10 11 13 14],:);
JointVel = [0; 0; dP_1(1:2); dP_2(1:2); dP_3(1:2); dP_4(1:2)];

P_fcn = matlabFunction(JointPos,'File','JointPos.m');
dP_fcn = matlabFunction(JointVel,'File','JointVel.m');

% Get relative joint angles
q_PEDAL = q1;
q_FOOT = q1 - q2;
q_ANKLE = q2 - q3;
q_KNEE = q3 - q4;
q_HIP = q4;
relativeJointPos = [q_PEDAL; q_FOOT; q_ANKLE; q_KNEE; q_HIP];
Prel_fcn = matlabFunction(relativeJointPos,'File','relativeJointPos.m');

% Get relative joint velocities
dq_PEDAL = dq1;
dq_FOOT = dq1 - dq2;
dq_ANKLE = dq2 - dq3;
dq_KNEE = dq3 - dq4;
dq_HIP = dq4;
relativeJointVel = [dq_PEDAL; dq_FOOT; dq_ANKLE; dq_KNEE; dq_HIP];
dPrel_fcn = matlabFunction(relativeJointVel,'File','relativeJointVel.m');

