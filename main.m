clear all; close all; clc;
import casadi.*

% Parameters of our cyclist (masses, lengths inertias, ...)
m1 = 0.3; 
m2 = 0.3; 
m3 = 3.2;
m4 = 3.2;
m8 = 7.9;
m9 = 23.1;
m10 = 1.4;
m11 = 0.8;
I1 = 0.2; 
I2 = 0.2; 
I3 = 0.93;
I4 = 1.08;
I8 = 0.05;
I9 = 1.43;
I10 = 0.007;
I11 = 0.002;
l1 = 0.175; 
l2 = 0.10; 
l3 = 0.4;
l4 = 0.4;
l8 = 0.1;
l9 = 0.65;
l10 = 0.32;
l11 = 0.25;
lc1 = l1/2;
lc2 = l2/2;
lc3 = l3/2;
lc4 = l4/2;
lc8 = l8/2;
lc9 = l9/2;
lc10 = l10/2;
lc11 = l11/2;
g = 9.81;

% Declare CasADi variables that will be used to create CasADi function
q1_MX = MX.sym('q1_MX',1); q2_MX = MX.sym('q2_MX',1); q3_MX = MX.sym('q3_MX',1); q4_MX = MX.sym('q4_MX',1); 
dq1_MX = MX.sym('dq1_MX',1); dq2_MX = MX.sym('dq2_MX',1); dq3_MX = MX.sym('dq3_MX',1); dq4_MX = MX.sym('dq4_MX',1); 
ddq1_MX = MX.sym('ddq1_MX',1); ddq2_MX = MX.sym('ddq2_MX',1); ddq3_MX = MX.sym('ddq3_MX',1); ddq4_MX = MX.sym('ddq4_MX',1); 
T1_MX = MX.sym('T1_MX',1); T2_MX = MX.sym('T2_MX',1); T3_MX = MX.sym('T3_MX',1); T4_MX = MX.sym('T4_MX',1); T5_MX = MX.sym('T5_MX',1);
Fx_MX = MX.sym('Fx_MX',1);Fy_MX = MX.sym('Fy_MX',1);
% Generate a CasADi function for the implicit constraint in order to satisfy the system dynamics --> f(T,q,dq,ddq) == 0
eq_SysDyn_Error = eq_SysDyn(Fx_MX,Fy_MX,I1,I2,I3,I4,T1_MX,T2_MX,T3_MX,T4_MX,T5_MX,ddq1_MX,ddq2_MX,ddq3_MX,ddq4_MX,dq1_MX,dq2_MX,dq3_MX,dq4_MX,g,l1,l2,l3,l4,lc1,lc2,lc3,lc4,m1,m2,m3,m4,q1_MX,q2_MX,q3_MX,q4_MX);
f_eq_SysDyn_Error_Nominal = Function('f_eq_SysDyn_Error_Nominal',{Fx_MX,Fy_MX,T1_MX,T2_MX,T3_MX,T4_MX,T5_MX,ddq1_MX,ddq2_MX,ddq3_MX,ddq4_MX,dq1_MX,dq2_MX,dq3_MX,dq4_MX,q1_MX,q2_MX,q3_MX,q4_MX},{eq_SysDyn_Error});


eq_SysDynUpperBody_Error = eq_SysDyn(Fx_MX,Fy_MX,I8,I9,I10,I11,T1_MX,T2_MX,T3_MX,T4_MX,T5_MX,ddq1_MX,ddq2_MX,ddq3_MX,ddq4_MX,dq1_MX,dq2_MX,dq3_MX,dq4_MX,g,l8,l9,l10,l11,lc8,lc9,lc10,lc11,m8,m9,m10,m11,q1_MX,q2_MX,q3_MX,q4_MX);
eq_SysDynUpperBody_Error_Nominal = Function('eq_SysDynUpperBody_Error_Nominal',{Fx_MX,Fy_MX,T1_MX,T2_MX,T3_MX,T4_MX,T5_MX,ddq1_MX,ddq2_MX,ddq3_MX,ddq4_MX,dq1_MX,dq2_MX,dq3_MX,dq4_MX,q1_MX,q2_MX,q3_MX,q4_MX},{eq_SysDynUpperBody_Error});


% Time horizon and mesh size for simulation
dt = 0.01;  % Mesh size
T = 1;    % Stride time
N = T/dt;   % Nr mesh intervals
time = 0:dt:T;

opti = casadi.Opti(); % Create opti instance

% Create optimization variables
q1 = opti.variable(1,N+1);   q2 = opti.variable(1,N+1);   q3 = opti.variable(1,N+1);   q4 = opti.variable(1,N+1);   q5 = opti.variable(1,N+1);     q6 = opti.variable(1,N+1);     q7 = opti.variable(1,N+1);    q8 = opti.variable(1,N+1);   q9 = opti.variable(1,N+1);     q10 = opti.variable(1,N+1);     q11 = opti.variable(1,N+1);
dq1 = opti.variable(1,N+1);  dq2 = opti.variable(1,N+1);  dq3 = opti.variable(1,N+1);  dq4 = opti.variable(1,N+1);  dq5 = opti.variable(1,N+1);    dq6 = opti.variable(1,N+1);    dq7 = opti.variable(1,N+1);   dq8 = opti.variable(1,N+1);  dq9 = opti.variable(1,N+1);    dq10 = opti.variable(1,N+1);    dq11 = opti.variable(1,N+1);
ddq1 = opti.variable(1,N);   ddq2 = opti.variable(1,N);   ddq3 = opti.variable(1,N);   ddq4 = opti.variable(1,N);   ddq5 = opti.variable(1,N+1);   ddq6 = opti.variable(1,N+1);   ddq7 = opti.variable(1,N+1);  ddq8 = opti.variable(1,N);   ddq9= opti.variable(1,N);      ddq10 = opti.variable(1,N+1);   ddq11 = opti.variable(1,N+1);   
T1 = opti.variable(1,N);     T2 = opti.variable(1,N);     T3 = opti.variable(1,N);     T4 = opti.variable(1,N);     T5 = opti.variable(1,N);       T6 = opti.variable(1,N);       T7 = opti.variable(1,N);      T8 = opti.variable(1,N);     T9 = opti.variable(1,N);       T10 = opti.variable(1,N);       T11 = opti.variable(1,N);  
T1_leg2 = opti.variable(1,N); Th_right = opti.variable(1,N); Th_left = opti.variable(1,N); 
Fx = opti.variable(1,N);     Fy = opti.variable(1,N);
Fx_leg2 = opti.variable(1,N);     Fy_leg2 = opti.variable(1,N);
Fx_hand = opti.variable(1,N);     Fy_hand = opti.variable(1,N);

% Crude bounds on the segment orientations
opti.subject_to(0 < q2 < pi);
opti.subject_to(-pi/2 < q3 < pi/2);
opti.subject_to(-3*pi/2 < q4 < pi/2);
opti.subject_to(-pi/3 < q8 < pi/8);


% Physiological joint limits
% opti.subject_to(-pi < q1 - q2 < 0); % Knee joint limit (no hyperflexion)
opti.subject_to(-pi < q3 - q4 < 0); % Knee joint limit (no hyperflexion)
opti.subject_to(pi/6 < q2-q3 < 3*pi/4); % Ankle joint limit (no hyperflexion)

opti.subject_to(-pi < q6 - q7 < 0); % Knee joint limit (no hyperflexion)
opti.subject_to(pi/6 < q5-q6 < 3*pi/4); % Ankle joint limit 

opti.subject_to(-pi/2 < q8-q9 < pi/2 ); % Shoulder joint limits
opti.subject_to(0 < q9-q10 < pi); % Shoulder joint limits
opti.subject_to(-pi < q10-q11 < -pi/12); % Elbow joint limits

% opti.subject_to(dq1 < -pi);

% Generate an initial guess for the positions (done very naively!)
q1_init = 0; q2_init = pi/3; q3_init = 0; q4_init = pi/4; q5_init = pi/3; q6_init = 0; q7_init = pi/4; q8_init = -pi/6; q9_init = -pi/4; q10_init = -2*pi/3; q11_init = -pi/2; 
q1_final = -2*pi; q2_final = pi/3; q3_final = 0; q4_final = pi/4; q5_final = pi/3; q6_final = 0; q7_final = pi/4; q8_final = -pi/6; q9_final = -pi/4; q10_final = -2*pi/3; q11_final = -pi/2; 
q1guess = linspace( q1_init, q1_final, N+1);
q2guess = linspace( q2_init, q2_final, N+1);
q3guess = linspace( q3_init, q3_final, N+1);
q4guess = linspace( q4_init, q4_final, N+1);
q5guess = linspace( q5_init, q5_final, N+1);
q6guess = linspace( q6_init, q6_final, N+1);
q7guess = linspace( q7_init, q7_final, N+1);
q8guess = linspace( q8_init, q8_final, N+1);
q9guess = linspace( q9_init, q9_final, N+1);
q10guess = linspace( q10_init, q10_final, N+1);
q11guess = linspace( q11_init, q11_final, N+1);

opti.set_initial(q1, q1guess);
opti.set_initial(q2, q2guess);
opti.set_initial(q3, q3guess);
opti.set_initial(q4, q4guess);
opti.set_initial(q5, q5guess);
opti.set_initial(q6, q6guess);
opti.set_initial(q7, q7guess);
opti.set_initial(q8, q8guess);
opti.set_initial(q9, q9guess);
opti.set_initial(q10, q10guess);
opti.set_initial(q11, q11guess);
% opti.set_initial(dq1, -2*pi);

P_J_init_leg1 = JointPos(l1,l2,l3,l4,q1_init,q2_init,q3_init,q4_init);
P_J_init_leg2 = JointPos(l1,l2,l3,l4,q1_init+pi,q5_init,q6_init,q7_init);

% Generate heel-strike map (periodicity for swing vs stance leg + impulsive
% collision constraint)
% Impulsive collision :: joint configuration does not change at collision,
% joint velocities do change, in such a way that angular momentum is
% conserved around the collision point and all joints (giving 5 constraint equations).
saddle_height = opti.variable(1,1);
reach = opti.variable(1,1);
stack_height = opti.variable(1,1);
saddle_x = -0.1;
% opti.set_initial(saddle_x, -0.1);
opti.set_initial(saddle_height, 0.7);


J = 0;
for k=1:N
    % State at mesh point k
    q1k = q1(:,k);     q2k = q2(:,k);     q3k = q3(:,k);     q4k = q4(:,k);    q5k = q5(:,k);     q6k = q6(:,k);     q7k = q7(:,k);   q8k = q8(:,k);    q9k = q9(:,k);     q10k = q10(:,k);     q11k = q11(:,k); 
    dq1k = dq1(:,k);   dq2k = dq2(:,k);   dq3k = dq3(:,k);   dq4k = dq4(:,k);  dq5k = dq5(:,k);   dq6k = dq6(:,k);   dq7k = dq7(:,k); dq8k = dq8(:,k);  dq9k = dq9(:,k);   dq10k = dq10(:,k);   dq11k = dq11(:,k);    
    
    % Control/lifted state variable for mesh k
    ddq1k = ddq1(:,k); ddq2k = ddq2(:,k); ddq3k = ddq3(:,k); ddq4k = ddq4(:,k); ddq5k = ddq5(:,k); ddq6k = ddq6(:,k); ddq7k = ddq7(:,k); ddq8k = ddq8(:,k); ddq9k = ddq9(:,k); ddq10k = ddq10(:,k); ddq11k = ddq11(:,k); 
    T1k = T1(:,k);     T2k = T2(:,k);     T3k = T3(:,k);     T4k = T4(:,k);     T5k = T5(:,k);     T6k = T6(:,k);     T7k = T7(:,k);      T1_leg2k = T1_leg2(:,k); T8k = T8(:,k);  T9k = T9(:,k);   T10k = T10(:,k);    T11k = T11(:,k);
    Fxk = Fx(:,k);     Fyk = Fy(:,k);     Th_rightk = Th_right(:,k);   Th_leftk = Th_left(:,k);
    Fx_leg2k = Fx_leg2(:,k);     Fy_leg2k = Fy_leg2(:,k);
    Fx_handk = Fx_hand(:,k);     Fy_handk = Fy_hand(:,k);
	
    % State at mesh point k+1
    q1k_plus = q1(:,k+1);     q2k_plus = q2(:,k+1);     q3k_plus = q3(:,k+1);     q4k_plus = q4(:,k+1);    q5k_plus = q5(:,k+1);     q6k_plus = q6(:,k+1);     q7k_plus = q7(:,k+1);     q8k_plus = q8(:,k+1);    q9k_plus = q9(:,k+1);     q10k_plus = q10(:,k+1);     q11k_plus = q11(:,k+1); 
    dq1k_plus = dq1(:,k+1);   dq2k_plus = dq2(:,k+1);   dq3k_plus = dq3(:,k+1);   dq4k_plus = dq4(:,k+1);  dq5k_plus = dq5(:,k+1);   dq6k_plus = dq6(:,k+1);   dq7k_plus = dq7(:,k+1);   dq8k_plus = dq8(:,k+1);  dq9k_plus = dq9(:,k+1);   dq10k_plus = dq10(:,k+1);   dq11k_plus = dq11(:,k+1);
       
    % Collect state at k and k+1
    Xk = [q1k; q2k; q3k; q4k; q5k; q6k; q7k; q8k; q9k; q10k; q11k; dq1k; dq2k; dq3k; dq4k; dq5k; dq6k; dq7k; dq8k; dq9k; dq10k; dq11k];
    Xk_next = [q1k_plus; q2k_plus; q3k_plus; q4k_plus; q5k_plus; q6k_plus; q7k_plus; q8k_plus; q9k_plus; q10k_plus; q11k_plus; dq1k_plus; dq2k_plus; dq3k_plus; dq4k_plus; dq5k_plus; dq6k_plus; dq7k_plus; dq8k_plus; dq9k_plus; dq10k_plus; dq11k_plus];
    
    % Collect state derivative (we use backward (not forward) Euler!)
    Uk = [dq1k_plus; dq2k_plus; dq3k_plus; dq4k_plus; dq5k_plus; dq6k_plus; dq7k_plus; dq8k_plus; dq9k_plus; dq10k_plus; dq11k_plus; ddq1k; ddq2k; ddq3k; ddq4k; ddq5k; ddq6k; ddq7k; ddq8k; ddq9k; ddq10k; ddq11k];
    
    % Integration
    opti.subject_to(eulerIntegrator(Xk,Xk_next,Uk,dt) == 0);
       
    % Dynamics error (backward Euler - derivative at k+1)
    error = f_eq_SysDyn_Error_Nominal(Fxk,Fyk,T1k,T2k,T3k,T4k,Th_rightk,ddq1k,ddq2k,ddq3k,ddq4k,dq1k_plus,dq2k_plus,dq3k_plus,dq4k_plus,q1k_plus,q2k_plus,q3k_plus,q4k_plus);
    opti.subject_to(error == 0);
    
    error = f_eq_SysDyn_Error_Nominal(Fx_leg2k,Fy_leg2k,T1_leg2k,T5k,T6k,T7k,Th_leftk,ddq1k,ddq5k,ddq6k,ddq7k,dq1k_plus,dq5k_plus,dq6k_plus,dq7k_plus,q1k_plus+pi,q5k_plus,q6k_plus,q7k_plus);
    opti.subject_to(error == 0);
	
	error = eq_SysDynUpperBody_Error_Nominal(Fx_handk,Fy_handk,T8k,T9k,T10k,T11k,0,ddq8k,ddq9k,ddq10k,ddq11k,dq8k_plus,dq9k_plus,dq10k_plus,dq11k_plus,q8k_plus,q9k_plus,q10k_plus,q11k_plus);
    opti.subject_to(error == 0);
    
    % Cost function contributions
    % Main term is torque minimization, we add minimization of
    % accelerations for regularization (contribution is confirmed to be
    % small)
    J = J + (T1k.^2 + T2k.^2 + T3k.^2 + T4k.^2 + T1_leg2k.^2 + T5k.^2 + T6k.^2 + T7k.^2 + Th_leftk.^2 + Th_rightk.^2 + T8k.^2 + T9k.^2 + T10k.^2 + T11k.^2)*dt + 1e-1*(ddq1k.^2 + ddq2k.^2 + ddq3k.^2 + ddq4k.^2 + ddq5k.^2 + ddq6k.^2 + ddq7k.^2 + ddq8k.^2 + ddq9k.^2 + ddq10k.^2 + ddq11k.^2)*dt;
    
    % Joint locations in x-y plane
    P_J_leg1 = JointPos(l1,l2,l3,l4,q1k,q2k,q3k,q4k);
    P_J_leg2 = JointPos(l1,l2,l3,l4,q1k+pi,q5k,q6k,q7k);
    P_J_upperbody = JointPos(l8,l9,l10,l11,q8k,q9k,q10k,q11k);
	P_J_hand = [saddle_x;saddle_height] + P_J_upperbody(9:10);
    P_J_head = [saddle_x;saddle_height] + P_J_upperbody(5:6);
	opti.subject_to([reach; stack_height] - P_J_hand == 0);
    % Saddle position fixed
    opti.subject_to([saddle_x;saddle_height] - P_J_leg1(9:10) == 0);
    opti.subject_to([saddle_x;saddle_height] - P_J_leg2(9:10) == 0);
    
    % OPTIONAL CONSTRAINTS TO CHANGE GAIT PATTERN
    % Generate specific power
    Cd = ((P_J_head(2)-saddle_height)/0.3)^2;
    opti.subject_to(T1k + T1_leg2k  == -0.9*0.2*Cd*dq1k^3 - 0.1*0.2*dq1k^3);

    % No torque in pedal joint
    opti.subject_to(T2k  == 0.0);
	opti.subject_to(T5k  == 0.0);
    
	opti.subject_to(T8k - Th_leftk - Th_rightk  == 0.0);

end

opti.subject_to(Fy + Fy_leg2  > 0.0);
J = J + 1e-3*(sumsqr(Fx_hand) + sumsqr(Fy_hand))*dt;

% Fix beginning and end position of crank
opti.subject_to(q1(:,1) == 0);
opti.subject_to(q1(:,end) == -2*pi);

% Equal start and end positions and velocities
% opti.subject_to(q2(:,end) - q5(:,1) == 0);
% opti.subject_to(q3(:,end) - q6(:,1) == 0);
% opti.subject_to(q4(:,end) - q7(:,1) == 0);
% opti.subject_to(q2(:,1) - q5(:,end) == 0);
% opti.subject_to(q3(:,1) - q6(:,end) == 0);
% opti.subject_to(q4(:,1) - q7(:,end) == 0);
% opti.subject_to(dq2(:,end) - dq5(:,1) == 0);
% opti.subject_to(dq3(:,end) - dq6(:,1) == 0);
% opti.subject_to(dq4(:,end) - dq7(:,1) == 0);
% opti.subject_to(dq2(:,1) - dq5(:,end) == 0);
% opti.subject_to(dq3(:,1) - dq6(:,end) == 0);
% opti.subject_to(dq4(:,1) - dq7(:,end) == 0);


opti.subject_to(q2(:,end) - q2(:,1) == 0);
opti.subject_to(q3(:,end) - q3(:,1) == 0);
opti.subject_to(q4(:,end) - q4(:,1) == 0);
opti.subject_to(q5(:,1) - q5(:,end) == 0);
opti.subject_to(q6(:,1) - q6(:,end) == 0);
opti.subject_to(q7(:,1) - q7(:,end) == 0);
opti.subject_to(q8(:,1) - q8(:,end) == 0);
opti.subject_to(q9(:,1) - q9(:,end) == 0);
opti.subject_to(q10(:,1) - q10(:,end) == 0);
opti.subject_to(q11(:,1) - q11(:,end) == 0);

opti.subject_to(dq2(:,end) - dq2(:,1) == 0);
opti.subject_to(dq3(:,end) - dq3(:,1) == 0);
opti.subject_to(dq4(:,end) - dq4(:,1) == 0);
opti.subject_to(dq5(:,1) - dq5(:,end) == 0);
opti.subject_to(dq6(:,1) - dq6(:,end) == 0);
opti.subject_to(dq7(:,1) - dq7(:,end) == 0);
opti.subject_to(dq8(:,1) - dq8(:,end) == 0);
opti.subject_to(dq9(:,1) - dq9(:,end) == 0);
opti.subject_to(dq10(:,1) - dq10(:,end) == 0);
opti.subject_to(dq11(:,1) - dq11(:,end) == 0);
% Define cost
opti.minimize(J);

% Create an NLP solver
optionssol.ipopt.linear_solver = 'mumps';
optionssol.ipopt.tol = 1e-2;
optionssol.ipopt.constr_viol_tol = 1e-10;
optionssol.ipopt.dual_inf_tol = 1e-8;
optionssol.ipopt.compl_inf_tol = 1e-6;
optionssol.ipopt.max_iter = 10000;
% optionssol.ipopt.hessian_approximation = 'limited-memory';
optionssol.ipopt.mu_strategy = 'adaptive';
opti.solver('ipopt',optionssol);

diary('2D-gait_diary.txt');
sol = opti.solve();
diary off

% Plot the torques, positions and velocities
T1_sol = sol.value(T1);
T1_leg2_sol = sol.value(T1_leg2);
T2_sol = sol.value(T2);
T3_sol = sol.value(T3);
T4_sol = sol.value(T4);
Th_right_sol = sol.value(Th_right);
T5_sol = sol.value(T5);
T6_sol = sol.value(T6);
T7_sol = sol.value(T7);
Th_left_sol = sol.value(Th_left);

T8_sol = sol.value(T8);
T9_sol = sol.value(T9);
T10_sol = sol.value(T10);
T11_sol = sol.value(T11);

Fx_hand_sol = sol.value(Fx_hand);
Fy_hand_sol = sol.value(Fy_hand);
figure(1)
plot(time(1:end-1),T1_sol,time(1:end-1),T3_sol,time(1:end-1),T4_sol,time(1:end-1),Th_right_sol,time(1:end-1),T5_sol,time(1:end-1),T6_sol,time(1:end-1),T7_sol,time(1:end-1),Th_left_sol);
legend('crank','ankle_r','knee_r','hip_r','ankle_l','knee_l','hip_l');
xlabel(['time [s]'])
ylabel(['Torque [Nm]'])

EffortLegs = (sumsqr(T3_sol) + sumsqr(T4_sol) + sumsqr(Th_right_sol) + sumsqr(T6_sol) + sumsqr(T7_sol) + sumsqr(Th_left_sol))*dt;
EffortUpperBody = (sumsqr(T8_sol) + sumsqr(T9_sol) + sumsqr(T10_sol) + sumsqr(T11_sol))*dt;

ContactHand = 1e-3*(sumsqr(Fx_hand_sol) + sumsqr(Fy_hand_sol))*dt;
bar([EffortLegs EffortUpperBody ContactHand])
% AccelerationCost = 

q1_sol = sol.value(q1);
q2_sol = sol.value(q2);
q3_sol = sol.value(q3);
q4_sol = sol.value(q4);
q5_sol = sol.value(q5);
q6_sol = sol.value(q6);
q7_sol = sol.value(q7);
q8_sol = sol.value(q8);
q9_sol = sol.value(q9);
q10_sol = sol.value(q10);
q11_sol = sol.value(q11);
figure(2)
plot(time,180/pi*q1_sol,time,180/pi*q2_sol,time,180/pi*q3_sol,time,180/pi*q4_sol,time,180/pi*q5_sol,time,180/pi*q6_sol,time,180/pi*q7_sol);
legend('crank','pedal_r','tibia_r','femur_r','pedal_l','tibia_l','femur_l');
xlabel(['time [s]'])
ylabel(['segment pos [°]'])

dq1_sol = sol.value(dq1);
dq2_sol = sol.value(dq2);
dq3_sol = sol.value(dq3);
dq4_sol = sol.value(dq4);
figure(3)
plot(time,180/pi*dq1_sol,time,180/pi*dq2_sol,time,180/pi*dq3_sol,time,180/pi*dq4_sol);
legend('crank','pedal','tibia','femur');
xlabel(['time'])
ylabel(['segment vel [°]'])

ddq1_sol = sol.value(ddq1);
ddq2_sol = sol.value(ddq2);
ddq3_sol = sol.value(ddq3);
ddq4_sol = sol.value(ddq4);

% Reconstruct cost function
J_torque = (sumsqr(T1_sol) + sumsqr(T2_sol) + sumsqr(T3_sol) + sumsqr(T4_sol));
J_ddq = 1e-1*(sumsqr(ddq1_sol) + sumsqr(ddq2_sol) + sumsqr(ddq3_sol) + sumsqr(ddq4_sol));

% Animate the cycling pattern
P_J_leg1 = JointPos(l1,l2,l3,l4,q1_sol,q2_sol,q3_sol,q4_sol)';
P_J_leg2 = JointPos(l1,l2,l3,l4,q1_sol+pi,q5_sol,q6_sol,q7_sol)';
P_J_upperbody = (JointPos(l8,l9,l10,l11,q8_sol,q9_sol,q10_sol,q11_sol)+repmat([sol.value(saddle_x); sol.value(saddle_height)],5,1))';

animation(P_J_leg1,P_J_leg2,P_J_upperbody,dt);

relativeJointPos = relativeJointPos(q1_sol,q2_sol,q3_sol,q4_sol);