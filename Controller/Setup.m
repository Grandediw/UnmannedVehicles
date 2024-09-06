%% Lezione 20/03/2024 - Control scheme of a Quadcopter
% Develop a controller for NL system
% Simulation based on Linearized the system
clc 
clear all
close all
%%
format compact
format short g
Ts = 0.1; %Sample time for output varaiables

%% Drone parameters from Ofodile turner European Journal of control 2017
%  From table 1
m = 2.1; % kg
g = 9.81;
J = diag([2.85e-6;2.85e-6;1.81e-6])
k1 = 0.69; % Force constant
k2 = 0.11; % Torque constant
rho = 0.3; % Distance between COG and propellers

% From equation 6.5
MMA = [k1 k1 k1 k1;
    0 -rho*k1 0 rho*k1
    rho*k1 0 -rho*k1 0
    -k2 k2 -k2 k2];

% Non-Linear States
p0 = [1;2;3];
v0 = [0;0;0];
q0 = [1;0;0;0];
q0= q0/norm(q0)
omega0 = [0;0;0];

x0 = [p0;v0;q0;omega0];

% Linearized State

S0 = [0 -q0(4) -q0(3);
    q0(4) 0 -q0(2);
    q0(3) q0(2) 0];
R0 = eye(3)+2*q0(1)*S0+2*S0*S0;
alpha0 = rotm2eul(R0)'; 

% Equilibrium
psi_eq = 50*pi/100;
peq = p0;
veq = [0;0;0];  % Velocities at equilibrium must alway be zero
alphaeq = [0;0;psi_eq];
omegaeq = [0;0;0];
fceq = m*g;

xeq = [p0;veq;alphaeq;omegaeq]

xlin0 = [p0;v0;alpha0;omega0];


