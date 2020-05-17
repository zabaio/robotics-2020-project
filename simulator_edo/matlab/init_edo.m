clear all
close all

% Load robot object from URDF
robot_edo = importrobot('../urdf/edo_sim.urdf');

% Parameters (same as in URDF)
L1 = 0.337; % Arm length [m]
L2 = 0.210; % Forearm length [m]
L3 = 0;
L4 = 0.268;
L5 = 0;
L6 = 0.174;