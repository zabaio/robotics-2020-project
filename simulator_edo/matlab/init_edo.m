clear all
close all

% Load robot object from URDF
robot_edo = importrobot('../urdf/edo_sim.urdf');
robot_edo_originale = importrobot('../urdf/edo_sim_originale.urdf');
robot_edo_nuove_inerzie = importrobot('../urdf/edo_sim_nuove_inerzie.urdf');

% Parameters (same as in URDF)
Lbx = 0.057188;
Lby = 0.0059831;
Lbz = 0.13343;

L1=0.18967;
L2=0.21050;
L3=0.1588;