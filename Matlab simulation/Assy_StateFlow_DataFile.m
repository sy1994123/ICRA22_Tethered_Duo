%SoftParameters for SoftRobotics SimMechanics library
%All values in SI units.
clc;clear;
%% Geometrical parameters of elements
w = 0.015;        %side width of soft element   
wg = 5;         %ground width
la = 0.001;      %width of actuation element
wa = 0.001;       %length of actuation element
ha = 0.001;      %heigth of actuation element
load('traj.mat')
%% Initial conditions
hi = 2;       %initial height of soft element
hia = 2;        %initial height of actuation element
ai = 20;        %initial angle of soft element in degrees around x-axis
aia = 0;        %initial angle of actuation element in degrees around x-axis

%% Ground contact parameters
ll = 1e-4;      %smallest ground-body distance allowed for force calculation
lu = 1e5;       %largest ground-body distance allowed for force calculation
gg = 100;       %ground approach gain
id = -20;       %dissipation factor for plastic impact
fx = -0.11;      %friction factor in X
fy = -0.11;      %friction factor in Y

%% Tendon force parameters
TF = 300;       %Tendon force amplitude
Tf = 3.14;      %Tendon force frequency

%% Stiffness and damping coefficients
Kt = 4.5e+05;       %translational spring stiffness
Dt = 47;           %translational damping
Kb = 30;           %spherical spring stiffness
Db = 1.1;           %spherical damping

