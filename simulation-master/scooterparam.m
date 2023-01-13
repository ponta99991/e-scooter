clear all
clc

Simulink.sdi.clear                                                          %Clear simulink data inspector

scooter_DataFile;                                                           %Scooter translation and rotation from CAD

scootermodelsim;

%% Scooter parameters

r_wheel = 0.11;                                                             %Radius of the wheel [m]
h = 0.27;                                                                   %Height of center of mass [m]
b = 0.835;                                                                  %Length between wheel centers [m]
a = 0.515;                                                                  %Distance from rear wheel to frame's center of mass [m]

%% Additional parameters

PIDswitch=0;                                                                %0 for 1 and 1 for 2 PIDs
Pidopton = 0;                                                               %0 = normal use, 1 = PID settling time optimization results

%Variables for TF H(s) IEEE 9655223, not used
%zeta = 0.6;                                                                %Constant found via testing (IEEE 9655223)
%d = 0.015;                                                                 %Time delay [s]

v = 8;                                                                      %Velocity [km/h] (model assumes constant velocity)

%Different sample rates
Ts=0.01;                                                                    %Outer PID [s]
Tsm=Ts/6;                                                                   %Inner PID [s]

simulationtime=8;                                                           %Scooter simulation time (Simulink)

%PID settings if running once (not using PID optimization calculation)
outer_p = 4;                                                                %Outer
outer_i = 0.1;
outer_d = 0.1;
inner_p = 5;                                                                %Inner
inner_i = 0;
inner_d = 1;

N = 100;                                                                    %Filter coefficient, always constant


%% Test parameters

%Disturbance
pushamp=5;                                                                  %Push moment [Nm]
phase=2;                                                                    %Time for push [s]

%Standard friction values on back and front wheel in static and dynamic
%states, find these in contactpoints block.
Friction_stat_front=0.8;
Friction_dyn_front=0.7;
Friction_stat_back=0.8;
Friction_dyn_back=0.7;

%The revolute joints damping coefficient and spring stiffness is available both for back and front wheel
damping_front=0.1;                                                          %[N*m/(deg/s)]
spring_front=0;                                                             %[N*m/deg]

damping_back=0.1;                                                           %[N*m/(deg/s)]
spring_back=0;                                                              %[N*m/deg]



%% Plane parameters

xPla = 80;                                                                  %x plane length [m]
yPla = 40;                                                                  %y plane length [m]
zPla = 0.01;                                                                %z plane depth [m]

%For MATLAB interpretation
surfacex = 0:1:xPla;
surfacey = 0:1:yPla;

%% Do not change below paramters without motivation

%Rotate the frames such Z is up also sets gravity in z
rotationaxis = [1 0 0];
rotationangle = pi/2;
gravity = 9.82;                                                             %gravity [m/s^2]

initvelocity = v/(r_wheel*3.6);                                             %[rad/s] (w = v/r), 3.6 is for km/h to m/s conversion

%The scooter is not aligned at plane height such 20 cm dropheight is used
%to have it over, future work to allign them better
dropheight = r_wheel + 0.14;                                                %[m]

%smlink_linksw                                                              %README before calling this (only if a model needs to be updated)
%smimport('scootermodel');                                                  %README before calling this (only if a model needs to be updated)

%% Other if not using GUI

%Run the simulation
% set_param('scootermodelsim','SimMechanicsOpenEditorOnUpdate','off');
% simOut = sim("scootermodelsim");
    
%PID_optimisation

%% TF

sysG = tf((a*(v/3.6)/(b*h))*[1,(v/3.6)/a],[1,0,gravity/h]);                 %G(s) bicycle model (Lund/Niklas)

%sysG = tf([0.05 6.2],[0.5 4.1 -5.8]);                                       %Alternative model
