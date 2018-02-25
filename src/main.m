%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%   RBE3001 C18 Team 4: Hannah Baez, Alex Tacescu, Sam White  %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% take out the trash
close all; clc; clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% variable declarations

% enables/disables debug prints
DEBUG   = false;

%displays communication debug messages
DEBUG_COMS = false;

% enables/diables plotting
PLOT    = true;

% enables/disables data logging
DATALOG = true;

%calibrates the degrees per encoder tic
degreesPerTics = 360/4096;

%sets the lab number
lab = 4;

%sets axis parameters for live plot
axe = [-400, 400, -400, 400, -150, 650];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% deletes old data logging data
%original unchanged data from all nine channels
delete armDataValuesCopy.csv;                 
%has smoothed load cell values
delete armDataValues.csv;

delete X-Y-Z-Position.csv;
delete X-Y-Z-Velocity.csv;
delete X-Y-Z-Force.csv;
delete TCP.csv;
delete JointAngles.csv;
delete JointVelocities.csv;
delete JointAcceletations.csv;
delete .Jp.csv;
delete JointTorque.csv;
delete averageLoadCell.csv;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% trajectory generation and conversion to encoder tics

%takes a maxix of X-Y-Z set-points and uses inverse kinematics to produce
%a trajectory with variable data resolution
%X-Y-Z set-points:

p = [ 233.85,   269.38,    247, 275.74, 230.93;  % X-axis poistion values
     -110.82,  -109.65,  9.572, 120.34, 117.58;  % Y-axis poistion values
      377.33,  -2.7074, 386.96, 6.1789, 372.87];  % Z-axis poistion values

%P = [255;
%      50;
%     135];

%{
p = [355, 250;
    0, 15;
    135, 135];
%}
      
% Cubic Polynomial interpolation between all setpoints
P = cubicPoly(p, 25, 1, DEBUG);

% quintic Polynomial interpolation between all setpoints
%P = quinticPoly(p, 10, 3, DEBUG);
      
% linear interpolation between all set-points
%P = linearInterpolation(p, 1, DEBUG);

% Can increase the number of identical points for greater data resolution when points are far apart.
% Converts x-y-z points (mm) to encoder values
viaPts = pointResolution(P, 1, degreesPerTics, DEBUG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% commands movement

tic %starts an elapse timer

gripper = 1;

[m, copym, time] = moveArm (viaPts, gripper, degreesPerTics, axe, lab, DATALOG, PLOT, DEBUG_COMS, DEBUG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% data logging and post process plotting

if DATALOG

    %writes the temporary data matrix data to a .csv file
    csvwrite('armDataValues.csv',m);
    csvwrite('armDataValuesCopy.csv',copym);

    %handles data logging and plotting utilities
    postProcessing(m, copym, time, degreesPerTics, lab, PLOT, DEBUG);
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% shutdown procedure

% Clear up memory upon termination
clear java;

toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%