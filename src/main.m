%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%   RBE3001 C18 Team 4: Hannah Baez, Alex Tacescu, Sam White  %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% take out the trash
close all; clc; clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% variable declarations

DEBUG   = false;                         %enables/disables debug prints
DEBUG_COMS = false;                      %displays communication debug messages
PLOT    = true;                          %enables/diables plotting
DATALOG = true;                          %enables/disables data logging
degreesPerTics = 360/4096;               %calibrates the degrees per encoder tic
lab = 5;                                 %sets the lab number
axe = [-400, 400, -400, 400, -150, 650]; %sets axis parameters for live plot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% deletes old data logging data

delete armDataValuesCopy.csv; %original unchanged data from all nine channels
delete armDataValues.csv;     %has smoothed load cell values
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
%% trajectory generation 

%input points that will become a trajectory by using inverse kinematics to
%produce a trajectory with variable data resolution
desiredPoints = [ 233.85,   269.38,    247, 275.74, 230.93;  % X-axis poistion values
                 -110.82,  -109.65,  9.572, 120.34, 117.58;  % Y-axis poistion values
                  377.33,  -2.7074, 386.96, 6.1789, 372.87];  % Z-axis poistion values

%selects between linear, cubic, and quintic polynomial point interpolation,
%1 = linear, 3 = cubic, 5 = quintic
interMode = 5;

%sets the number of points between two points for tracjectory interpolation
interPoints = 25;

%sets the duration (s) between two points for cubic/quintic intermpolation
nonLinearInterDuration = 3;

%sets the number of points to be reapeated for better data resolution
res = 1;

encoderTrajectory = buildTrajectory( desiredPoints, interMode, interPoints, nonLinearInterDuration, res, degreesPerTics, DEBUG);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% commands movement

tic %starts an elapse timer

gripper = 1;

[m, copym, time] = moveArm (encoderTrajectory, gripper, degreesPerTics, axe, lab, DATALOG, PLOT, DEBUG_COMS, DEBUG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% data logging and post process plotting

if DATALOG

    %writes the temporary data matrix data to a .csv file
    csvwrite('armDataValues.csv',m);
    csvwrite('armDataValuesCopy.csv',copym);

    %handles data logging and plotting utilities
    postProcessing(m, copym, time, degreesPerTics, lab, PLOT, DEBUG);
    
end

%displays end time of operation
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%