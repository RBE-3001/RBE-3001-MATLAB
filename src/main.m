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
%% deletes old .csv files
cleanCSV();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% trajectory generation 

%input points that will become a trajectory by using inverse kinematics to
%produce a trajectory with variable data resolution
desiredPoints = [ 233.85,   269.38,    247, 275.74, 230.93;  % X-axis poistion values
                 -110.82,  -109.65,  9.572, 120.34, 117.58;  % Y-axis poistion values
                  377.33,  -2.7074, 386.96, 6.1789, 372.87];  % Z-axis poistion values

%interpolation mode selection: 1 = linear, 3 = cubic, 5 = quintic
interMode = 5;

%number of points between interpolations
interPoints = 25;

%duration (s) between intermpolations
nonLinearInterDuration = 3;

%data resolution
res = 1;

encoderTrajectory = buildTrajectory( desiredPoints, interMode, interPoints, nonLinearInterDuration, res, degreesPerTics, DEBUG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% commands movement

tic %starts an elapse timer

gripper = 1;

[m, copym, time] = moveArm (encoderTrajectory, gripper, degreesPerTics, axe, lab, DATALOG, PLOT, DEBUG_COMS, DEBUG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% data logging and post process plotting

    %handles data logging and plotting utilities
    postProcessing(m, copym, time, degreesPerTics, lab, DATALOG, PLOT, DEBUG);

%displays end time of operation
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%