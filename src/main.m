%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%   RBE3001 C18 Team 4: Hannah Baez, Alex Tacescu, Sam White  %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clc; clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

javaaddpath('../    lib/hid4java-0.5.1.jar');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pp = PacketProcessor(7); % deviceID == 7

DEBUG   = false;          % enables/disables debug prints
PLOT    = true;          % enables/diables plotting
DATALOG = true;          % enables/disables data logging
degreesPerTics = 360/4096;    %calibrates the degrees per encoder tic
lab = 4;                  %sets the lab number
axe = [-400, 400, -400, 400, -150, 650]; %sets axis parameters for live plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

% Create a PacketProcessor object to send data to the nucleo firmware
SERV_ID = 30;            % we will be talking to server ID 30, LabServer
% the Nucleo

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Instantiate a packet - the following instruction allocates 64
% bytes for this purpose. Recall that the HID interface supports
% packet sizes up to 64 bytes.
packet = zeros(15, 1, 'single');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%initialize our temporary matrix to store data to be written to the .csv in
%a matrix the size of the number of setpoints by the number of returned
%data elements (15)
m = zeros(size(viaPts,2),15);
m(:,:) = 0;
copym = m;
time = zeros(1, size(viaPts,2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%displays a large mark to offset pre-comms debug information
if DEBUG
    disp('#######################################################################################################################################');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic %starts an elapse timer

% Iterate through commands for joint values
%size(matrix_name, 1 (rows) or 2 (columns))
for k = 1:size(viaPts,2)
    
    %joint 1 set-point packet
    packet(1) = viaPts(1,k);
    
    %joint 2 set-point packet
    packet(4) = viaPts(2,k);
    
    %joint 3 set-point packet
    packet(7) = viaPts(3,k);
    
    %actuates the gripper: 1 opens gripper and 0 closes gripper
    packet(10) = 1;
    
    
    %Send packet to the server and get the response
    returnPacket = pp.command(SERV_ID, packet);
    
    toc %displays the elapsed time since tic

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if DATALOG
        %records the elapsed time since tic
        time(1,k) = toc; 

        %adds the returned data to the temporary matrix as a row instead of a
        %column (list)
        m(k,:) = returnPacket;
        copym(k,:) = returnPacket;

        %sets number of past data points to use in running average
        n = 3;
        %smoothes the load cell data incrimentally
        if k > n
            for i = 3*(1:3)
                m(k-n:k,i) = dataSmooth(m(k-n:k,i), n, lab, false, DEBUG);
            end
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if DEBUG
        disp('Sent Packet:');
        disp(packet);
        disp('Received Packet:');
        disp(returnPacket);
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %plots a stick model with green spheres for joints, thick blue lines for links,
    %and a thin red line for path
    if PLOT
        %plots links andjoints
        f1 = stickModel([m(k,1), m(k,4), m(k,7)]*degreesPerTics, degreesPerTics, lab, axe);
        %plots path
        if k > 1
            traceModel([m(k-1,1), m(k-1,4), m(k-1,7),m(k,1), m(k,4), m(k,7)]*degreesPerTics, lab, axe);
        end
        
        %instantaneous joint angles
        instJointAngles = [m(k,1); m(k,4); m(k,7)]*degreesPerTics;
        
        %instantaneous torques
        instJointTorque(1,1) = ADCToTorque(m(k,3),1,DEBUG).';
        instJointTorque(2,1) = ADCToTorque(m(k,6),2,DEBUG).';
        instJointTorque(3,1) = ADCToTorque(m(k,9),3,DEBUG).';
        instJointTorques = statics3001(instJointAngles, instJointTorque, DEBUG);
        
        %draws vector of force on end effector
        quiverModel(instJointAngles, instJointTorques, norm(instJointTorques)*100, axe, true, true);
        %draws vector of velocity of end effector
        %quiverModel([m(k,1); m(k,4); m(k,7)]*degreesPerTics, [m(k,2); m(k,5); m(k,8)]*degreesPerTics, 0.025, false, DEBUG);
        
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    pause(0.001) %timeit(returnPacket) !FIXME why is this needed?
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if DATALOG

    %writes the temporary data matrix data to a .csv file
    csvwrite('armDataValues.csv',m);
    csvwrite('armDataValuesCopy.csv',copym);

    %handles data logging and plotting utilities
    postProcessing(m, copym, time, degreesPerTics, lab, PLOT, DEBUG);
    
end

% Clear up memory upon termination
pp.shutdown()
clear java;

toc