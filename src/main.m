% RBE3001 C18 Team 4: Hannah Baez, Alex Tacescu, Sam White

close all; clc; clear;

javaaddpath('../    lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

pp = PacketProcessor(7); % !FIXME why is the deviceID == 7?s

DEBUG   = true;          % enables/disables debug prints
PLOT    = true;          % enables/diables plotting
DATALOG = true;          % enables/disables data logging
degreesPerTics = 40/400;    %calibrates the degrees per encoder tic
                            %this is also in stickModel.m
                            
delete TCP.csv;
delete armEncoderValues.csv;
delete JointAngles.csv;
delete JointVelocities.csv;

%{
%Set up PID for the arm at the beginning of runtime
%Server ID, see SERVER_ID in PidConfigServer.h in Nucleo code
PID_SERVER_ID = 65;

%PID values for the arm
pidValues = [0.005, 0, 0, 1, 0;     %Base
             0.005, 0, 0, 1, 0;     %Shoulder
             0.005, 0, 0, 1, 0];    %Wrist

pidPacket = zeros(15, 1, 'single');
         
for a = 0:size(pidValues,2)-1
    
     %joint 1 packet
    pidPacket(a*3+1) = pidValues(1,a+1);
    
    %joint 2 packet
    pidPacket(a*3+2) = pidValues(2,a+1);
    
    %joint 3 packet
    pidPacket(a*3+3) = pidValues(3,a+1);
    
    % Send packet to the server
    returnPIDPacket = pp.command(PID_SERVER_ID, pidPacket)
    
end
%}
% Create a PacketProcessor object to send data to the nucleo firmware
SERV_ID = 37;            % we will be talking to server ID 37 on
% the Nucleo

% Instantiate a packet - the following instruction allocates 64
% bytes for this purpose. Recall that the HID interface supports
% packet sizes up to 64 bytes.
packet = zeros(15, 1, 'single');

% The following code generates a sinusoidal trajectory to be
% executed on joint 1 of the arm and iteratively sends the list of
% setpoints to the Nucleo firmware.

%viaPts = [0, -400, 0, 400, 0, -400, 0];


% the following is a null trajectory of five positions so that there will
% be five sets of arm data replyed to the status request
%viaPts = [0, 0, 0, 0, 0, 0];

%The following code generates a repeating trajectory for collecting step response
%data; the trajectory is between two points 45 degrees apart
%{
numRepeats = 2;
holdSize = 10;
numRows = numRepeats*holdSize*2;
viaPts(1:numRows) = 0;
for j = 1:holdSize*2:numRows
    disp(j)
    viaPts(1,j:j+holdSize) = 400;
end
%}


%creates a full trajectory with set-points for each joint
%viaPts = zeros(3,5);
%viaPts(1,:) = [0, 0, 0, 0, 0]; %base joint
%viaPts(2,:) = [839, 227, -8, 839, 839]; %elbow joint
%viaPts(3,:) = [-36, -191, -1218, -36, -36]; %wrist joint

%{
%creates a full trajectory with the same set-point for each joint with data points
holdSize = 10;
setPts = [0, 1000, 300, 700, 100, 800, 0]; %must be positive because the 
                            %elbow joint doesnt tollerate negative values
viaPts = zeros(3,size(setPts*holdSize,2));
                            
for k = 1:size(setPts,2)
    viaPts(:,holdSize*(k-1)+1:holdSize*k) = setPts(1,k);
end
%}


%creates a full trajectory with set-points for each joint in a triangle
%this is the hard-coded output of the cubicPoly function that we had
%trouble with, but this does a good job of linear interpolation
viaPts = zeros(3,40);
holdSize = 10;
counter = 0;
for u = holdSize*0+1:holdSize*1
viaPts(1,u) = 0;
viaPts(2,u) = 839-(839-227)/holdSize*counter;
viaPts(3,u) = -36-(-36--191)/holdSize*counter;
counter = counter + 1;
end
counter = 0;
for u = holdSize*1+1:holdSize*2
viaPts(1,u) = 0;
viaPts(2,u) = 227-(227--8)/holdSize*counter;
viaPts(3,u) = -191-(-191--36)/holdSize*counter;
counter = counter + 1;
end
counter = 0;
for u = holdSize*2+1:holdSize*3
viaPts(1,u) = 0;
viaPts(2,u) = -8-(-8-839)/holdSize*counter;
viaPts(3,u) = -36-(-36--1218)/holdSize*counter; 
counter = counter + 1;
end
for u = holdSize*3+1:holdSize*4
viaPts(1,u) = 0;
viaPts(2,u) = 839-(839-839)/holdSize*counter;
viaPts(3,u) = -1218-(-1218--1218)/holdSize*counter; 
counter = counter + 1;
end
%viaPts(2,41) = 839;
%viaPts(3,41) = -36;



%displays the set-points matrix
if DEBUG
    viaPts
end

%initialize our temporary matrix to store data to be written to the .csv in
%a matrix the size of the number of setpoints by the number of returned
%data elements (15)
m = zeros(size(viaPts,2),15);
m(:,:) = 0;
time = zeros(1, size(viaPts,2));

tic %starts an elapse timer

% Iterate through commands for joint values
%size(matrix_name, 1 (rows) or 2 (columns))
for k = 1:size(viaPts,2)
    %incremtal = (single(k) / sinWaveInc);
    
    %joint 1 set-point packet
    packet(1) = viaPts(1,k);
    
    %joint 2 set-point packet
    packet(4) = viaPts(2,k);
    
    %joint 3 set-point packet
    packet(7) = viaPts(3,k);
    
    
    %Send packet to the server and get the response
    returnPacket = pp.command(SERV_ID, packet);
    
    toc %displays the elapsed time since tic
    
    if DATALOG
        %records the elapsed time since tic
        time(1,k) = toc; 

        %adds the returned data to the temporary matrix as a row instead of a
        %column (list)
        m(k,:) = returnPacket;
    end
    
    if DEBUG
        disp('Sent Packet:');
        disp(packet);
        disp('Received Packet:');
        disp(returnPacket);
    end
    
    %plots a stick model with green spheres for joints, thick blue lines for links,
    %and a thin red line for path
    if PLOT
        %plots links andjoints
        f1 = stickModel([m(k,1), m(k,4), m(k,7)]*degreesPerTics);
        %plots path
        if k > 1
            traceModel([m(k-1,1), m(k-1,4), m(k-1,7),m(k,1), m(k,4), m(k,7)]*degreesPerTics);
        end
    end
    
    pause(0.01) %timeit(returnPacket) !FIXME why is this needed?
end

if DATALOG

    %writes the temporary matrix data to a .csv file
    csvwrite('armEncoderValues.csv',m);

    %writes a .csv file for just the arm's joint angles
    Joint1Angles = m(:,1).'*degreesPerTics;
    Joint2Angles = m(:,4).'*degreesPerTics;
    Joint3Angles = m(:,7).'*degreesPerTics;
    dlmwrite('JointAngles.csv', time, '-append');
    dlmwrite('JointAngles.csv', Joint1Angles, '-append');
    dlmwrite('JointAngles.csv', Joint2Angles, '-append');
    dlmwrite('JointAngles.csv', Joint3Angles, '-append');
    
    if PLOT
        %plots the arm's joint angles over time
        figure('Position', [0, 50, 864, 864]);
        plot(time, Joint1Angles, 'r-*', time, Joint2Angles, 'b--x', time, Joint3Angles, 'g-.O', 'LineWidth', 2);
        title('RBE 3001 Lab 2: Joint Angles vs. Time');
        xlabel('Time (s)');
        ylabel('Joint Angle (degrees)');
        legend('Base joint', 'Elbow joint', 'Wrist joint');
        grid on;
        
    end
    
    %writes a .csv file for just the arm's joint velocities
    Joint1Velocities = diff(m(:,1).'*degreesPerTics);
    Joint2Velocities = diff(m(:,4).'*degreesPerTics);
    Joint3Velocities = diff(m(:,7).'*degreesPerTics);
    dlmwrite('JointVelocities.csv', time, '-append');
    dlmwrite('JointVelocities.csv', Joint1Velocities, '-append');
    dlmwrite('JointVelocities.csv', Joint2Velocities, '-append');
    dlmwrite('JointVelocities.csv', Joint3Velocities, '-append');

    if PLOT
        %plots the arm's joint velocities over time
        figure('Position', [864, 50, 864, 864]);
        plot(time(1,1:(size(time,2)-1)), Joint1Velocities, 'r-*', time(1,1:(size(time,2)-1)), Joint2Velocities, 'b--x', time(1,1:(size(time,2)-1)), Joint3Velocities, 'g-.O', 'LineWidth', 2);
        title('RBE 3001 Lab 2: Joint Velocities vs. Time');
        xlabel('Time (s)');
        ylabel('Joint Velocity (mm/s)');
        legend('Base joint', 'Elbow joint', 'Wrist joint');
        grid on;
        
    end
    
end

% Clear up memory upon termination
pp.shutdown()
clear java;

toc
