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
degreesPerTics = 360/4096;    %calibrates the degrees per encoder tic
                              %this is also in stickModel.m
                            
delete armEncoderValues.csv;
delete JointAngles.csv;
delete JointVelocities.csv;
delete X-Y-Z-Position.csv;

% Create a PacketProcessor object to send data to the nucleo firmware
SERV_ID = 37;            % we will be talking to server ID 37 on
% the Nucleo

% Instantiate a packet - the following instruction allocates 64
% bytes for this purpose. Recall that the HID interface supports
% packet sizes up to 64 bytes.
packet = zeros(15, 1, 'single');

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

%{
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
%}

%takes a maxix of X-Y-Z set-points and uses inverse kinematics to produce
%a trajectory with variable data resolution
%X-Y-Z set-points:
p = [355,   300, 250, 200;  % X-axis poistion values
       0,     0,   0,   0;  % Y-axis poistion values
     135,   135, 135, 135];  % Z-axis poistion values

%{
p = [300,   0,   0, 300;
       0, 300,   0,   0;
       0,   0, 470,   0];
%}
%set data resultion (number of data points per set-point)
 holdSize = 10;

 %builds trajectory using inverse kinimatics
viaPts = zeros(3,holdSize*size(p,2));
for k = 1:size(p,2)
    viaPt = zeros(3,1);
    viaPt = ikin3001(p(:,k), DEBUG)
    viaPt = viaPt/degreesPerTics;
    for j = 1:holdSize
        viaPts(:,(j+(k-1)*holdSize)) = viaPt(:,:)
    end
end

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
        f1 = stickModel([m(k,1), m(k,4), m(k,7)]*degreesPerTics, degreesPerTics);
        %plots path
        if k > 1
            traceModel([m(k-1,1), m(k-1,4), m(k-1,7),m(k,1), m(k,4), m(k,7)]*degreesPerTics);
        end
    end
    
    pause(0.2) %timeit(returnPacket) !FIXME why is this needed?
end

if DATALOG

    %writes the temporary encoder matrix data to a .csv file
    csvwrite('armEncoderValues.csv',m);

    %writes a .csv file for just the arm's joint angles
    joint1Angles = m(:,1).'*degreesPerTics;
    joint2Angles = m(:,4).'*degreesPerTics;
    joint3Angles = m(:,7).'*degreesPerTics;
    dlmwrite('JointAngles.csv', time, '-append');
    dlmwrite('JointAngles.csv', joint1Angles, '-append');
    dlmwrite('JointAngles.csv', joint2Angles, '-append');
    dlmwrite('JointAngles.csv', joint3Angles, '-append');
    
    if PLOT
        %plots the arm's joint angles over time
        figure('Position', [0, 50, 864, 864]);
        plot(time, joint1Angles, 'r-*', time, joint2Angles, 'b--x', time, joint3Angles, 'g-.O', 'LineWidth', 2);
        title('RBE 3001 Lab 3: Joint Angles vs. Time');
        xlabel('Time (s)');
        ylabel('Joint Angle (degrees)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;       
    end
    
    %writes a .csv file for just the arm's joint velocities
    joint1Velocities = diff(m(:,1).'*degreesPerTics);
    joint2Velocities = diff(m(:,4).'*degreesPerTics);
    joint3Velocities = diff(m(:,7).'*degreesPerTics);
    dlmwrite('JointVelocities.csv', time, '-append');
    dlmwrite('JointVelocities.csv', joint1Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint2Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint3Velocities, '-append');

    if false %PLOT
        %plots the arm's joint velocities over time
        figure('Position', [864, 50, 864, 864]);
        plot(time(1,1:(size(time,2)-1)), joint1Velocities, 'r-*', time(1,1:(size(time,2)-1)), joint2Velocities, 'b--x', time(1,1:(size(time,2)-1)), joint3Velocities, 'g-.O', 'LineWidth', 2);
        title('RBE 3001 Lab 2: Joint Velocities vs. Time');
        xlabel('Time (s)');
        ylabel('Joint Velocity (degrees/s)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;
    end
    
    %writes a .csv file for the X-Y-Z position of the TCP
    Position = zeros(size(m,1),3);
    for k = 1:size(m,1)
        Position(k,1:3) = fwkin3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)],DEBUG).';
    end
    xPosition = Position(:,1).';
    yPosition = Position(:,2).';
    zPosition = Position(:,3).';
    dlmwrite('X-Y-Z-Position.csv', time, '-append');
    dlmwrite('X-Y-Z-Position.csv', xPosition, '-append');
    dlmwrite('X-Y-Z-Position.csv', yPosition, '-append');
    dlmwrite('X-Y-Z-Position.csv', zPosition, '-append');
    
    if PLOT
        %plots the X-Y-Z Position of the TCP over time
        figure('Position', [864, 50, 864, 864]);
        plot(time, xPosition, 'r-*', time, yPosition, 'b--x', time, zPosition, 'g-.O', 'LineWidth', 2);
        title('RBE 3001 Lab 3: X-Y-Z Position of the TCP  vs. Time');
        xlabel('Time (s)');
        ylabel('Position of the TCP (mm)');
        legend('X Position', 'Y Position', 'Z Position');
        grid on;
    end
    
    if DEBUG
        disp('Position: X, Y, Z');
        disp(Position);
    end
    
end

% Clear up memory upon termination
pp.shutdown()
clear java;

toc
