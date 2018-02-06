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

DEBUG   = true;          % enables/disables debug prints
PLOT    = true;          % enables/diables plotting
DATALOG = true;          % enables/disables data logging
degreesPerTics = 360/4096;    %calibrates the degrees per encoder tic
                                                          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                            
delete armEncoderValues.csv;
delete JointAngles.csv;
delete JointVelocities.csv;
delete X-Y-Z-Position.csv;
delete JointAcceletations.csv;
delete TCP.csv;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create a PacketProcessor object to send data to the nucleo firmware
SERV_ID = 37;            % we will be talking to server ID 37 on
% the Nucleo

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Instantiate a packet - the following instruction allocates 64
% bytes for this purpose. Recall that the HID interface supports
% packet sizes up to 64 bytes.
packet = zeros(15, 1, 'single');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%takes a maxix of X-Y-Z set-points and uses inverse kinematics to produce
%a trajectory with variable data resolution
%X-Y-Z set-points:
p = [ 250, 175,  50,  250;  % X-axis poistion values
     -200, -50, 250, -200;  % Y-axis poistion values
      300, -50, 250,  300];  % Z-axis poistion values

%{
p = [300,   0,   0, 300;
       0, 300,   0,   0;
       0,   0, 470,   0];
%}
      
% linear interpolation between all set-points
P = linearInterpolation(p, 20, DEBUG);

% Can increase the number of identical points for greater data resolution when points are far apart.
% Converts x-y-z points (mm) to encoder values
viaPts = pointResolution(P, 1, degreesPerTics, DEBUG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initialize our temporary matrix to store data to be written to the .csv in
%a matrix the size of the number of setpoints by the number of returned
%data elements (15)
m = zeros(size(viaPts,2),15);
m(:,:) = 0;
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
        f1 = stickModel([m(k,1), m(k,4), m(k,7)]*degreesPerTics, degreesPerTics);
        %plots path
        if k > 1
            traceModel([m(k-1,1), m(k-1,4), m(k-1,7),m(k,1), m(k,4), m(k,7)]*degreesPerTics);
        end
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    pause(0.001) %timeit(returnPacket) !FIXME why is this needed?
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if DATALOG

    %writes the temporary encoder matrix data to a .csv file
    csvwrite('armEncoderValues.csv',m);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%   save and plot joint angles   %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%   save and plot joint velocities   %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for just the arm's joint VELOCITIES
    joint1Velocities = diff(m(:,1).'*degreesPerTics);
    joint2Velocities = diff(m(:,4).'*degreesPerTics);
    joint3Velocities = diff(m(:,7).'*degreesPerTics);
    dlmwrite('JointVelocities.csv', time, '-append');
    dlmwrite('JointVelocities.csv', joint1Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint2Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint3Velocities, '-append');

    if PLOT
        %plots the arm's joint velocities over time
        figure('Position', [864, 50, 864, 864]);
        plot(time(1,1:(size(time,2)-1)), joint1Velocities, 'r-*', time(1,1:(size(time,2)-1)), joint2Velocities, 'b--x', time(1,1:(size(time,2)-1)), joint3Velocities, 'g-.O', 'LineWidth', 2);
        title('RBE 3001 Lab 3: Joint Velocities vs. Time');
        xlabel('Time (s)');
        ylabel('Joint Velocity (degrees/s)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%   save and plot joint accelerations   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for just the arm's joint acceleration
    joint1Accelerations = diff(diff(m(:,1)).'*degreesPerTics);
    joint2Accelerations = diff(diff(m(:,4)).'*degreesPerTics);
    joint3Accelerations = diff(diff(m(:,7)).'*degreesPerTics);
    dlmwrite('JointAcclerations.csv', time, '-append');
    dlmwrite('JointAcclerations.csv', joint1Accelerations, '-append');
    dlmwrite('JointAcclerations.csv', joint2Accelerations, '-append');
    dlmwrite('JointAcclerations.csv', joint3Accelerations, '-append');

    if PLOT
        %plots the arm's joint acceleration over time
        figure('Position', [864, 50, 864, 864]);
        plot(time(1,1:(size(time,2)-2)), joint1Accelerations, 'r-*', time(1,1:(size(time,2)-2)), joint2Accelerations, 'b--x', time(1,1:(size(time,2)-2)), joint3Accelerations, 'g-.O', 'LineWidth', 2);
        title('RBE 3001 Lab 3: Joint Acclerations vs. Time');
        xlabel('Time (s)');
        ylabel('Joint Acceleration (degrees/s^2)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%   save and plot TCP x-y-z  position   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
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
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

% Clear up memory upon termination
pp.shutdown()
clear java;

toc