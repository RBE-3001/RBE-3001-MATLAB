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
delete detJp.csv;
delete JointTorque.csv;

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

%p = [ 233.85,   269.38,    247, 275.74, 230.93;  % X-axis poistion values
%     -110.82,  -109.65,  9.572, 120.34, 117.58;  % Y-axis poistion values
%      377.33,  -2.7074, 386.96, 6.1789, 372.87];  % Z-axis poistion values

P = [355;
       0;
     135];

%{
p = [355, 250;
    0, 15;
    135, 135];
%}
      
% Cubic Polynomial interpolation between all setpoints
%P = cubicPoly(p, 1, 1, DEBUG);

% quintic Polynomial interpolation between all setpoints
%P = quinticPoly(p, 10, 3, DEBUG);
      
% linear interpolation between all set-points
%P = linearInterpolation(p, 1, DEBUG);

% Can increase the number of identical points for greater data resolution when points are far apart.
% Converts x-y-z points (mm) to encoder values
viaPts = pointResolution(P, 100, degreesPerTics, DEBUG);

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
        quiverModel(instJointAngles, instJointTorques, norm(instJointTorques), axe, true, true);
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
    
    if false
        %plots the arm's joint angles over time
        figure('Position', [0, 50, 864, 864]);
        plot(time, joint1Angles, 'r-*', time, joint2Angles, 'b--x', time, joint3Angles, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Angles vs. Time', lab));
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
    timeV = time(1,1:(size(time,2)-1));
    dlmwrite('JointVelocities.csv', timeV, '-append');
    dlmwrite('JointVelocities.csv', joint1Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint2Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint3Velocities, '-append');

    if false
        %plots the arm's joint velocities over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeV, joint1Velocities, 'r-*', timeV, joint2Velocities, 'b--x', timeV, joint3Velocities, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Velocities vs. Time', lab));
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
    timeA = time(1,1:(size(time,2)-2));
    dlmwrite('JointAcclerations.csv', timeA, '-append');
    dlmwrite('JointAcclerations.csv', joint1Accelerations, '-append');
    dlmwrite('JointAcclerations.csv', joint2Accelerations, '-append');
    dlmwrite('JointAcclerations.csv', joint3Accelerations, '-append');

    if false
        %plots the arm's joint acceleration over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeA, joint1Accelerations, 'r-*', timeA, joint2Accelerations, 'b--x', timeA, joint3Accelerations, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Acclerations vs. Time', lab));
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
        Position(k,1:3) = fwkin3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], true, DEBUG).';
    end
    
    xPosition = Position(:,1).';
    yPosition = Position(:,2).';
    zPosition = Position(:,3).';
    dlmwrite('X-Y-Z-Position.csv', time, '-append');
    dlmwrite('X-Y-Z-Position.csv', xPosition, '-append');
    dlmwrite('X-Y-Z-Position.csv', yPosition, '-append');
    dlmwrite('X-Y-Z-Position.csv', zPosition, '-append');
    
    if false
        %plots the X-Y-Z Position of the TCP over time
        figure('Position', [864, 50, 864, 864]);
        plot(time, xPosition, 'r-*', time, yPosition, 'b--x', time, zPosition, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: X-Y-Z Position of the TCP  vs. Time', lab));
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
%%%%%%%%%%%%%%%%    save and plot determinant of Jp    %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
%writes a .csv file for the determinant of Jp
    detJp = zeros(size(m,1),1);
    for k = 1:size(m,1)
        J = jacob0([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], DEBUG);
        detJp(k,1) = det(J(1:3,:));
    end
    
    det = detJp(:,1).';

    dlmwrite('detJp.csv', time, '-append');
    dlmwrite('detJp.csv', det, '-append');
    
    if PLOT
        %plots the determinant of Jp over time
        figure('Position', [50, 50, 864, 864]);
        plot(time, det, 'r-*', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Determinant of Jacobian Position Matrix (Jp) vs. Time',lab));
        xlabel('Time (s)');
        ylabel('Determinant of Jp');
        legend('Determinant of Jp');
        grid on;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%   save and plot TCP x-y-z  velocity   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for the X-Y-Z velocity of the TCP using position
    Velocity = zeros(size(m,1),3);
    for k = 1:size(m,1)-1
        Velocity(k,1:3) = fwddiffkin3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], [joint1Velocities(1,k); joint2Velocities(1,k); joint3Velocities(1,k)], DEBUG).';
    end
    
    xVelocity = Velocity(1:(size(Velocity,1)-1),1).';
    yVelocity = Velocity(1:(size(Velocity,1)-1),2).';
    zVelocity = Velocity(1:(size(Velocity,1)-1),3).';
    dlmwrite('X-Y-Z-Velocity.csv', timeV, '-append');
    dlmwrite('X-Y-Z-Velocity.csv', xVelocity, '-append');
    dlmwrite('X-Y-Z-Velocity.csv', yVelocity, '-append');
    dlmwrite('X-Y-Z-Velocity.csv', zVelocity, '-append');
    
    if false
        %plots the X-Y-Z Velocity of the TCP over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeV, xVelocity, 'r-*', timeV, yVelocity, 'b--x', timeV, zVelocity, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: X-Y-Z Velocity of the TCP  vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Velocity of the TCP (mm/s)');
        legend('X Velocity', 'Y Velocity', 'Z Velocity');
        grid on;
    end
    
    if DEBUG  
        disp('Velocity: X, Y, Z');
        disp(Velocity);
    end  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%   save and plot joint toques   %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %writes a .csv file for just the arm's joint angles
    joint1Torques = ADCToTorque(m(:,3),1,true).';
    joint2Torques = ADCToTorque(m(:,6),2,true).';
    joint3Torques = ADCToTorque(m(:,9),3,true).';
    dlmwrite('JointTorque.csv', time, '-append');
    dlmwrite('JointTorque.csv', joint1Torques, '-append');
    dlmwrite('JointTorque.csv', joint2Torques, '-append');
    dlmwrite('JointTorque.csv', joint3Torques, '-append');
    
    if PLOT
        %plots the arm's joint angles over time
        figure('Position', [0, 50, 864, 864]);
        plot(time, joint1Torques, 'r-*', time, joint2Torques, 'b--x', time, joint3Torques, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Torque vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Joint Torque (Nm)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;       
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%   save and plot TCP x-y-z  Forces   %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for the X-Y-Z forces of the TCP using position and
    %joint torques
    Force = zeros(size(m,1),3);
    for k = 1:size(m,1)-1
        Force(k,1:3) = statics3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], [joint1Torques(1,k); joint2Torques(1,k); joint3Torques(1,k)], DEBUG).';
    end
    
    xForce = Force(1:size(Force,1)-1,1).';
    yForce = Force(1:size(Force,1)-1,2).';
    zForce = Force(1:size(Force,1)-1,3).';
    dlmwrite('X-Y-Z-Force.csv', timeV, '-append');
    dlmwrite('X-Y-Z-Force.csv', xForce, '-append');
    dlmwrite('X-Y-Z-Force.csv', yForce, '-append');
    dlmwrite('X-Y-Z-Force.csv', zForce, '-append');
    
    if PLOT
        %plots the X-Y-Z Force of the TCP over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeV, xForce, 'r-*', timeV, yForce, 'b--x', timeV, zForce, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: X-Y-Z Force of the TCP  vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Force of the TCP (N)');
        legend('X Force', 'Y Force', 'Z Force');
        grid on;
    end
    
    if DEBUG  
        disp('Force: X, Y, Z');
        disp(Force);
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

% Clear up memory upon termination
pp.shutdown()
clear java;

toc