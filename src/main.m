% RBE3001 C18 Team 4: Hannah Baez, Alex Tacescu, Sam White

close all; clc; clear;

javaaddpath('../    lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

pp = PacketProcessor(7); % !FIXME why is the deviceID == 7?s

DEBUG   = false;          % enables/disables debug prints
PLOT    = true;          % enables/diables plotting
DATALOG = true;          % enables/disables data logging
degreesPerTics = 40/400;    %calibrates the degrees per encoder tic
                            %this is also in stickModel.m
                            
delete TCP.csv;
delete armEncoderValues.csv;
delete JointAngles.csv;
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
viaPts = zeros(3,6);
viaPts(1,:) = [ 800, 400,   0, -400,   0, 0]; %base joint
viaPts(2,:) = [ 800, 800,   800, 1600,   1600, 50]; %elbow joint
viaPts(3,:) = [ 800, 0, 800, 0, 800, 0]; %wrist joint


%{
%creates a full trajectory with set-points for each joint in a triangle
viaPts = zeros(3,90);
for u = 1:30
viaPts(1,u) = 0+800/30*u;
viaPts(2,u) = 0+800/30*u;
viaPts(3,u) = 0+800/30*u;
end
for u = 31:60
viaPts(1,u) = 1200-400/30*u;
viaPts(2,u) = 1600-800/30*u;
viaPts(3,u) = 1600-800/30*u;
end
for u = 61:90
viaPts(1,u) = 400-0/30*u;
viaPts(2,u) = 0+0/30*u;
viaPts(3,61:90) = viaPts(3,1:30);    
end
%}

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
    
    pause(0.001) %timeit(returnPacket) !FIXME why is this needed?
end

if DATALOG

    %writes the temporary matrix data to a .csv file
    csvwrite('armEncoderValues.csv',m);

    %writes a .csv file for just the arm angles
    Joint1Angles = m(:,1)*degreesPerTics.';
    Joint2Angles = m(:,4)*degreesPerTics.';
    Joint3Angles = m(:,7)*degreesPerTics.';
    dlmwrite('JointAngles.csv', time, '-append');
    dlmwrite('JointAngles.csv', Joint1Angles, '-append');
    dlmwrite('JointAngles.csv', Joint2Angles, '-append');
    dlmwrite('JointAngles.csv', Joint3Angles, '-append');

    if PLOT
        %plots the arm's joint angles over time
        figure('Position', [50, 50, 864, 864], 'Color', 'w');
        plot(time, Joint1Angles, 'r-*', time, Joint2Angles, 'b--O', time, Joint3Angles, 'g-.x', 'LineWidth', 2);
        title('RBE 3001 Lab 2: Joint Angles vs. Time');
        xlabel('Time (s)');
        ylabel('Joint Angle (degrees)');
        legend('Base joint', 'Elbow joint', 'Wrist joint');
        grid on;
    end

end

% Clear up memory upon termination
pp.shutdown()
clear java;

toc
