%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

clr;

javaaddpath('../    lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(7); % !FIXME why is the deviceID == 7?
SERV_ID = 30;            % we will be talking to server ID 37 on
                         % the Nucleo

DEBUG   = true;          % enables/disables debug prints

% Instantiate a packet - the following instruction allocates 64
% bytes for this purpose. Recall that the HID interface supports
% packet sizes up to 64 bytes.
packet = zeros(15, 1, 'single');

% The following code generates a sinusoidal trajectory to be
% executed on joint 1 of the arm and iteratively sends the list of
% setpoints to the Nucleo firmware. 

%viaPts = [0, -400, 0, 400, 0, -400, 0];

numRows = 40;
holdSize = 10;
viaPts(1:numRows) = 0;
for j = 1:holdSize*2:numRows
    disp(j)
    viaPts(1,j:j+holdSize) = 400;
end

% the following is a null trajectory of five positions so that there will
% be five sets of arm data replyed to the status request
%viaPts = [0, 0, 0, 0, 0, 0];

%initialize our temporary matrix to store data to be written to the .csv in
%a matrix the size of the number of setpoints by the number of returned
%data elements (15)
m = zeros(size(viaPts,2),15);
m(:,:) = 1;
counter = 0;
time = zeros(1, numRows);
tic % What does this do? --> starts an elapse timer

% Iterate through a sine wave for joint values
for k = viaPts
    %incremtal = (single(k) / sinWaveInc);
    
    packet(1) = k;
    
    
    % Send packet to the server and get the response
    returnPacket = pp.command(SERV_ID, packet);
    
    %records the elapsed time since tic
    time(1,counter+1) = toc;
    
    %displays the elapsed time since tic
    toc
    
    %adds the returned data to the temporary matrix as a row instead of a
    %column (list)
    m(counter+1,:) = returnPacket;
    counter = counter + 1;
    
    if DEBUG
        disp('Sent Packet:');
        disp(packet);
        disp('Received Packet:');
        disp(returnPacket);
    end
    
    pause(0.01) %timeit(returnPacket) !FIXME why is this needed?
end

%writes the temporary matrix data to a .csv file
csvwrite('armData.csv',m);

%displays the data inside the .csv file
if DEBUG
    %reads .csv file and stores contents in a temporary matrix
    M = csvread('armData.csv');
    %displays the matrix of the data in the .csv file
    disp('Matlab wrote:');
    disp(M);
end

%writes a .csv file for just the arm angle
degreesPerTics = 45/400;
baseJointAngles = m(:,1)*degreesPerTics.';
csvwrite('baseJointAngle.csv', time);
csvwrite('baseJointAngle.csv', baseJointAngles);

%plots the base joint angle over time
figure('Position', [50, 50, 864, 864], 'Color', 'w');
title('RBE 3001 Lab 1: Base Joint Angle vs. Time');
xlabel('Time (s)');
ylabel('Base Joint Angle (degrees)');
grid on;
plot(time,baseJointAngles,'r-x')

%displays the data inside the .csv file
if DEBUG
    %reads .csv file and stores contents in a temporary Array
    MM = csvread('baseJointAngle.csv');
    %displays the matrix of the data in the .csv file
    disp('Matlab wrote:');
    disp(MM);
end


% Clear up memory upon termination
pp.shutdown()
clear java;

toc
