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

javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;


% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(7); % !FIXME why is the deviceID == 7?
SERV_ID = 37;            % we will be talking to server ID 37 on
                         % the Nucleo
                         % ***************This ID # needs to be changed to
                         % whatever the new server ID is******************

DEBUG   = true;          % enables/disables debug prints

% Instantiate a packet - the following instruction allocates 64
% bytes for this purpose. Recall that the HID interface supports
% packet sizes up to 64 bytes.
packet = zeros(15, 1, 'single');

% The following code generates a sinusoidal trajectory to be
% executed on joint 1 of the arm and iteratively sends the list of
% setpoints to the Nucleo firmware. 
%viaPts = [0, -400, 400, -400, 400, 0];

% the following is a null trajectory of five positions so that there will
% be five sets of arm data replyed to the status request
viaPts = [0, 0, 0, 0, 0, 0];

%initialize our temporary matrix to store data to be written to the .csv in
%a matrix the size of the number of setpoints by the number of returned
%data elements (9)
m = zeros(size(viaPts,2),9);

tic % What does this do? --> starts an elapse timer

% Iterate through a sine wave for joint values
for k = viaPts
    %incremtal = (single(k) / sinWaveInc);
    
    packet(1) = k;
    
    
    % Send packet to the server and get the response
    returnPacket = pp.command(SERV_ID, packet);
    toc %What does this do? --> ends the timer started by tic
    
    %adds the returned data to the temporary matrix as a row instead of a
    %column (list)
    m(k,:) = returnPacket;
    
    if DEBUG
        disp('Sent Packet:');
        disp(packet);
        disp('Received Packet:');
        disp(returnPacket);
    end
    
    pause(1) %timeit(returnPacket) !FIXME why is this needed?
end

%writes the temporary matrix data to a .csv file
for row = 0:size(m,1)
    csvwrite('armData.csv',m,0,row);
end

%displays the data inside the .csv file
if DEBUG
    %reads .csv file and stores contents in a temporary matrix
    M = csvread(armData.csv);
    %displays the matrix of the data in the .csv file
    disp('Matlab wrote:');
    disp(M);
end

% Clear up memory upon termination
pp.shutdown()
clear java;

toc
