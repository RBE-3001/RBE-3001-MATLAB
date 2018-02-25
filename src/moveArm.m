function [m, copym, time] = moveArm (v, g, dpt, a, l, dL, p, dC, d)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% sets java path
javaaddpath('../    lib/hid4java-0.5.1.jar');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% imports

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% variable initialization

%trajectory in encoder ticks
viaPts = v;

%opens or closes the gripper during the trajectory
gripper = g;

%calibrates the degrees per encoder tic
degreesPerTics = dpt;

%applies axis configuration for plots
axe = a;

%sets the lab number
lab = l;

%packet processor (device ID)
pp = PacketProcessor(7);

%logs data
DATALOG = dL;

%plotting functions
PLOT = p;

%communication debug statements
DEBUG_COMS = dC;

%debug messages
DEBUG = d;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% server selection
% Create a PacketProcessor object to send data to the nucleo firmware
SERV_ID = 30;            % we will be talking to server ID 30, LabServer
% the Nucleo

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% large matrix initialization

%initialize our temporary matrix to store data to be written to the .csv in
%a matrix the size of the number of setpoints by the number of returned
%data elements (15)
m = zeros(size(viaPts,2),15);
copym = m;
time = zeros(1, size(viaPts,2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% packet initialization
% Instantiate a packet - the following instruction allocates 64
% bytes for this purpose. Recall that the HID interface supports
% packet sizes up to 64 bytes.
packet = zeros(15, 1, 'single');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% packet communication

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
    packet(10) = gripper;
    
    
    %Send packet to the server and get the response
    returnPacket = pp.command(SERV_ID, packet);
    
    toc %displays the elapsed time since tic

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% critical data logging

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
%% communication debugging 

    if DEBUG_COMS
        disp('Sent Packet:');
        disp(packet);
        disp('Received Packet:');
        disp(returnPacket);
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% live plotting

    %scalars for vector plotting of force and velocity
    forceScale = 100;
    velocityScale = 0.025;
    
    %chose between plotting force vector and velocity vector: true = force,
    %false = velocity
    FORCE = true;

    %handles all live plotting: robot, trajectory, and force/velocity vector
    livePlotModel(m, k, degreesPerTics, lab, axe, forceScale, velocityScale, FORCE, PLOT, DEBUG);   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pause

    pause(0.001) %timeit(returnPacket) !FIXME why is this needed?
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% shutdown of communication channel to allow for resume later

    pp.shutdown()

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%