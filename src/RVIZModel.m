javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;


pp = PacketProcessor(7);
values = zeros(15, 1, 'single');
sinWaveInc = 10.0;
enctomm = 3;
range = 400.0;
%Inputs: angles of each link in radians
%   sends joint commands via ROS to the RVIZ model
%   IF THIS FUNCTION IS IN USE, MUST USE rosinit ONCE BEFORE RUNNING
%**INITIALIZATION COMMANDS WILL NEED TO BE MOVED INTO MAIN CODE, WHEN WE
%ACTUALLY WANT TO RUN THIS **
%Create cell array of link names for the model
LinkNames = {'Bottomtolink0', 'link0tolink1', 'link1tolink2'};
%Initialize ros joint state publisher
[LinkPublisher,message] = rospublisher('/joint_states', 'sensor_msgs/JointState');

message.Name = LinkNames;%Set Link names to tell ROS what link to apply what value
i = 1;
while 1%for testing purposes
    for k=1:sinWaveInc
     incremtal = (single(k) / sinWaveInc);
     for j=0:4
         %Send a new setpoint based on a since wave
         values((j * 3) + 1) = (sin(incremtal * pi *2.0 )*range)+(range);
         %Send junk data for velocity and force targets
         values((j * 3) + 2) = 0;
         values((j * 3) + 3) = 3;
     end
     tic
     %Process command and print the returning values
     returnValues = pp.command(30, values);
    end
i = i+1;
message.Header.Stamp = rostime('now');%Set ros time stamp
link1Theta = (returnValues(1)/4096)*2*pi;
link2Theta = -(returnValues(4)/4096)*2*pi;
link3Theta = -(returnValues(7)/4096)*2*pi;

pause (0.1);
message.Position = [link1Theta, link2Theta, link3Theta];%Set Position Data
send(LinkPublisher, message);%Send Position Data to ROS
end
pp.shutdown()
clear java;

