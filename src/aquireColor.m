function color = aquireColor(img, c, p, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
close all; clear all; clc;
%connects to webcam
cam = webcam();

%waits for image to stabilize
pause(1)

%grabs a frame of the webcame
img = snapshot(cam);

%plot
p = true;

%debug
d= true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%plot
PLOT = p;

%debug
DEBUG = d;

red = img(y, x, 1);
green = img(y, x, 2);
blue = img(y, x, 3);

pixel = squeeze(img(x,y,:);

end