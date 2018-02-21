function c = aquireImage

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%   Image Aquisition   %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc;
%connects to webcam
cam = webcam();

%waits for image to stabilize
pause(1)

%grabs a frame of the webcame
img = snapshot(cam);

%displays image
%imshow(img);

c = img;
%use to convert camera coodinates to x and y coordinates
%[x,y] = mn2xy(m,n)

end