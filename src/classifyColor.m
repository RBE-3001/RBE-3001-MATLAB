function color = classifyColor (hue, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
close all; clear all; clc;

%hue = 0;
%hue = 0.1236; %yellow
%hue = 0.2788; %green
%hue = 0.3573; %blue

%debug
d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;

if hue < 0.2
    color = 'yellow'
end

if 0.2 < hue & hue < 0.3
    color  = 'green'
end

if 0.3 < hue
    color  = 'blue'
end
    
    if DEBUG
       color
    end
end