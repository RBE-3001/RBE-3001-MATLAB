function color = classifyColor (LAB, d)

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

%debug
DEBUG = d;

if size(LAB,1) == 0
    color = 0;
    
else
    
    %sets up output matrix
    color = zeros(size(LAB,1),1);
    
    for i = 1:size(LAB,1)
        
        %define color channels
        a = LAB(i,2);
        b = LAB(i,3);
        
        if abs(a) > abs(b)
            if a < 0
                color(i,1)  = 2; %green
            else %a > 0
                color(i,1) = 4; %red
            end
        else % abs(a) < abs(b)
            if b < 0
                color(i,1)  = 3; %blue
            else %b > 0
                color(i,1) = 1; %yellow
            end
        end
    end
    
    if DEBUG
        color
    end
    
end
    
end


