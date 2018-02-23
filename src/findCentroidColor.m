function centroids = findCentroidColor (w, p, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
close all; clear all; clc;

%marker plot
w = true;

%plot
p = false;

%debug
d = false;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PLOT_M = w;
PLOT   = p;
DEBUG  = d;

%gets image
image = aquireImage;

%finds the centroids of the round objects in the image
iCentroids = aquireCentroid(image, PLOT_M, PLOT, DEBUG);

iCentroids

%converts centroid image coordinates to robot base-frame coordiates
centroids = zeros(size(iCentroids,1),2);
for i = 1:size(iCentroids,1)
    centroids(i,:) = mn2xy(iCentroids(i,1),iCentroids(i,2));
end

%aquires the colors of the objects
LAB = aquireColor(image, iCentroids, 14, PLOT, false);

if true
    LAB
end

%classifies the objects as yellow, green, or blue
color = classifyColor(LAB, DEBUG);

for i = 1:size(color,1)
    switch color(i,1)
        case 0
        case 1
            'yellow'
        case 2
            'green'
        case 3 
            'blue'
        case 4
            'red'
    end
    
end

end