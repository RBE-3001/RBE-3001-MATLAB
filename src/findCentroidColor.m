function centroids = findCentroidColor (w, p, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear all; clc;

%marker plot
w = true;

%plot
p = false;

%debug
d = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PLOT_M = w;
PLOT   = p;
DEBUG  = d;

%gets image
image = aquireImage;

%finds the centroids of the round objects in the image
iCentroids = aquireCentroid(image, PLOT_M, PLOT, false);

%converts centroid image coordinates to robot base-frame coordiates
centroids = zeros(size(iCentroids,1),2);
for i = 1:size(iCentroids,1)
    centroids(i,:) = mn2xy(iCentroids(i,1),iCentroids(i,2));
end

%aquires the colors of the objects
hue = aquireColor(image, iCentroids, PLOT, true);

if DEBUG
    hue
end

%classifies the objects as yellow, green, or blue
for i = 1:size(hue,1)
    classifyColor(hue(i,1), DEBUG);
end

end