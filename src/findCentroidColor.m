function centroids = findCentroidColor (k, w, p, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
close all; clear all; clc;

%lighting conditions (dark = true, bright = false)
k = false;

%marker plot
w = true;

%plot
p = false;

%debug
d = false;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DARK   = k;
PLOT_M = w;
PLOT   = p;
DEBUG  = d;

%gets image
image = aquireImage;

%finds the centroids of the round objects in the image
iCentroids = aquireCentroid(image, DARK, PLOT_M, PLOT, DEBUG);

%checks whether there are objects
if iCentroids(1,:) == [40 4]
    %writes special error code to output if no objects
    centroids = iCentroids;
    disp('No objects left!');
    
else
    
    %converts centroid image coordinates to robot base-frame coordiates
    centroids = zeros(size(iCentroids,1),3);
    for i = 1:size(iCentroids,1)
        centroids(i,1:2) = mn2xy(iCentroids(i,1), iCentroids(i,2), true);
    end
    
    %aquires the colors of the objects
    LAB = aquireColor(image, iCentroids, 14, PLOT, false);
    
    if true
        LAB
    end
    
    %classifies the objects as yellow, green, or blue
    color = classifyColor(LAB, DEBUG);
    
    %displays color and appends centroid matrix with color identfier
    for i = 1:size(color,1)
        
        %writes the identifier of the color of the centroid to the centroid
        %matrix
        centroids(i,3) = color(i,1);
        
        %displays the color of each centroid in order
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

end