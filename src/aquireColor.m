function averageHue = aquireColor(img, c, r, p, d)

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

%test centroid
 c = [253, 376;  % yellow
      311, 273;  % green
      404, 368]; % blue

%plot
p = true;

%debug
d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%sets radius of hue averaging
radius = r;

%plot
PLOT = p;

%debug
DEBUG = d;

if DEBUG & PLOT
    figure;
    imshow(img);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%grabs the color at the centroids
%red = img(y, x, 1);
%green = img(y, x, 2);
%blue = img(y, x, 3);

%pre-alocates the hue matrix, where each row is the average hue around a
%centroid
averageHue = zeros(size(c,1),1);

%converts image from RGB format to HSV
imgHSV = rgb2hsv(img);
    
%loops through all of the centroid coordinates to grab the pixel HSV values
for i = 1:size(c,1)
    cX = c(i,1);
    cY = c(i,2);
    
    %resets terms for calculating average hue repeatedly
    count = 0;
    sumH = 0;
    
    %adds up the hue from each pixel in the y-axis
    for m = 0:radius*2
        
        %adds up the hue from each pixel in the x-axis
        for n = 0:radius*2
            x = cX - radius + m;
            y = cY - radius + n;
            hsv = squeeze(imgHSV(x,y,:)).';
            sumH = sumH + hsv(1,1);
            count = count +1;
            
            if DEBUG
                disp(sprintf('x = %f, y = %f, hue = %f, sumH = %f, count = %f', x, y, hsv(1,1), sumH, count));
            end
        end
                
    end
            
    if DEBUG
        disp(sprintf('Cx = %f, Cy = %f', cX,cY));
        averageHue
    end
    
    %averages the hue for each dot
    averageHue(i,1) = sumH/count;

end

end