function averageLAB = aquireColor(img, c, r, p, d)

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
 c = [186, 290;  % blue
      304, 249;  % yellow
      427, 404]; % green

%radius
r = 5;
  
%plot
p = true;

%debug
d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%sets radius of LAB averaging
radius = r;

%plot
PLOT_I = p;

%debug
DEBUG = d;

if DEBUG & PLOT_I
    figure;
    imshow(img);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%grabs the color at the centroids
%red = img(y, x, 1);
%green = img(y, x, 2);
%blue = img(y, x, 3);

%pre-alocates the LAB matrix, where each row is the average LAB around a
%centroid
averageLAB = zeros(size(c,1),3);

%converts image from RGB format to HSV
%imgHSV = rgb2hsv(img);

%converts image from RGB format to L*a*b
imgLAB = rgb2lab(img);

%converts the image from int8 to double precision
%imgd = im2double(img);
    
%loops through all of the centroid coordinates to grab the pixel HSV values
for i = 1:size(c,1)
    cX = c(i,1);
    cY = c(i,2);
    
    %resets terms for calculating average color repeatedly
    count = 0;
    sumLAB = zeros(1,3);
    
    if cX == 0 & cY == 0
        
        %gives a zero average
        averageLAB(i,:) = [0 0 0];
        
    else
    
    %adds up the LAB from each pixel in the y-axis
    for m = 0:radius*2
        
        %adds up the LAB from each pixel in the x-axis
        for n = 0:radius*2
            x = cX - radius + m;
            y = cY - radius + n;
            LAB = squeeze(imgLAB(y,x,:)).';
            sumLAB(1,1) = sumLAB(1,1) + LAB(1,1);
            sumLAB(1,2) = sumLAB(1,2) + LAB(1,2);
            sumLAB(1,3) = sumLAB(1,3) + LAB(1,3);
            count = count +1;
            
            if DEBUG
                disp(sprintf('x=%f, y=%f, L*a*b: L=%f a=%f b=%f, sumLAB: L=%f a=%f b=%f, count=%f', x, y, LAB(1,1), LAB(1,2), LAB(1,3), sumLAB(1,1), sumLAB(1,2), sumLAB(1,3), count));
            end
        end
                
    end
            
    if DEBUG
        disp(sprintf('Cx = %f, Cy = %f', cX,cY));
        averageLAB
    end
    
    %averages the RGB for each dot
    averageLAB(i,:) = sumLAB/count;
    
    end
    
end

end