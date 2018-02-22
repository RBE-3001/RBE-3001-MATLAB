function averageRGB = aquireColor(img, c, r, p, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
close all; clear all; clc;
%connects to webcam
cam = webcam();

%waits for image to stabilize
pause(1)

%grabs a frame of the webcame
img = snapshot(cam);

%test centroid
 c = [301, 384;  % green
      307, 307;  % yellow
      380, 342]; % blue

%radius
r = 3;
  
%plot
p = true;

%debug
d = true;
%
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
averageRGB = zeros(size(c,1),3);

%converts image from RGB format to HSV
%imgHSV = rgb2hsv(img);
    
%loops through all of the centroid coordinates to grab the pixel HSV values
for i = 1:size(c,1)
    cX = c(i,1);
    cY = c(i,2);
    
    %resets terms for calculating average color repeatedly
    count = 0;
    sumRGB = zeros(1,3);
    
    %adds up the hue from each pixel in the y-axis
    for m = 0:radius*2
        
        %adds up the hue from each pixel in the x-axis
        for n = 0:radius*2
            x = cX - radius + m;
            y = cY - radius + n;
            RGB = squeeze(img(x,y,:)).';
            sumRGB(1,1) = sumRGB(1,1)*dk + RGB(1,1)*dk;
            sumRGB(1,2) = sumRGB(1,2)*dk + RGB(1,2)*dk;
            sumRGB(1,3) = sumRGB(1,3)*dk + RGB(1,3)*dk;
            count = count +1;
            
            if DEBUG
                disp(sprintf('x=%f, y=%f, RGB: R=%f G=%f B=%f, sumH: R=%f G=%f B=%f, count=%f', x, y, RGB(1,1), RGB(1,2), RGB(1,3), sumRGB(1,1), sumRGB(1,2), sumRGB(1,3), count));
            end
        end
                
    end
            
    if DEBUG
        disp(sprintf('Cx = %f, Cy = %f', cX,cY));
        averageRGB
    end
    
    %averages the RGB for each dot
    averageRGB(i,:) = sumRGB/count;

end

end