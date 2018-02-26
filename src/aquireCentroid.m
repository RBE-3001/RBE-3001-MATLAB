function centroidMatrix = aquireCentroid(img, k, w, p, d)

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

%lighting conditions (dark = true, bright = false)
k = false;

%marker plot
w = true;

%plot
p = true;

%debug
d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%lighting conditions
DARK = k;

%plot base image with markers
PLOT_M = w;

%plot
PLOT = p;

%debug
DEBUG = d;

if PLOT_M
    %shows original image
    figure;
    imshow(img);
    hold on
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%blur image
h = ones(5,5) / 25;
img2 = imfilter(img, h);

if DEBUG & PLOT
    figure;
    imshow(img2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%filters out most of the junk
if DARK
    %for darker lighting conditions
    img3 = createMask1(img2);
    img3 = imcomplement(img3);
else
    %for more bright lighting conditions
    img3 = createMask2(img2);
end

if DEBUG & PLOT
    figure;
    imshow(img3);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%remove tiny bits
LB = 750;
UB = 2500;
img4 = xor(bwareaopen(img3, LB), bwareaopen(img3, UB));

if DEBUG & PLOT
    figure;
    imshow(img4);
    hold on;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%finds the bounaries
[B,L] = bwboundaries(img4,'noholes');

% Display the label matrix and draw each boundary
if PLOT
    imshow(label2rgb(L, @jet, [.5 .5 .5]));
end

%finds the boundaries
for k = 1:length(B)
  boundary = B{k};
  
  if PLOT
  plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
  end
  
end

stats = regionprops(L,'Area','Centroid');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%threshold of % likelyhood that an object is a circle
threshold = 0.50;

%initialize centroid matrix's minimum size
centM = zeros(length(B),2);

  %initializes circle counter
  i = 1;

  %initializes numCircles
  numCircles = 0;

% loop over the boundaries
for k = 1:length(B)

  % obtain (X,Y) boundary coordinates corresponding to label 'k'
  boundary = B{k};

  % compute a simple estimate of the object's perimeter
  delta_sq = diff(boundary).^2;    
  perimeter = sum(sqrt(sum(delta_sq,2)));
  
  % obtain the area calculation corresponding to label 'k'
  area = stats(k).Area;
  
  % compute the roundness metric
  metric = 4*pi*area/perimeter^2;
  
  % display the results
  metric_string = sprintf('%2.2f',metric);


  
  % mark objects above the threshold with a black circle
  if metric > threshold
    centroid = round(stats(k).Centroid);
    centM(i,:) = [centroid(1), centroid(2)];
    i = i + 1;
    numCircles = i - 1;
    
    if DEBUG
        disp(sprintf('threshold = %f, metric = %f, length(B) = %d, Number of Circles: %d', threshold, metric, length(B), numCircles));
        centM
    end
    
    if PLOT_M
        %plots centroid locations of circles
        plot(centroid(1),centroid(2),'ko');
    end
    
  end
  
  if PLOT
      text(boundary(1,2)-35,boundary(1,1)+13,metric_string,'Color','y','FontSize',12,'FontWeight','bold');
  end
 
end

%checks if there are no objects left
if numCircles == 0
    %writes a special code to the output
    centroidMatrix = [40 4];
    
else
    %removes empty lines
    centroidMatrix = centM(1:numCircles,:);
end

end