function c = aquireImage2

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

%blur image
h = ones(5,5) / 25;
img2 = imfilter(img, h);
%figure;
%imshow(img2);

%filters out most of the junk
img3 = createMask1(img2);
img3 = imcomplement(img3);
%figure;
%imshow(img3);

%remove tiny bits
LB = 750;
UB = 2200;
img4 = xor(bwareaopen(img3, LB), bwareaopen(img3, UB));
%figure;
%imshow(img4);

[B,L] = bwboundaries(img4,'noholes');

% Display the label matrix and draw each boundary
imshow(label2rgb(L, @jet, [.5 .5 .5]))
hold on
for k = 1:length(B)
  boundary = B{k};
  plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
end

stats = regionprops(L,'Area','Centroid');

threshold = 0.60;

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
    centroid = stats(k).Centroid;
    plot(centroid(1),centroid(2),'ko');
  end
  
  text(boundary(1,2)-35,boundary(1,1)+13,metric_string,'Color','y',...
       'FontSize',12,'FontWeight','bold');
  
end

end