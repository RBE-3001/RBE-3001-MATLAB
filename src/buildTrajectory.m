function encoderTrajectory = buildTrajectory( p, iM, iP, nlD, r, dpt, d)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% variable initialization

%input points that will become a trajectory by using inverse kinematics to
%produce a trajectory with variable data resolution
desiredPoints = p;

%selects between linear, cubic, and quintic polynomial point interpolation,
%1 = linear, 3 = cubic, 5 = quintic
interMode = iM;

%sets the number of points between two points for cubic/quintic/linear
%interpolation
InterPoints = iP;

%sets the duration (s) between two points for cubic/quintic intermpolation
nonLinearInterDuration = nlD;

%sets the number of points to be reapeated for better data resolution,
%limited by communication speed, so keep low if the number of points
% between two points in the trajectory generation is high (InterPpoints)
res = r;

%calibrates the degrees per encoder tic
degreesPerTics = dpt;

%debug messages
DEBUG = d;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% X-Y-Z trajectory interpolation

%sets interpolation mode
switch interMode
    
    case 1
        %linear interpolation between all set-points
        xyzTrajectory = linearInterpolation(desiredPoints, InterPoints, DEBUG);
        
    case 3
        %cubic polynomial interpolation between all setpoints
        xyzTrajectory = cubicPoly(desiredPoints, InterPoints, nonLinearInterDuration, DEBUG);
    
    case 5
        %quintic polynomial interpolation between all setpoints
        xyzTrajectory = quinticPoly(desiredPoints, InterPoints, nonLinearInterDuration, DEBUG);
    
    otherwise
        %no interpolation
        xyzTrajectory = desiredPoints;
end
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% converts to encoder ticks and modifies point resolution

xyzTrajectory

%converts x-y-z points (mm) to encoder values, can increase the number of
%identical points for greater data resolution when points are far apart
encoderTrajectory = pointResolution(xyzTrajectory, res, degreesPerTics, DEBUG);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%