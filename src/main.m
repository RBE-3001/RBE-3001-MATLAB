%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%   RBE3001 C18 Team 4: Hannah Baez, Alex Tacescu, Sam White  %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% take out the trash
close all; clc; clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% variable declarations

%booleans
DEBUG             = false;    %enables/disables debug prints
DEBUG_COMS        = false;    %displays communication debug messages
PLOT              = true;    %enables/diables plotting
PLOT_I            = false;    %enables/disables image plotting
PLOT_M            = true;     %plots a marker for centroids on the camera image
DATALOG           = true;     %enables/disables data logging
GRAVITY_COMP_TEST = false;    %enables gravity comp test, setting all PID values to 0     
DARK              = true;    %lighting conditions for camera (dark = true, bright = false)

%numerical
degreesPerTics    = 360/4096;               %calibrates the degrees per encoder tic
lab               = 5;                      %sets the lab number
axe = [-400, 400, -400, 400, -150, 650];    %sets axis parameters for live plot
routine           = 0;                      %controls what swtich case is active
last              = 0;                      %record of last case
next              = 1;                      %controls what logic next case is after any auxilary tasks

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% deletes old .csv files
cleanCSV();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% core matrix initialization

%initializes empty matricies for core data logging
m = zeros(0,15);
copym = m;
time = zeros(1, 0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% sets locations for object sorting

%home and get force vector position
homeForce = [355;0;135];

%move arm out of the way for photo
outOfWay = [250; 0; 300];

%move arm to light green drop location
dropLightGreen = [0; -180; 0];

%move arm to heavy green drop location
dropHeavyGreen = [0; 180; 0];

%move arm to light blue drop location
dropLightBlue = [100; -180; 0];

%move arm to heavy blue drop location
dropHeavyBlue = [100; 180; 0];

%move arm to light yellow drop location
dropLightYellow = [200; -180; 0];

%move arm to heavy yellow drop location
dropHeavyYellow = [200; 180; 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% sets initial position as home position

currentPoint = homeForce;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% autonomous sorting logic
while routine ~= 666
    
    disp(sprintf('last = %d, routine = %d, next = %d', last, routine, next));
    
    switch routine
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% moves robot to an out of the way position to get ready for a photo
        case 10
            %sets desired robot position to out of the way
            desiredPoints = [currentPoint, outOfWay];
            
            %updates current robot location
            currentPoint = outOfWay;
            
            %makes sure gripper is open initially
            gripper = 1;
            
            last = routine;
            routine = 105; %build trajectory, quintic interpolation
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% gets desired points and colors from camera
        case 0
                    
            %checks if robot is in out of way position
            if last ~= 10
                %puts robot in out of way position
                routine = 10;
                next = 0;
                
            else
                
                %gets x,y and color data of each point of interest
                centroids = findCentroidColor(DARK, PLOT_M, PLOT_I, DEBUG);
                               
                if centroids == [4 0 4]
                    disp('I am done sorting!');
                    
                else
                    %extracts the first color of the first object
                    colorFirst = centroids(1,3)
                    
                    switch colorFirst
                        case 0 %empty
                            disp('I found an empty object');
                            %kicks out of program because no next routine case set
                            routine = 666;

                        case 4 %red
                            disp('I found a red object. What do I do?');
                            %kicks out of program because no next routine case set
                            routine = 666;
                            
                        otherwise
                            %sets the color code for sorting later
                            colorFirst = centroids(1,3);
                            %pick up an object and weigh it
                            last = routine;
                            routine = 20; %pickup
                            next = 30; %close gripper
                    end
                end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% picks up an object
        case 20
            %waits for arm to stabilize
            pause(1);
            
            %sets up a trajectory between the home position and the object
            objectLocation = [centroids(1,1); centroids(1,2); -40]
            %makes a clearance target
            objectClearance = objectLocation;
            %sets clearance
            objectClearance(1,3) = 0;
            desiredPoints = [currentPoint, objectClearance, objectLocation];
            
            %updates current robot location
            currentPoint = objectLocation; 
            
            last = routine;
            routine = 105; %build trajectory, quintic interpolation
            next = 30; %weigh, already set
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% closes gripper
        case 30
            %waits for arm to stabilize
            pause(1);
            
            %open gripper
            gripper = 0;
            
            %stays in current position while closing the gripper
            desiredPoints = currentPoint;
            
            last = routine;
            routine = 100; %stay stationary
            res = 5; %data resolution
            next = 1;
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% opens gripper
        case 31
            %open gripper
            gripper = 1;
            
            %stays in current position while opening the gripper
            desiredPoints = currentPoint;
            
            last = routine;
            routine = 100; %stay stationary
            res = 5; %data resolution
            next = 0;
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% go to home position for weighing
        case 11
            %sets up a trajectory between the objectLocation and the weigh
            %position
            desiredPoints = [currentPoint, homeForce];
            
            last = routine;
            routine = 105; %build trajectory, quintic interpolation
            next = 12;
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% stay at home position to get steady values for the weight of the object
        case 12
            %sets up a static trajectory in the weigh position
            desiredPoints = [homeForce];
            
            last = routine;
            routine = 100; %stationary trajectory, no interpolation
            res = 50; %data resolution
            next = 1;
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% determines if an object is heavy or not
        case 1
            %checks if arm is in the correct weigh position and moves it there
            %if it isn't
            if last ~= 12
                last = 1;
                routine = 11;
                next = 1;
            else
                %find average z force over previous stationary
                avgZForce = calculateForces(mS, degreesPerTics, DEBUG);
                
                avgZForce
                                
                last = routine;
                %selects set-down position based on weight and color
                if avgZForce > 350 %heavy
                    switch colorFirst
                        case 1 %move arm to heavy yellow drop location
                            desiredPoints = [homeForce, dropHeavyYellow];
                            currentPoint = dropHeavyYellow;

                        case 2 %move arm to heavy green drop location
                            desiredPoints = [homeForce, dropHeavyLight];
                            currentPoint = dropHeavyGreen;

                        case 3 %move arm to heavy blue drop location
                            desiredPoints = [homeForce, dropHeavyBlue];
                            currentPoint = dropHeavyBlue;
                            
                    end
                    
                else %light
                    switch colorFirst
                        case 1 %move arm to heavy yellow drop location
                            desiredPoints = [homeForce, dropLightYellow];
                            currentPoint = dropLightYellow;

                        case 2 %move arm to heavy green drop location
                            desiredPoints = [homeForce, dropLightGreen];
                            currentPoint = dropLightGreen;

                        case 3 %move arm to heavy blue drop location
                            desiredPoints = [homeForce, dropLightBlue];
                            currentPoint = dropLightBlue;
                    end
                end
                
                routine = 105; %build trajectory, quintic interpolation
                next = 31; %opens gripper
                
            end
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AUXILARY: stationary trajectory generation for a single point to collect data
        case 100
            %interpolation mode selection: 0 = no interpolation
            interMode = 0;
            
            %number of points between interpolations
            interPoints = 0;
            
            %duration (s) between intermpolations
            nonLinearInterDuration = 0;
                       
            encoderTrajectory = buildTrajectory( desiredPoints, interMode, interPoints, nonLinearInterDuration, res, degreesPerTics, DEBUG);
            
            %do not set last because auxilary routines have no trace
            routine = 200;
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AUXILARY: trajectory generation for multiple points using qunintic interpolation
        case 105
            %interpolation mode selection: 5 = quintic
            interMode = 5;
            
            %number of points between interpolations
            interPoints = 50;
            
            %duration (s) between intermpolations
            nonLinearInterDuration = 3;
            
            %data resolution
            res = 1;
            
            encoderTrajectory = buildTrajectory( desiredPoints, interMode, interPoints, nonLinearInterDuration, res, degreesPerTics, DEBUG);
            
            %do not set last because auxilary routines have no trace
            routine = 200;
            %do not set next because auxilary routine isn't a logic routine
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AUXILARY: commands movement
        case 200
            tic %starts an elapse timer
                      
            [mS, copymS, timeS] = moveArm (encoderTrajectory, gripper, degreesPerTics, axe, lab, GRAVITY_COMP_TEST, DATALOG, PLOT, DEBUG_COMS, DEBUG);
            
            %concatenation of core data logging matricies
            m = [m; mS];
            copym = [copym; copymS];
            time = [time, timeS];
            
            %do not set last because auxilary routines have no trace
            routine = next;
            %do not set next because auxilary routine isn't a logic routine
            
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% data logging and post process plotting

    %handles data logging and plotting utilities
    postProcessing(m, copym, time, degreesPerTics, lab, DATALOG, PLOT, DEBUG);

%displays end time of operation
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%