function avgZForce = calculateForces (m, dpt, d)

%calibrates the degrees per encoder tic
degreesPerTics = dpt;

%debug messages
DEBUG = d;

%find joint angles
joint1Angles = m(:,1).'*degreesPerTics;
joint2Angles = m(:,4).'*degreesPerTics;
joint3Angles = m(:,7).'*degreesPerTics;

%find joint torques
joint1Torques = ADCToTorque(m(:,3), 1, DEBUG).';
joint2Torques = ADCToTorque(m(:,6), 2, DEBUG).';
joint3Torques = ADCToTorque(m(:,9), 3, DEBUG).';

%find x-y-z forces
Force = zeros(size(m,1), 3);
for k = 1:size(m, 1)-1
    Force(k,1:3) = statics3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], [joint1Torques(1,k); joint2Torques(1,k); joint3Torques(1,k)], DEBUG).';
end

%extracts specifically the z-force
zForce = Force(1:size(Force,1)-1, 3);

%initializes variables for calculating the average z force
sum = 0;
count = 0;

%adds up all of the elements in the z force
for i = 1:size(zForce)
    sum = sum + zForce(i,1);
    count = count + 1;
end

%calculates average z force
avgZForce = sum/count;

end