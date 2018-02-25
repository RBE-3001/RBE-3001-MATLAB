function postProcessing (m, copym, time, dpt, l, p ,d)

%calibrates the degrees per encoder tic
degreesPerTics = dpt;

%lab number
lab = l;

%plot
PLOT = p;

%debug
DEBUG = d;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%   save and plot joint angles   %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %writes a .csv file for just the arm's joint angles
    joint1Angles = m(:,1).'*degreesPerTics;
    joint2Angles = m(:,4).'*degreesPerTics;
    joint3Angles = m(:,7).'*degreesPerTics;
    dlmwrite('JointAngles.csv', time, '-append');
    dlmwrite('JointAngles.csv', joint1Angles, '-append');
    dlmwrite('JointAngles.csv', joint2Angles, '-append');
    dlmwrite('JointAngles.csv', joint3Angles, '-append');
    
    if false
        %plots the arm's joint angles over time
        figure('Position', [0, 50, 864, 864]);
        plot(time, joint1Angles, 'r-*', time, joint2Angles, 'b--x', time, joint3Angles, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Angles vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Joint Angle (degrees)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;       
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%   save and plot joint velocities   %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for just the arm's joint VELOCITIES
    joint1Velocities = diff(m(:,1).'*degreesPerTics);
    joint2Velocities = diff(m(:,4).'*degreesPerTics);
    joint3Velocities = diff(m(:,7).'*degreesPerTics);
    timeV = time(1,1:(size(time,2)-1));
    dlmwrite('JointVelocities.csv', timeV, '-append');
    dlmwrite('JointVelocities.csv', joint1Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint2Velocities, '-append');
    dlmwrite('JointVelocities.csv', joint3Velocities, '-append');

    if false
        %plots the arm's joint velocities over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeV, joint1Velocities, 'r-*', timeV, joint2Velocities, 'b--x', timeV, joint3Velocities, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Velocities vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Joint Velocity (degrees/s)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%   save and plot joint accelerations   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for just the arm's joint acceleration
    joint1Accelerations = diff(diff(m(:,1)).'*degreesPerTics);
    joint2Accelerations = diff(diff(m(:,4)).'*degreesPerTics);
    joint3Accelerations = diff(diff(m(:,7)).'*degreesPerTics);
    timeA = time(1,1:(size(time,2)-2));
    dlmwrite('JointAcclerations.csv', timeA, '-append');
    dlmwrite('JointAcclerations.csv', joint1Accelerations, '-append');
    dlmwrite('JointAcclerations.csv', joint2Accelerations, '-append');
    dlmwrite('JointAcclerations.csv', joint3Accelerations, '-append');

    if false
        %plots the arm's joint acceleration over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeA, joint1Accelerations, 'r-*', timeA, joint2Accelerations, 'b--x', timeA, joint3Accelerations, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Acclerations vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Joint Acceleration (degrees/s^2)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%   save and plot TCP x-y-z  position   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for the X-Y-Z position of the TCP
    Position = zeros(size(m,1),3);
    for k = 1:size(m,1)
        Position(k,1:3) = fwkin3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], true, DEBUG).';
    end
    
    xPosition = Position(:,1).';
    yPosition = Position(:,2).';
    zPosition = Position(:,3).';
    dlmwrite('X-Y-Z-Position.csv', time, '-append');
    dlmwrite('X-Y-Z-Position.csv', xPosition, '-append');
    dlmwrite('X-Y-Z-Position.csv', yPosition, '-append');
    dlmwrite('X-Y-Z-Position.csv', zPosition, '-append');
    
    if false
        %plots the X-Y-Z Position of the TCP over time
        figure('Position', [864, 50, 864, 864]);
        plot(time, xPosition, 'r-*', time, yPosition, 'b--x', time, zPosition, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: X-Y-Z Position of the TCP  vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Position of the TCP (mm)');
        legend('X Position', 'Y Position', 'Z Position');
        grid on;
    end
    
    if DEBUG
        disp('Position: X, Y, Z');
        disp(Position);
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%    save and plot determinant of Jp    %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
%writes a .csv file for the determinant of Jp
    detJp = zeros(size(m,1),1);
    for k = 1:size(m,1)
        J = jacob0([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], DEBUG);
        detJp(k,1) = det(J(1:3,:));
    end
    
    deter = detJp(:,1).';

    dlmwrite('detJp.csv', time, '-append');
    dlmwrite('detJp.csv', deter, '-append');
    
    if PLOT
        %plots the determinant of Jp over time
        figure('Position', [50, 50, 864, 864]);
        plot(time, deter, 'r-*', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Determinant of Jacobian Position Matrix (Jp) vs. Time',lab));
        xlabel('Time (s)');
        ylabel('Determinant of Jp');
        legend('Determinant of Jp');
        grid on;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%   save and plot TCP x-y-z  velocity   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for the X-Y-Z velocity of the TCP using position
    Velocity = zeros(size(m,1),3);
    for k = 1:size(m,1)-1
        Velocity(k,1:3) = fwddiffkin3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], [joint1Velocities(1,k); joint2Velocities(1,k); joint3Velocities(1,k)], DEBUG).';
    end
    
    xVelocity = Velocity(1:(size(Velocity,1)-1),1).';
    yVelocity = Velocity(1:(size(Velocity,1)-1),2).';
    zVelocity = Velocity(1:(size(Velocity,1)-1),3).';
    dlmwrite('X-Y-Z-Velocity.csv', timeV, '-append');
    dlmwrite('X-Y-Z-Velocity.csv', xVelocity, '-append');
    dlmwrite('X-Y-Z-Velocity.csv', yVelocity, '-append');
    dlmwrite('X-Y-Z-Velocity.csv', zVelocity, '-append');
    
    if false
        %plots the X-Y-Z Velocity of the TCP over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeV, xVelocity, 'r-*', timeV, yVelocity, 'b--x', timeV, zVelocity, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: X-Y-Z Velocity of the TCP  vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Velocity of the TCP (mm/s)');
        legend('X Velocity', 'Y Velocity', 'Z Velocity');
        grid on;
    end
    
    if DEBUG  
        disp('Velocity: X, Y, Z');
        disp(Velocity);
    end  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%   save and plot joint toques   %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %writes a .csv file for just the arm's joint angles
    joint1Torques = ADCToTorque(m(:,3),1,true).';
    joint2Torques = ADCToTorque(m(:,6),2,true).';
    joint3Torques = ADCToTorque(m(:,9),3,true).';
    dlmwrite('JointTorque.csv', time, '-append');
    dlmwrite('JointTorque.csv', joint1Torques, '-append');
    dlmwrite('JointTorque.csv', joint2Torques, '-append');
    dlmwrite('JointTorque.csv', joint3Torques, '-append');
    
    if PLOT
        %plots the arm's joint angles over time
        figure('Position', [0, 50, 864, 864]);
        plot(time, joint1Torques, 'r-*', time, joint2Torques, 'b--x', time, joint3Torques, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: Joint Torque vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Joint Torque (Nm)');
        legend('Base Joint', 'Elbow Joint', 'Wrist Joint');
        grid on;       
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%   save and plot TCP x-y-z  Forces   %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %writes a .csv file for the X-Y-Z forces of the TCP using position and
    %joint torques
    Force = zeros(size(m,1),3);
    for k = 1:size(m,1)-1
        Force(k,1:3) = statics3001([joint1Angles(1,k); joint2Angles(1,k); joint3Angles(1,k)], [joint1Torques(1,k); joint2Torques(1,k); joint3Torques(1,k)], DEBUG).';
    end
    
    xForce = Force(1:size(Force,1)-1,1).';
    yForce = Force(1:size(Force,1)-1,2).';
    zForce = Force(1:size(Force,1)-1,3).';
    dlmwrite('X-Y-Z-Force.csv', timeV, '-append');
    dlmwrite('X-Y-Z-Force.csv', xForce, '-append');
    dlmwrite('X-Y-Z-Force.csv', yForce, '-append');
    dlmwrite('X-Y-Z-Force.csv', zForce, '-append');
    
    if PLOT
        %plots the X-Y-Z Force of the TCP over time
        figure('Position', [864, 50, 864, 864]);
        plot(timeV, xForce, 'r-*', timeV, yForce, 'b--x', timeV, zForce, 'g-.O', 'LineWidth', 2);
        title(sprintf('RBE 3001 Lab %d: X-Y-Z Force of the TCP  vs. Time', lab));
        xlabel('Time (s)');
        ylabel('Force of the TCP (N)');
        legend('X Force', 'Y Force', 'Z Force');
        grid on;
    end
    
    if DEBUG  
        disp('Force: X, Y, Z');
        disp(Force);
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%   save and averages load cell readings   %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initializes average matrix
averageLoadCell = zeros(1,3);

%grabs load cell values and averages them
for i = 1:3
    count = 0;
    sum = 0;
    
    for j = 1:size(copym,1)
        %adds up all load cell readings
        sum = sum + copym(j,i*3);
        %incriments counter
        count = count + 1;
    end
    
    %writes average to matrix
    averageLoadCell(1,i) = sum/count;
    
end

%writes average values to a .csv file
dlmwrite('averageLoadCell.csv', averageLoadCell, '-append');

if DEBUG
    disp(sprintf('Load cell #1 = %f, #2 = %f, #3 = %f', averageLoadCell(1,1), averageLoadCell(1,2), averageLoadCell(1,3)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end