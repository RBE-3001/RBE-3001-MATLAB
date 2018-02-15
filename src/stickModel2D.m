function ans = stickModel2D(q0, labNumber, DEBUG)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%variables
    
    %input = [theta1, theta2,   theta3]
         q0 = [     0;    -20;       30];
         labNumber = 4;
         DEBUG = true;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    lab = labNumber;                 %sets the lab number
   
%transform matrices
    %  = tdh( theta,      d,    alpha,      a)
    A2 = tdh(-q0(2,1),      0,       0,    175);
    A3 = tdh(-q0(3,1),      0,       0,    180);
    
    F1 = A2;
    F2 = A2 * A3;

    x = [0, F1(1, 4), F2(1, 4)];
    
    z = [135, F1(2, 4) + 135, F2(2, 4) + 135];
    
    plot(x, z, 'linewidth', 5, 'Color', 'b');
    xlabel('X-axis');
    ylabel('Z-axis');
    axis([0 , 370, 0, 350]);
    title(sprintf('RBE Lab 3001 %d: Interactive Robot Arm Display', labNumber));
    
    pF = ginput(1);
    
    if DEBUG
        q0
        pF
    end

%get output from numerical Inverse Kinematic function
    qF = numInvKin(q0, pF, DEBUG);
    
    A2 = tdh(-qF(2,1),      0,       0,    175);
    A3 = tdh(-qF(3,1),      0,       0,    180);
    
    F1 = A2;
    F2 = A2 * A3;

    x = [0, F1(1, 4), F2(1, 4)];
    z = [135, F1(2, 4) + 135, F2(2, 4) + 135];
    
    plot(x, z, 'linewidth', 5, 'Color', 'b');
    xlabel('X-axis');
    ylabel('Z-axis');
    axis([0 , 370, 0, 350]);
    title(sprintf('RBE Lab 3001 %d: Interactive Robot Arm Display', labNumber));
    
    ans = qF;
end