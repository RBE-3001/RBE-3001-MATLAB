function T = traceModel(q)
%close all;

%variables
    %input = [theta1b, theta2b,   theta3b, theta1b, theta2b,   theta3b]
       % q = [      0,      15,         0,       0,      25,         0];
    
%constants
    %link lenths
    L1 = 0.5;
    L2 = 0.5;
    L3 = 0.2;
    
%transform matrices
    %  = tdh( theta,      d, alpha,      a)
    A1 = tdh(q(1,1),    135,     0,      0);
    A2 = tdh(q(1,2),      0,   -90,    175);
    A3 = tdh(q(1,3),      0,     0, 169.28);
    B1 = tdh(q(1,4),    135,     0,      0);
    B2 = tdh(q(1,5),      0,   -90,    175);
    B3 = tdh(q(1,6),      0,     0, 169.28);

% create a new figure, enable axes and grid
    T = gcf;
    
    %plot settings
    axis on, grid on, axis equal, axis square, shading interp
    %saves all of the points the robot has been on the graph
       hold on            
    %keeps graph and data the same size
    % pbaspect([1 1 1]);
    % daspect([1 1 1]);
     
     lim = [-350, 350];
     xlim(lim);
     ylim(lim);
     zlim(lim);
     
     axis([-350, 350, -350, 350, -350, 350]);
     
    % center the figure on screen and resize it
         fig_pos = [0, 0, 900, 900];
         set(T, 'Position', fig_pos);
    %sets camera angle
    view(45,45);

%points
    %creates empty matrix of x, y, z columns values for points
    p = zeros(2,3);
    %take x, y, z from transforms as a 3x1 matrix and transforms them into
    %a 1x3 matrix and writes them into p on a different row so that each
    %row is a new set of points
    %a1 = A1;
    %a2 = A1*A2;
    a3 = A1*A2*A3;
    %b1 = B1;
    %b2 = B1*B2;
    b3 = B1*B2*B3;
    p(1,:) = a3(1:3,4).';
    p(2,:) = b3(1:3,4).';
    
%plots the x, y, z columns 
    plot3(p(:,1), p(:,2), p(:,3), 'linewidth', 1, 'Marker', '+', 'Color', 'r');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    title('RBE 3001 Lab 2: Live Arm-position Plot');

hold off
    end