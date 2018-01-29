function T = stickModel(q, plot_on)

%variables
    %input = [theta1, theta2,   theta3]
        q = [     0,    15,       0];
    
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

plot_on = true;

if plot_on
% create a new figure, enable axes and grid
f = figure;
axis on, grid on, hold on, axis equal, shading interp

pbaspect([1 1 1]);
daspect([1 1 1]);
% center the figure on screen and resize it
fig_pos = [0, 0, 900, 900];
set(f, 'Position', fig_pos);

%sets camera angle
view(45,45);

%points
    %creates empty matrix of x, y, z columns values for points
    p = zeros(4,3);
    %take x, y, z from transforms as a 3x1 matrix and transforms them into
    %a 1x3 matrix and writes them into p on a different row so that each
    %row is a new set of points
    a1 = A1;
    a2 = A1*A2;
    a3 = A1*A2*A3;
    p(2,:) = a1(1:3,4).';
    p(3,:) = a2(1:3,4).';
    p(4,:) = a3(1:3,4).';
    
%plots the x, y, z columns 
plot3(p(:,1), p(:,2), p(:,3), 'linewidth', 5);
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');

%color map for joints
CMap = parula(4);
%radius of dots for joints
radius = 10;
%sets dots to joint locations
D = zeros(3,4);
D(1:3,1:3) = p(1:3,:);
D(:,4) = radius;

%plots the joints as dots
[X, Y, Z] = sphere;
    for k = 1:size(D,1)
      XX = X * D(k, 4) + D(k, 1);
      YY = Y * D(k, 4) + D(k, 2);
      ZZ = Z * D(k, 4) + D(k, 3);
      surface(XX, YY, ZZ, 'FaceAlpha', 1, 'FaceColor', CMap(k, :), ...
              'EdgeColor', [0.4, 0.4, 0.4]);
    end

end