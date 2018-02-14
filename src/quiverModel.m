function T = quiverModel(q, v, d)

DEBUG = d;

%position of end effector
x = q(1,1);
y = q(2,1);
z = q(3,1);

%velocity of end effector
u = v(1,1);
v = v(2,1);
w = v(3,1);

% create a new figure, enable axes and grid
T = gcf;

%plot settings
    axis on, grid on, shading interp
    %saves all of the points the robot has been on the graph
       hold on            
    %keeps graph and data the same size
    % pbaspect([1 1 1]);
    % daspect([1 1 1]);
     
     lim = [-350, 350];
     xlim(lim);
     ylim(lim);
     zlim([-100, 600]);
     
     axis([-350, 350, -350, 350, -100, 600]);
     
    % center the figure on screen and resize it
         fig_pos = [0, 0, 900, 900];
         set(T, 'Position', fig_pos);
    %sets camera angle
    view(45,45);

%graphs the velocity vecotors
quiver3(x, y, z, u, v, w);


end