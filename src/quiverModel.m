function T = quiverModel(q, qd, s,  d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
%input = [theta1;   theta2;  theta3]
     q = [     0;       15;       0];
%input = [dtheta1; dtheta2; dtheta3]
    qd = [      5;       5;       0];
     
d = false;
s = 0.05;
%}     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


DEBUG = d;
scale = s;

%position of end effector
tcp = fwkin3001(q, true, d);
x = tcp(1,1);
y = -tcp(2,1);
z = tcp(3,1);

%velocity of end effector
pd = fwddiffkin3001(q, qd, d);

u = pd(1,1);
v = pd(2,1);
w = pd(3,1);

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
quiver3(x, y, z, u, v, w, scale);


end