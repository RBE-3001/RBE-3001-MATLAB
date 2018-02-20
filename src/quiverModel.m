function T = quiverModel(q, qd, s, f, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
%input = [theta1;   theta2;  theta3]
     q = [     1;       5;       4];
%input = [dtheta1; dtheta2; dtheta3]
    qd = [      0;      0;       9.8];

%scale
s = 0.05;

%Force = true, Velocity = false
f = true;

%debug
d = false;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


DEBUG = d;
scale = s;
FORCE = f;

%position of end effector
tcp = fwkin3001(q, true, d);
x = tcp(1,1);
y = -tcp(2,1);
z = tcp(3,1);

if FORCE
    %force vector of end effector
    u = qd(1,1);
    v = qd(2,1);
    w = qd(3,1);

    %scales force from Nm to Nmm
    scale = scale/1000;

else
    %velocity of end effector
    pd = fwddiffkin3001(q, qd, d);

    u = pd(1,1);
    v = pd(2,1);
    w = pd(3,1);
end

if DEBUG
    disp(sprintf('x = %f, y = %f, z = %f, u = %f, v = %f, w = %f, scale = %f', x, y, z, u, v, w, scale));


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

%graphs the velocity vectors
quiver3(x, y, z, u, v, w, scale);


end