function T = quiverModel(q, qd, s, e, f, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
%input = [theta1;   theta2;  theta3]
     q = [     30;       45;       -73];
%input = [dtheta1; dtheta2; dtheta3]
    qd = [      9;     63;       8];

%scale
s = norm(qd);

%sets axis parameters for live plot
e = [-400, 400, -400, 400, -150, 650]; 


%Force = true, Velocity = false
f = true;

%debug
d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;
scale = s;
FORCE = f;
axe   = e;

%position of end effector
tcp = fwkin3001(q, true, d);
x = tcp(1,1);
y = -tcp(2,1);
z = tcp(3,1);

if FORCE
    %force vector of end effector
    u = qd(3,1);
    v = qd(2,1);
    w = qd(1,1);

else
    %velocity of end effector
    pd = fwddiffkin3001(q, qd, d);

    u = pd(1,1);
    v = pd(2,1);
    w = pd(3,1);
end

if DEBUG
    disp(sprintf('x = %f, y = %f, z = %f, u = %f, v = %f, w = %f, scale = %f', x, y, z, u, v, w, scale));
end

% create a new figure, enable axes and grid
T = gcf;

%plot settings
    axis on, grid on, shading interp
    %saves all of the points the robot has been on the graph
       hold on            
    %keeps graph and data the same size
    % pbaspect([1 1 1]);
    % daspect([1 1 1]);
     
     %lim = [-350, 350];
     %xlim(lim);
     %ylim(lim);
     %zlim([-100, 600]);
     
     axis(axe);
     
    % center the figure on screen and resize it
         fig_pos = [0, 0, 900, 900];
         set(T, 'Position', fig_pos);
    %sets camera angle
    view(45,45);

%graphs the velocity vectors
quiver3(x, y, z, u, v, w, scale);

end