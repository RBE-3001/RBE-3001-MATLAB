function qT = cubicPoly(p, n, time, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
%initial trajectory
p = [ 250, 175,  50,  250;  % X-axis poistion values
     -200, -50, 250, -200;  % Y-axis poistion values
     300, -50, 250,  300];  % Z-axis poistion values

%number of points between two points
n = 10;

%time between points
time = 5;

d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;


%Uses quintic function to build the quintic trajectory
qT = zeros(3, ((size(p,2)-1)*n)+1);

if DEBUG
qT
end

%loop for rows (makes a trajectory for the x-axis, then y-axis, then %z-axis)
for u = 1:size(p,1)

%resets the timeOffset back to zero
timeOffset = 0;

%loop for columns (makes a tracjectory between points in different
                   %columns in a specific row)
for y = 1:(size(p,2)-1)

%grabs the two points to interpolate between from the the input
%trajectory
positions = [p(u,y); p(u,y+1)];

if DEBUG
positions
end

%start/stop positions (mm)
q_0     = positions(1,1);
q_f     = positions(2,1);

if DEBUG
positions
q_0
q_f
end

%difference in position (mm)
q_d     = q_f - q_0;
%start/stop velocity (degrees/s)
v_0     = 0;
v_f     = 0;

terms = [    q_0;
         v_0;
         q_f;
         v_f; ];

if q_d ~= 0
t_0 = timeOffset;
t_f = timeOffset + time;

%This is the main base matrix
QBaseM = [1,  t_0, (t_0)^2,   (t_0)^3;
          0,    1, 2*(t_0), 3*(t_0)^2;
          1,  t_f, (t_f)^2,   (t_f)^3;
          0,    1, 2*(t_f), 3*(t_f)^2];
if DEBUG
QBaseM
rankQ = rank(QBaseM);
rankQ
terms
end


%Quintic Polynomial Coeficents
%A = [a0; a1; a2; a3]


%Matrix Output
A = (inv(QBaseM))*terms;

else
A = zeros(4,1);
end

if DEBUG
A
end

%creates a time matrix for plugging into the quintic funciton
t = zeros(1,n);
for j = 1:n

if DEBUG
t
j
n
timeOffset
end

t(1,j) = time/n*j + timeOffset;

end

if DEBUG
t
end


%builds the quintic trajectory with the quintic function
if y == 1
qT(u,1) = A(1,1);
end

for k = 1:n
if DEBUG
qT
disp(sprintf('u = %f, y = %f, k = %f, timeOffset = %f', u, y, k, timeOffset));

end
%fills in each trajectory point between the start and stop positions
qT(u,((y-1)*n+k+1)) = A(1,1) + A(2,1)*(t(1,k))^1 + A(3,1)*(t(1,k))^2 + A(4,1)*(t(1,k))^3;

end

%incriments time as the columns advance
timeOffset = timeOffset + time;

end

end

if DEBUG
disp('final trajectory:');
qT
end

end
