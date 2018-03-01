function qd = invdiffkin3001(q, pd, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%test joint angles (units: degrees)
theta1 = 0;
theta2 = 15;
theta3 = 0;
%test vector velocities (units:mm/s)
dx = 5;
dy = 5;
dz = 0;

q  = [ theta1;  theta2;  theta3];
pd = [dx; dy; dz];

d = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;

%gets jacobian matrix
J = jacob0(q,DEBUG);
Jp = J(1:3,:);

%calculates the vector of task-space velocities
qd = inv(Jp)*pd;
disp(sprintf('rank = %f', rank(Jp)));
r = rank(Jp);

if r < 3
    disp('close to a singularity. Requested joint velocities are potentially too high.')
    disp(sprintf('Joint 1 velocity: %f', qd(1,1)));
    disp(sprintf('Joint 2 velocity: %f', qd(2,1)));
    disp(sprintf('Joint 3 velocity: %f', qd(3,1)));
    error('Please avoid singularities.');
else if DEBUG
    disp(sprintf('Joint 1 velocity: %f', qd(1,1)));
    disp(sprintf('Joint 2 velocity: %f', qd(2,1)));
    disp(sprintf('Joint 3 velocity: %f', qd(3,1)));
    end
end
