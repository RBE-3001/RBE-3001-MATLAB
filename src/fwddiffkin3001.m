function pd = fwddiffkin3001 (q, qd, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
%test joint angles (units: degrees)
theta1 = 0;
theta2 = 15;
theta3 = 0;
%test joint velocities (units:degrees/s)
dtheta1 = 5;
dtheta2 = 5;
dtheta3 = 0;

q  = [ theta1;  theta2;  theta3];
qd = [dtheta1; dtheta2; dtheta3];

d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;

%gets jacobian matrix
J = jacob0(q,DEBUG);
Jp = J(1:3,:);

%calculates the vector of task-space velocities
pd = inv(Jp)*qd;

end
