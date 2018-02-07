function R = jacob0 (q, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%test joint angles (units: degrees)
theta1 = 0;
theta2 = 0;
theta3 = 0;
q = [theta1; theta2; theta3];
d = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;

%gets the rotation matricies from frame zero to each joint
R = fwkin3001(q, false, DEBUG);

if DEBUG
    R
    R0 = R(:,   1:4)
    R1 = R(:,   5:8)
    R2 = R(:,  9:12)
    R3 = R(:, 13:16)
end

%initializes Jacobian Matrix components
Jp = zeros(3, size(q, 1));
Jo = zeros(3, size(q, 1));

for y = 1:size(q,1)
    Jp(1:3,y) = cross(R(1:3,y*4+3), (R(1:3,size(q,2))-R(1:3,y*4)));
end

%creates rotational velocities of Jacobian Matrix

for y = 1:size(q,1)
    Jo(:,y) = R(1:3, (y-1)*4+3);
end

J = [Jp; Jo];

if DEBUG
    q
    Jp
    Jo
    J
end



end
