function T = fwkin3001(q,h,d)
DEBUG = d;

%joint angle (degrees)
theta1 = q(1,1);
theta2 = q(2,1);
theta3 = q(3,1);

%joint offset (mm)
d1 = 135;
d2 = 0;
d3 = 0;

%link lenth (mm)
a1 = 0;
a2 = 175;
%a3 = 169.28;
a3 = 180;

%twist angle (degrees)
alpha1 = -90;
alpha2 = 0;
alpha3 = 0;

A1 = [cosd(theta1),            0,  sind(theta1),                0;
      sind(theta1),            0, -cosd(theta1),                0;
                 0,            1,             0,               d1;
                 0,            0,             0,                1];
            
A2 = [cosd(theta2), -sind(theta2),            0,  a2*cosd(theta2);
      sind(theta2),  cosd(theta2),            0,  a2*sind(theta2);
                 0,             0,            1,                0;
                 0,             0,            0,                1];
            
A3 = [cosd(theta3), -sind(theta3),            0,  a3*cosd(theta3);
      sind(theta3),  cosd(theta3),            0,  a3*sind(theta3);
                 0,             0,            1,                0;
                 0,             0,            0,                1];

if DEBUG
    disp(A1);
    disp(A2);
    disp(A3);
end

%returns the homogenous transform matrix from frame zero to the EOAT
if h
    H = A1*A2*A3;
    T = H(1:3,4);

%returns a matrix with the homogenous transform matricies from frame zero
%to each of the joints
else
    T = zeros(4,16);
    T(:,1:4) = eye(4);
    T(:,5:8) = A1;
    T(:,9:12) = A1*A2;
    T(:,13:16) = A1*A2*A3;
end

end