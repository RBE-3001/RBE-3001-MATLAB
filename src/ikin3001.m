function S = ikin3001(p)
try
    %test point
    %p = ([2000;100;135]);
    
    px = p(1,1);
    py = p(2,1);
    pz = p(3,1);
    
    %Links (mm)
    L1 = 135;
    L2 = 175;
    L3 = 169.28;
    L4 = sqrt((pz-L1)^2+(px)^2+(py)^2);
    
    theta1 = atan2d(px,py);
    theta2 = atan2d(pz-L1, (sqrt((px)^2+(py)^2)+acosd(((L2)^2+(L4)^2-(L3)^2)/(2*L2*L4))));
    theta3 = acosd((-(L2)^2-(L3)^2+(L4)^2)/(2*L2*L3));

    S = [theta1 ; theta2; theta3];
catch
    %input errors lead to this message
    message = sprintf('Desired position x=%f, y=%f, z=%f, is out of reach.',px, py, pz);
    error(message);
end