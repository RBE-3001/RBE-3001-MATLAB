function S = ikin3001(p,d)
try
    %d = true;
    DEBUG = d;
    
    %test point (x, y, z) (units: mm)
    %p = ([300;100;135]);
    
    px = p(1,1);
    py = p(2,1);
    pz = p(3,1);
    
    %Links (mm)
    L1 = 135;
    L2 = 175;
    %L3 = 169.28;
    L3 = 180;
    L4 = sqrt((pz-L1)^2+(px)^2+(py)^2);
    
    theta1 = atan2d(py,px);
    theta2 = atan2d( ( pz-L1 ), ( sqrt( (px)^2 + (py)^2 ) ) ) + acosd( ( (L2)^2 + (L4)^2 - (L3)^2 ) / (2*L2*L4));        
    theta3 = -acosd(-((L3)^2+(L2)^2-(L4)^2)/(2*L2*L3));
        
    if DEBUG
        disp(sprintf('theta1 = %f, theta2 = %f, theta3 = %f', theta1, theta2, theta3));
    end
    
    if theta1 > 180 || theta1 < -180
        error('Theta1 is out of bounds');
        error('Desired position x = %f, y = %f, z = %f, is out of reach.',px, py, pz);
    elseif theta2 > 200 || theta2 < -10
        error('Theta2 is out of bounds');
        error('Desired position x = %f, y = %f, z = %f, is out of reach.',px, py, pz);
    elseif theta3 > 200 || theta3 < -45
        error('Theta3 is out of bounds');
        error('Desired position x = %f, y = %f, z = %f, is out of reach.',px, py, pz);
    else
        S = [theta1 ; theta2; theta3];
    end
    
catch
    %input errors lead to this message
    error('Desired position x = %f, y = %f, z = %f, is out of reach.',px, py, pz);
end