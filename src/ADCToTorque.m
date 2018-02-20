function T = ADCToTorque(v, a, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%ADC value to be converted to torque
v = [0.05];

%specifies which joint 
a = 1;

%debug
d = true;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;

%Sets the scaling factor k and the offset y based on which joint is
%specified by a: 1 = base, 2 = elbow, 3 = wrist
switch (a)
    case 1
        k = 178.5;
        y = 0;
        
    case 2
        k = 178.5;
        y = 10;
        
    case 3
        k = 178.5;
        y = 100;
end

%converts the raw ADC value to applied torque (Nm)
T = k*v + y;

if DEBUG
    disp(sprintf('T = %f, k = %f, v = %f, y = %f', T, k, v, y));
end

end