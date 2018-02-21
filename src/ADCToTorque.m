function T = ADCToTorque(v, a, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
%ADC value to be converted to torque
v = [0.05;
     0.02;
     0.04];

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
        y = -0.5760*k;
     
    case 2
        k = 178.5;
        y = -0.4610*k;
                       
    case 3
        k = 178.5;
        y = -0.4835*k;
       
end

%initializes size of output matrix
T = zeros(size(v,1),1);

%converts each raw ADC value to applied torque (Nm)
for i = 1:size(v,1)
    T(i,1) = k*v(i,1) + y;

    if DEBUG
        disp(sprintf('T = %f, k = %f, v = %f, y = %f', T(i,1), k, v(i,1), y));
    end
end

end